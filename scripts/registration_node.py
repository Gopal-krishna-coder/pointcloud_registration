#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os
import glob
import numpy as np
import open3d as o3d
import tf2_ros
import tf.transformations as tf_trans
import time

from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped, Transform

from pointcloud_registration.srv import RegisterPointCloud, RegisterPointCloudResponse
from std_srvs.srv import Trigger, TriggerResponse

def o3d_to_ros_pc2(o3d_pc, frame_id="map"):
    """将 Open3D 点云转换为 ROS PointCloud2 消息"""
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    points_xyz = np.asarray(o3d_pc.points)
    
    import sensor_msgs.point_cloud2 as pc2
    return pc2.create_cloud_xyz32(header, points_xyz)

def ros_pc2_to_o3d(ros_pc2):
    """将 ROS PointCloud2 消息转换为 Open3D 点云"""
    import sensor_msgs.point_cloud2 as pc2
    gen = pc2.read_points(ros_pc2, skip_nans=True, field_names=("x", "y", "z"))
    points = np.array(list(gen))
    
    o3d_pc = o3d.geometry.PointCloud()
    o3d_pc.points = o3d.utility.Vector3dVector(points)
    return o3d_pc

class PointCloudRegistrationNode:
    def __init__(self):
        rospy.init_node('pointcloud_registration_node', anonymous=True)

        # --- 参数 ---
        self.pcd_path = rospy.get_param('~pcd_path', 'pcd_files')
        self.pointcloud_topic = rospy.get_param('~pointcloud_topic', '/camera/depth_registered/points')
        self.write_file = rospy.get_param("~write_file", False)
        self.fitness = rospy.get_param("~fitness", 0.6)
        self.voxel_size = rospy.get_param("~voxel_size", 0.005)
        # 新增：控制配准次数的参数，默认为1
        self.registration_attempts = rospy.get_param("~registration_attempts", 1)

        # --- 内部变量 ---
        self.target_clouds = {}
        self.accumulated_clouds_list = []

        # --- 发布者和订阅者 ---
        self.pub_target = rospy.Publisher('~target_cloud', PointCloud2, queue_size=1, latch=True)
        self.pub_source = rospy.Publisher('~source_cloud', PointCloud2, queue_size=1, latch=True)
        self.pub_aligned = rospy.Publisher('~aligned_cloud', PointCloud2, queue_size=1, latch=True)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # --- TF ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.loginfo("TF listener initialized.")

        # --- 服务 ---
        self.load_point_clouds()
        self.registration_service = rospy.Service('~register', RegisterPointCloud, self.handle_registration_request)
        self.reload_service = rospy.Service('~reload_pcds', Trigger, self.handle_reload_request)

        rospy.loginfo("Point cloud registration node is ready.")
        rospy.loginfo("Call '~register' to align clouds or '~reload_pcds' to refresh target clouds.")

    def load_point_clouds(self):
        self.target_clouds.clear()
        rospy.loginfo("Clearing existing point clouds and reloading from: %s", self.pcd_path)
        
        pcd_files = glob.glob(os.path.join(self.pcd_path, '*.pcd'))
        if not pcd_files:
            rospy.logwarn("No PCD files found in directory: %s", self.pcd_path)
            return
            
        for pcd_file in pcd_files:
            try:
                cloud = o3d.io.read_point_cloud(pcd_file)
                if not cloud.has_points():
                    rospy.logwarn("Skipping empty point cloud: %s", pcd_file)
                    continue

                cloud_name = os.path.splitext(os.path.basename(pcd_file))[0]
                self.target_clouds[cloud_name] = cloud
                rospy.loginfo("Loaded point cloud: %s", cloud_name)
            except Exception as e:
                rospy.logerr("Failed to load point cloud %s: %s", pcd_file, e)
        
        rospy.loginfo("Finished loading. Total %d target clouds available.", len(self.target_clouds))

    def handle_reload_request(self, req):
        rospy.loginfo("Received request to reload point clouds.")
        try:
            self.load_point_clouds()
            count = len(self.target_clouds)
            message = "Successfully reloaded {} point clouds.".format(count)
            rospy.loginfo(message)
            return TriggerResponse(success=True, message=message)
        except Exception as e:
            message = "An error occurred while reloading point clouds: {}".format(e)
            rospy.logerr(message)
            return TriggerResponse(success=False, message=message)

    def _accumulation_callback(self, msg):
        o3d_pc = ros_pc2_to_o3d(msg)
        if o3d_pc.has_points():
            self.accumulated_clouds_list.append(o3d_pc)

    def handle_registration_request(self, req):
        rospy.loginfo("Received registration request for target: '%s'", req.target_cloud_name)
        if req.target_cloud_name not in self.target_clouds:
            msg = "Target cloud '{}' not found. Available clouds: {}".format(
                req.target_cloud_name, ', '.join(self.target_clouds.keys()))
            rospy.logerr(msg)
            return RegisterPointCloudResponse(success=False, message=msg)
        reg_start_time = time.time()
        
        target_cloud = o3d.geometry.PointCloud(self.target_clouds[req.target_cloud_name])

        rospy.loginfo("Acquiring source point cloud from topic: %s", self.pointcloud_topic)
        source_cloud = o3d.geometry.PointCloud()
        source_cloud_frame_id = ""

        if req.registration_attempts == 0:
            registration_attempts = self.registration_attempts
        else:
            registration_attempts = req.registration_attempts
        if registration_attempts == 0:
            registration_attempts = 1

        try:
            initial_msg = rospy.wait_for_message(self.pointcloud_topic, PointCloud2, timeout=5.0)
            source_cloud_frame_id = initial_msg.header.frame_id
            rospy.loginfo("Source cloud frame_id is '%s'. All clouds will be published in this frame.", source_cloud_frame_id)

            if req.accumulation_seconds <= 0:
                rospy.loginfo("Capturing a single frame for the source cloud.")
                source_cloud = ros_pc2_to_o3d(initial_msg)
            else:
                rospy.loginfo("Accumulating source clouds for %.2f seconds.", req.accumulation_seconds)
                self.accumulated_clouds_list = [ros_pc2_to_o3d(initial_msg)]
                sub = rospy.Subscriber(self.pointcloud_topic, PointCloud2, self._accumulation_callback)
                rospy.sleep(req.accumulation_seconds)
                sub.unregister()
                rospy.loginfo("Accumulated %d clouds.", len(self.accumulated_clouds_list))
                if not self.accumulated_clouds_list: raise Exception("No point clouds were accumulated.")
                merged_cloud = o3d.geometry.PointCloud()
                for pc in self.accumulated_clouds_list: merged_cloud += pc
                source_cloud = merged_cloud

            if not source_cloud.has_points(): raise Exception("Source cloud is empty after capture.")

        except Exception as e:
            msg = "Failed to get source point cloud: %s" % e
            rospy.logerr(msg)
            return RegisterPointCloudResponse(success=False, message=msg)

        rospy.loginfo("Publishing original target and source clouds.")
        self.pub_target.publish(o3d_to_ros_pc2(target_cloud, source_cloud_frame_id))
        self.pub_source.publish(o3d_to_ros_pc2(source_cloud, source_cloud_frame_id))

        # --- 多次配准与平均 ---
        successful_translations = []
        successful_quaternions = []
        
        # 确定实际执行次数
        rospy.loginfo("Starting registration process for %d attempt(s).", registration_attempts)
        
        for attempt in range(registration_attempts):
            rospy.loginfo("--- Registration Attempt %d/%d ---", attempt + 1, registration_attempts)
            
            # 预处理
            is_default_box = all(v == 0.0 for v in [req.crop_min_x, req.crop_max_x, req.crop_min_y, req.crop_max_y, req.crop_min_z, req.crop_max_z])
            min_bound = [-2.0, -2.0, -2.0] if is_default_box else [req.crop_min_x, req.crop_min_y, req.crop_min_z]
            max_bound = [2.0, 2.0, 2.0] if is_default_box else [req.crop_max_x, req.crop_max_y, req.crop_max_z]
            bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=min_bound, max_bound=max_bound)
            
            # 每次循环都从原始点云开始裁剪，避免状态污染
            source_cloud_processed = source_cloud.crop(bbox)
            target_cloud_processed = target_cloud.crop(bbox)
            
            if not source_cloud_processed.has_points() or not target_cloud_processed.has_points():
                rospy.logerr("Attempt %d failed: One or both clouds are empty after cropping.", attempt + 1)
                continue
            
            voxel_size = req.voxel_size if req.voxel_size > 0 else self.voxel_size
            source_down = source_cloud_processed.voxel_down_sample(voxel_size)
            target_down = target_cloud_processed.voxel_down_sample(voxel_size)

            search_param = o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30)
            source_down.estimate_normals(search_param)
            target_down.estimate_normals(search_param)

            # ICP 配准
            max_correspondence_distance = req.max_correspondence_distance if req.max_correspondence_distance > 0 else voxel_size * 5
            reg_result = o3d.pipelines.registration.registration_icp(
                source_down, target_down, max_correspondence_distance, np.identity(4),
                o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))

            rospy.loginfo("Attempt %d finished. Fitness: %.4f, RMSE: %.4f", attempt + 1, reg_result.fitness, reg_result.inlier_rmse)
            
            if reg_result.fitness >= self.fitness:
                rospy.loginfo("Attempt %d successful.", attempt + 1)
                trans = tf_trans.translation_from_matrix(reg_result.transformation)
                quat = tf_trans.quaternion_from_matrix(reg_result.transformation)
                successful_translations.append(trans)
                successful_quaternions.append(quat)
            else:
                rospy.logwarn("Attempt %d failed: fitness score is below threshold (%.4f).", attempt + 1, self.fitness)

        # --- 处理所有尝试的结果 ---
        if not successful_translations:
            msg = "All {} registration attempts failed to meet the fitness threshold.".format(registration_attempts)
            rospy.logerr(msg)
            return RegisterPointCloudResponse(success=False, message=msg)

        rospy.loginfo("Averaging results from %d successful registration(s).", len(successful_translations))

        # 平均平移
        avg_trans = np.mean(np.array(successful_translations), axis=0)

        # 平均四元数
        quats_array = np.array(successful_quaternions)
        # 将所有四元数对齐到与第一个四元数相同的半球，以进行正确的平均
        for i in range(1, len(quats_array)):
            if np.dot(quats_array[0], quats_array[i]) < 0:
                quats_array[i] *= -1
        avg_quat = np.mean(quats_array, axis=0)
        avg_quat /= np.linalg.norm(avg_quat) # 归一化

        # 从平均后的平移和旋转构建最终的变换矩阵
        avg_trans_matrix = tf_trans.translation_matrix(avg_trans)
        avg_rot_matrix = tf_trans.quaternion_matrix(avg_quat)
        transformation_matrix = np.dot(avg_trans_matrix, avg_rot_matrix)

        rospy.loginfo("Final Averaged Transformation Matrix:\n%s", str(transformation_matrix))

        source_cloud_aligned = o3d.geometry.PointCloud(source_cloud)
        source_cloud_aligned.transform(transformation_matrix)

        rospy.loginfo("Publishing aligned cloud based on averaged transformation.")
        self.pub_aligned.publish(o3d_to_ros_pc2(source_cloud_aligned, source_cloud_frame_id))
        
        if self.write_file:
            filename = "aligned_{}_{}.pcd".format(req.target_cloud_name, int(time.time()))
            filepath = os.path.join(self.pcd_path, filename)
            rospy.loginfo("Writing aligned cloud to %s", filepath)
            o3d.io.write_point_cloud(filepath, source_cloud_aligned)

        # 准备服务响应
        transform_msg = Transform()
        transform_msg.translation.x = avg_trans[0]
        transform_msg.translation.y = avg_trans[1]
        transform_msg.translation.z = avg_trans[2]
        transform_msg.rotation.x = avg_quat[0]
        transform_msg.rotation.y = avg_quat[1]
        transform_msg.rotation.z = avg_quat[2]
        transform_msg.rotation.w = avg_quat[3]

        msg = "Registration successful. Averaged from {} successful attempts".format(len(successful_translations))
        rospy.loginfo(msg)
        cost_time = time.time() - reg_start_time
        rospy.loginfo(f"Total cost time: {cost_time:.3f} seconds." )
        return RegisterPointCloudResponse(success=True, message=msg, transformation=transform_msg, cost_time=cost_time)

if __name__ == '__main__':
    try:
        node = PointCloudRegistrationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass