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
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    points_xyz = np.asarray(o3d_pc.points)
    
    import sensor_msgs.point_cloud2 as pc2
    return pc2.create_cloud_xyz32(header, points_xyz)

def ros_pc2_to_o3d(ros_pc2):
    import sensor_msgs.point_cloud2 as pc2
    gen = pc2.read_points(ros_pc2, skip_nans=True, field_names=("x", "y", "z"))
    points = np.array(list(gen))
    
    o3d_pc = o3d.geometry.PointCloud()
    o3d_pc.points = o3d.utility.Vector3dVector(points)
    return o3d_pc

class PointCloudRegistrationNode:
    def __init__(self):
        rospy.init_node('pointcloud_registration_node', anonymous=True)

        self.pcd_path = rospy.get_param('~pcd_path', 'pcd_files')
        self.pointcloud_topic = rospy.get_param('~pointcloud_topic', '/camera/depth_registered/points')
        self.write_file = rospy.get_param("~write_file", False)
        self.fitness = rospy.get_param("~fitness", 0.6)
        self.voxel_size = rospy.get_param("~voxel_size", 0.005)

        self.target_clouds = {}
        self.accumulated_clouds_list = []

        self.pub_target = rospy.Publisher('~target_cloud', PointCloud2, queue_size=1, latch=True)
        self.pub_source = rospy.Publisher('~source_cloud', PointCloud2, queue_size=1, latch=True)
        self.pub_aligned = rospy.Publisher('~aligned_cloud', PointCloud2, queue_size=1, latch=True)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.loginfo("TF listener initialized.")

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
        """订阅器的回调函数，将接收到的点云消息转换为o3d格式并添加到列表中"""
        o3d_pc = ros_pc2_to_o3d(msg)
        if o3d_pc.has_points():
            self.accumulated_clouds_list.append(o3d_pc)

    def _transform_stamped_to_matrix(self, transform):
        """将 geometry_msgs/TransformStamped 转换为 4x4 的 numpy 矩阵"""
        t = transform.transform.translation
        q = transform.transform.rotation
        trans_matrix = tf_trans.translation_matrix((t.x, t.y, t.z))
        rot_matrix = tf_trans.quaternion_matrix((q.x, q.y, q.z, q.w))
        return np.dot(trans_matrix, rot_matrix)

    def handle_registration_request(self, req):
        """
        处理点云配准请求的核心函数。
        1. 获取并验证目标点云。
        2. 从ROS话题采集源点云（单帧或累积）。
        3. 发布原始点云以供可视化。
        4. 对点云进行预处理（裁剪、降采样、法线估计）。
        5. 执行ICP配准。
        6. 发布配准后的点云并返回变换矩阵。
        """
        rospy.loginfo("Received registration request for target: '%s'", req.target_cloud_name)
        if req.target_cloud_name not in self.target_clouds:
            msg = "Target cloud '{}' not found. Available clouds: {}".format(
                req.target_cloud_name, ', '.join(self.target_clouds.keys()))
            rospy.logerr(msg)
            return RegisterPointCloudResponse(success=False, message=msg)
        
        target_cloud = o3d.geometry.PointCloud(self.target_clouds[req.target_cloud_name])

        rospy.loginfo("Acquiring source point cloud from topic: %s", self.pointcloud_topic)
        source_cloud = o3d.geometry.PointCloud()
        source_cloud_frame_id = ""

        try:
            # 首先，等待一条消息以获取其 frame_id，后续所有发布都将使用此 frame
            rospy.loginfo("Waiting for one message to determine frame_id...")
            initial_msg = rospy.wait_for_message(self.pointcloud_topic, PointCloud2, timeout=5.0)
            source_cloud_frame_id = initial_msg.header.frame_id
            rospy.loginfo("Source cloud frame_id is '%s'. All clouds will be published in this frame.", source_cloud_frame_id)

            if req.accumulation_seconds <= 0:
                rospy.loginfo("Capturing a single frame for the source cloud.")
                source_cloud = ros_pc2_to_o3d(initial_msg)
            else:
                rospy.loginfo("Accumulating source clouds for %.2f seconds.", req.accumulation_seconds)
                self.accumulated_clouds_list = []
                
                sub = rospy.Subscriber(self.pointcloud_topic, PointCloud2, self._accumulation_callback)
                rospy.sleep(req.accumulation_seconds)
                sub.unregister()

                rospy.loginfo("Accumulated %d clouds.", len(self.accumulated_clouds_list))
                if not self.accumulated_clouds_list:
                    raise Exception("No point clouds were accumulated.")

                # 将所有累积的点云合并成一个
                merged_cloud = o3d.geometry.PointCloud()
                for pc in self.accumulated_clouds_list:
                    merged_cloud += pc
                source_cloud = merged_cloud

            if not source_cloud.has_points():
                raise Exception("Source cloud is empty after capture.")

        except Exception as e:
            msg = "Failed to get source point cloud: %s" % e
            rospy.logerr(msg)
            return RegisterPointCloudResponse(success=False, message=msg)

        rospy.loginfo("Publishing original target and source clouds.")
        self.pub_target.publish(o3d_to_ros_pc2(target_cloud, source_cloud_frame_id))
        self.pub_source.publish(o3d_to_ros_pc2(source_cloud, source_cloud_frame_id))

        rospy.loginfo("Preprocessing clouds...")
        
        is_default_box = all(v == 0.0 for v in [
            req.crop_min_x, req.crop_max_x, req.crop_min_y, req.crop_max_y,
            req.crop_min_z, req.crop_max_z
        ])
        if is_default_box:
            rospy.loginfo("Using default crop box [-2, 2, -2, 2, -2, 2].")
            min_bound, max_bound = ([-2.0, -2.0, -2.0], [2.0, 2.0, 2.0])
        else:
            rospy.loginfo("Using user-defined crop box.")
            min_bound = [req.crop_min_x, req.crop_min_y, req.crop_min_z]
            max_bound = [req.crop_max_x, req.crop_max_y, req.crop_max_z]
            
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=min_bound, max_bound=max_bound)
        source_cloud_processed = source_cloud.crop(bbox)
        target_cloud_processed = target_cloud.crop(bbox)
        
        if not source_cloud_processed.has_points() or not target_cloud_processed.has_points():
            msg = "One or both clouds are empty after cropping. Adjust crop box."
            rospy.logerr(msg)
            return RegisterPointCloudResponse(success=False, message=msg)
            
        if req.voxel_size <= 0:
            voxel_size = self.voxel_size
        else:
            voxel_size = req.voxel_size
        
        rospy.loginfo("Downsampling with voxel size: %.4f", voxel_size)
        source_down = source_cloud_processed.voxel_down_sample(voxel_size)
        target_down = target_cloud_processed.voxel_down_sample(voxel_size)

        rospy.loginfo("Estimating normals...")
        search_param = o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30)
        source_down.estimate_normals(search_param)
        target_down.estimate_normals(search_param)

        rospy.loginfo("Performing ICP registration...")
        trans_init = np.identity(4)
        
        max_correspondence_distance = req.max_correspondence_distance
        if max_correspondence_distance <= 0:
            default_dist = voxel_size * 5 
            rospy.logwarn(
                "Invalid max_correspondence_distance (%.4f) received. "
                "It must be positive. Using a default value of %.4f instead.",
                max_correspondence_distance, default_dist
            )
            max_correspondence_distance = default_dist
        
        reg_result = o3d.pipelines.registration.registration_icp(
            source_down, target_down, max_correspondence_distance, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))

        transformation_matrix = reg_result.transformation
        rospy.loginfo("ICP Registration finished.")
        rospy.loginfo("Fitness: %.4f", reg_result.fitness)
        rospy.loginfo("Inlier RMSE: %.4f", reg_result.inlier_rmse)
        rospy.loginfo("Transformation Matrix:\n%s", str(transformation_matrix))

        if reg_result.fitness < self.fitness:
            msg = "Registration failed: fitness score (%.4f) is below threshold (%.4f)." % (reg_result.fitness, self.fitness)
            rospy.logwarn(msg)
            return RegisterPointCloudResponse(success=False, message=msg)

        source_cloud_aligned = o3d.geometry.PointCloud(source_cloud)
        source_cloud_aligned.transform(transformation_matrix)

        rospy.loginfo("Publishing aligned cloud.")
        self.pub_aligned.publish(o3d_to_ros_pc2(source_cloud_aligned, source_cloud_frame_id))
        
        if self.write_file:
            filename = "aligned_{}_{}.pcd".format(req.target_cloud_name, int(time.time()))
            filepath = os.path.join(self.pcd_path, filename)
            rospy.loginfo("Writing aligned cloud to %s", filepath)
            o3d.io.write_point_cloud(filepath, source_cloud_aligned)

        trans = tf_trans.translation_from_matrix(transformation_matrix)
        quat = tf_trans.quaternion_from_matrix(transformation_matrix)

        transform_msg = Transform()
        transform_msg.translation.x = trans[0]
        transform_msg.translation.y = trans[1]
        transform_msg.translation.z = trans[2]
        transform_msg.rotation.x = quat[0]
        transform_msg.rotation.y = quat[1]
        transform_msg.rotation.z = quat[2]
        transform_msg.rotation.w = quat[3]

        msg = "Registration successful with fitness %.4f." % reg_result.fitness
        rospy.loginfo(msg)
        return RegisterPointCloudResponse(success=True, message=msg, transformation=transform_msg)
    

if __name__ == '__main__':
    try:
        node = PointCloudRegistrationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass