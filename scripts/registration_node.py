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
        self.pub_frame_id = rospy.get_param("~pub_frame_id", "map")

        self.target_clouds = {}

        self.pub_target = rospy.Publisher('~target_cloud', PointCloud2, queue_size=1, latch=True)
        self.pub_source = rospy.Publisher('~source_cloud', PointCloud2, queue_size=1, latch=True)
        self.pub_aligned = rospy.Publisher('~aligned_cloud', PointCloud2, queue_size=1, latch=True)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

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

    def handle_registration_request(self, req):
        start_time = time.time()
        response = RegisterPointCloudResponse()

        if req.target_cloud_name not in self.target_clouds:
            response.success = False
            response.message = "Target cloud '{}' not found.".format(req.target_cloud_name)
            rospy.logerr(response.message)
            return response

        target_cloud_o3d = self.target_clouds[req.target_cloud_name]

        try:
            rospy.loginfo("Waiting for a single point cloud message from topic: %s", self.pointcloud_topic)
            source_cloud_ros = rospy.wait_for_message(self.pointcloud_topic, PointCloud2, timeout=5.0)
        except rospy.ROSException as e:
            response.success = False
            response.message = "Failed to get source point cloud: {}".format(e)
            rospy.logerr(response.message)
            return response

        source_cloud_o3d = ros_pc2_to_o3d(source_cloud_ros)

        is_default_crop_box = (req.crop_min_x == 0.0 and req.crop_max_x == 0.0 and \
                               req.crop_min_y == 0.0 and req.crop_max_y == 0.0 and \
                               req.crop_min_z == 0.0 and req.crop_max_z == 0.0)

        if is_default_crop_box:
            rospy.loginfo("Applying default crop box: [-1, 1] for all axes.")
            min_bound = (-1.0, -1.0, -1.0)
            max_bound = (1.0, 1.0, 1.0)
        else:
            rospy.loginfo("Applying user-specified crop box.")
            min_bound = (req.crop_min_x, req.crop_min_y, req.crop_min_z)
            max_bound = (req.crop_max_x, req.crop_max_y, req.crop_max_z)
        
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=min_bound, max_bound=max_bound)
        
        target_cloud_o3d = target_cloud_o3d.crop(bbox)
        source_cloud_o3d = source_cloud_o3d.crop(bbox)
        
        if not source_cloud_o3d.has_points() or not target_cloud_o3d.has_points():
            message = "One or both point clouds are empty after cropping. Adjust crop box or check source data."
            rospy.logerr(message)
            response.success = False
            response.message = message
            return response
        
        rospy.loginfo("Cropping complete. Source points: %d, Target points: %d",
                    len(source_cloud_o3d.points), len(target_cloud_o3d.points))
        if req.voxel_size <= 0:
            voxel_size = 0.05
        else:
            voxel_size = req.voxel_size

        rospy.loginfo("Downsampling point clouds with voxel size: %f", voxel_size)
        target_down = target_cloud_o3d.voxel_down_sample(voxel_size=voxel_size)
        source_down = source_cloud_o3d.voxel_down_sample(voxel_size=voxel_size)

        if not source_down.has_points() or not target_down.has_points():
            message = "Downsampling resulted in an empty point cloud. Check voxel_size or source data."
            rospy.logerr(message)
            response.success = False
            response.message = message
            return response

        rospy.loginfo("Source cloud has %d points, target cloud has %d points after downsampling.",
                    len(source_down.points), len(target_down.points))

        rospy.loginfo("Estimating normals...")
        target_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
        source_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
        
        if not source_down.has_normals() or not target_down.has_normals():
            message = "Failed to estimate normals. Check point cloud density or normal estimation parameters."
            rospy.logerr(message)
            response.success = False
            response.message = message
            return response

        rospy.loginfo("Performing ICP registration...")
        try:
            if req.max_correspondence_distance <= 0:
                max_correspondence_distance = 0.1
            else:
                max_correspondence_distance = req.max_correspondence_distance
            
            icp_result = o3d.pipelines.registration.registration_icp(
                source_down, target_down, max_correspondence_distance,
                np.identity(4),
                o3d.pipelines.registration.TransformationEstimationPointToPlane()
            )
            print(f"ICP cost time: {time.time() - start_time:.3f} s")
        except Exception as e:
            error_message = "An error occurred during ICP: {}".format(e)
            rospy.logerr(error_message)
            response.success = False
            response.message = error_message
            return response

        transformation = icp_result.transformation
        rospy.loginfo("ICP Fitness: %f", icp_result.fitness)
        rospy.loginfo("ICP Inlier RMSE: %f", icp_result.inlier_rmse)

        if self.write_file:
            rospy.loginfo("Writting Source file...")
            full_path = os.path.join(self.pcd_path, "source.pcd")
            open3d.io.write_point_cloud(full_path, source_down)
            rospy.loginfo("Writting Target file...")
            full_path = os.path.join(self.pcd_path, "target.pcd")
            open3d.io.write_point_cloud(full_path, target_down)

        if icp_result.fitness < 0.6:
            message = "ICP fitness score is too low ({}). Registration may be inaccurate.".format(icp_result.fitness)
            rospy.logwarn(message)
            response.success = False
            response.message = message
            return response

        source_cloud_aligned_o3d = source_cloud_o3d.transform(transformation)

        self.pub_target.publish(o3d_to_ros_pc2(target_cloud_o3d, self.pub_frame_id))
        self.pub_source.publish(o3d_to_ros_pc2(source_cloud_o3d, source_cloud_ros.header.frame_id))
        self.pub_aligned.publish(o3d_to_ros_pc2(source_cloud_aligned_o3d, self.pub_frame_id))

        self.broadcast_transform(transformation, self.pub_frame_id, source_cloud_ros.header.frame_id)

        response.success = True
        response.message = "Registration successful with fitness score: {:.4f}".format(icp_result.fitness)

        trans = transformation[:3, 3]
        quat = tf_trans.quaternion_from_matrix(transformation)
        response.transformation.translation.x = trans[0]
        response.transformation.translation.y = trans[1]
        response.transformation.translation.z = trans[2]
        response.transformation.rotation.x = quat[0]
        response.transformation.rotation.y = quat[1]
        response.transformation.rotation.z = quat[2]
        response.transformation.rotation.w = quat[3]

        return response

    def broadcast_transform(self, transformation, parent_frame, child_frame):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        translation = transformation[:3, 3]
        rotation = tf_trans.quaternion_from_matrix(transformation)

        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        t.transform.rotation.x = rotation[0]
        t.transform.rotation.y = rotation[1]
        t.transform.rotation.z = rotation[2]
        t.transform.rotation.w = rotation[3]

        self.tf_broadcaster.sendTransform(t)
        rospy.loginfo("Published transform from '%s' to '%s'", parent_frame, child_frame)


if __name__ == '__main__':
    try:
        node = PointCloudRegistrationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass