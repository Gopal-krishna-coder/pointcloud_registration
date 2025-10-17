#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os
import threading
import numpy as np
import open3d as o3d
from collections import deque
import struct
import ctypes

import tf
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

from pointcloud_registration.srv import SaveAccumulatedCloud, SaveAccumulatedCloudResponse

def ros_pc2_to_o3d(ros_pc2, save_color=False):
    """
    Utility function to convert ROS PointCloud2 to Open3D PointCloud.
    Now handles color information if 'rgb' field is present and save_color is True.
    """
    # Check if we should process color
    has_color = False
    if save_color:
        field_names = [field.name for field in ros_pc2.fields]
        if 'rgb' in field_names:
            has_color = True
            rospy.logdebug_once("Color information found in PointCloud2 topic. Processing color.")
        else:
            rospy.logwarn_once("'save_color' is True, but 'rgb' field not found in PointCloud2 message. Saving XYZ only.")

    fields_to_read = ["x", "y", "z"]
    if has_color:
        fields_to_read.append("rgb")
    
    point_generator = pc2.read_points(ros_pc2, skip_nans=True, field_names=fields_to_read)
    points_list = list(point_generator)

    o3d_pc = o3d.geometry.PointCloud()
    if not points_list:
        return o3d_pc

    if has_color:
        xyz = np.array([[p[0], p[1], p[2]] for p in points_list])
        
        rgb_packed = [p[3] for p in points_list]
        rgb_colors = []
        for packed_float in rgb_packed:
            # Use struct to unpack the float into bytes, then reinterpret as an integer
            s = struct.pack('>f', packed_float)
            i = struct.unpack('>l', s)[0]
            # Bitwise operations to extract R, G, B channels
            r = (i >> 16) & 0xFF
            g = (i >> 8) & 0xFF
            b = i & 0xFF
            # Normalize to [0, 1] for Open3D
            rgb_colors.append([r / 255.0, g / 255.0, b / 255.0])
        
        colors = np.array(rgb_colors)
        o3d_pc.points = o3d.utility.Vector3dVector(xyz)
        o3d_pc.colors = o3d.utility.Vector3dVector(colors)
    else:
        xyz = np.array([[p[0], p[1], p[2]] for p in points_list])
        o3d_pc.points = o3d.utility.Vector3dVector(xyz)
        
    return o3d_pc

class PointCloudAccumulatorNode:
    def __init__(self):
        rospy.init_node('pointcloud_accumulator_node', anonymous=True)

        self.save_path = rospy.get_param('~save_path', 'pcd_files')
        self.pointcloud_topic = rospy.get_param('~pointcloud_topic', '/camera/depth_registered/points')
        self.accumulation_duration = rospy.Duration(rospy.get_param('~accumulation_seconds', 1.0))
        self.voxel_size = rospy.get_param('~voxel_size', 0.005)
        self.save_color = rospy.get_param('~save_color', False)

        self.point_cloud_buffer = deque()
        self.lock = threading.Lock()
        self.tf_listener = tf.TransformListener()

        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)
            rospy.loginfo("Created directory: %s", self.save_path)

        self.pc_subscriber = rospy.Subscriber(self.pointcloud_topic, PointCloud2, self.pc_callback, queue_size=10)
        self.save_service = rospy.Service('~save_cloud', SaveAccumulatedCloud, self.handle_save_request)

        rospy.loginfo("Point cloud accumulator is ready.")
        rospy.loginfo("Listening on topic: %s", self.pointcloud_topic)
        rospy.loginfo("Saving to directory: %s", self.save_path)
        rospy.loginfo("Save color point clouds: %s", "Enabled" if self.save_color else "Disabled")


    def pc_callback(self, msg):
        with self.lock:
            self.point_cloud_buffer.append((msg.header.stamp, msg))
            
            while self.point_cloud_buffer and \
                  (msg.header.stamp - self.point_cloud_buffer[0][0]) > self.accumulation_duration:
                self.point_cloud_buffer.popleft()

    def handle_save_request(self, req):
        if not req.filename:
            return SaveAccumulatedCloudResponse(success=False, message="Filename cannot be empty.")
        
        filename = req.filename if req.filename.endswith('.pcd') else req.filename + '.pcd'
        full_path = os.path.join(self.save_path, filename)
        
        rospy.loginfo("Service called to save accumulated cloud to %s", full_path)
        
        is_default_bbox = (req.min_x == 0.0 and req.max_x == 0.0 and \
                           req.min_y == 0.0 and req.max_y == 0.0 and \
                           req.min_z == 0.0 and req.max_z == 0.0)

        if is_default_bbox:
            min_bound = (-2.0, -2.0, -2.0)
            max_bound = (2.0, 2.0, 2.0)
        else:
            min_bound = (req.min_x, req.min_y, req.min_z)
            max_bound = (req.max_x, req.max_y, req.max_z)
        
        rospy.loginfo("Detail informations:")
        rospy.loginfo(f"\tBounding Box: x=[{min_bound[0], max_bound[0]}], y=[{min_bound[1], max_bound[1]}], z=[{min_bound[2], max_bound[2]}]")
        rospy.loginfo(f"\tFrame: {req.frame}")
        rospy.loginfo(f"\tDuration: {self.accumulation_duration:.3f} seconds.")

        clouds_to_process = []
        with self.lock:
            if not self.point_cloud_buffer:
                message = "No point clouds in the buffer to save, topic=[{}]".format(self.pointcloud_topic)
                rospy.logwarn(message)
                return SaveAccumulatedCloudResponse(success=False, message=message)
            
            clouds_to_process = [msg for _, msg in self.point_cloud_buffer]

        accumulated_cloud_o3d = o3d.geometry.PointCloud()
        for ros_pc2_msg in clouds_to_process:
            o3d_pc = ros_pc2_to_o3d(ros_pc2_msg, self.save_color)
            accumulated_cloud_o3d += o3d_pc
        
        if not accumulated_cloud_o3d.has_points():
            message = "Accumulated point cloud is empty."
            rospy.logwarn(message)
            return SaveAccumulatedCloudResponse(success=False, message=message)
        
        cloud_to_process = accumulated_cloud_o3d
        source_frame = clouds_to_process[0].header.frame_id
        target_frame = req.frame

        if target_frame and source_frame and target_frame != source_frame:
            rospy.loginfo("Attempting to transform point cloud from '{}' to '{}'.".format(source_frame, target_frame))
            try:
                self.tf_listener.waitForTransform(target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0))
                (trans, rot) = self.tf_listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
                transform_matrix = self.tf_listener.fromTranslationRotation(trans, rot)
                cloud_to_process.transform(transform_matrix)
                rospy.loginfo("Successfully transformed point cloud.")

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                message = "Failed to get transform from '{}' to '{}': {}".format(source_frame, target_frame, e)
                rospy.logerr(message)
                return SaveAccumulatedCloudResponse(success=False, message=message)
        else:
            rospy.loginfo("Skipping TF transformation. Reason: target_frame or source_frame is empty, or they are the same.")

        rospy.loginfo("Cropping point cloud in frame '{}' with bounding box: min=({:.2f}, {:.2f}, {:.2f}), max=({:.2f}, {:.2f}, {:.2f})".format(
            target_frame if target_frame else source_frame,
            min_bound[0], min_bound[1], min_bound[2], max_bound[0], max_bound[1], max_bound[2]
        ))
        
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=min_bound, max_bound=max_bound)
        cropped_cloud = cloud_to_process.crop(bbox)

        if not cropped_cloud.has_points():
            message = "Point cloud is empty after applying the bounding box filter."
            rospy.logwarn(message)
            return SaveAccumulatedCloudResponse(success=False, message=message)

        rospy.loginfo("Downsampling filtered cloud with voxel size: %.3f", self.voxel_size)
        downsampled_cloud = cropped_cloud.voxel_down_sample(self.voxel_size)

        try:
            o3d.io.write_point_cloud(full_path, downsampled_cloud)
            message = "Successfully saved point cloud with %d points to %s" % (len(downsampled_cloud.points), full_path)
            rospy.loginfo(message)
            return SaveAccumulatedCloudResponse(success=True, message=message)
        except Exception as e:
            message = "Failed to save point cloud: %s" % e
            rospy.logerr(message)
            return SaveAccumulatedCloudResponse(success=False, message=message)

if __name__ == '__main__':
    try:
        PointCloudAccumulatorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass