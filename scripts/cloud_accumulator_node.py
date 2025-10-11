#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os
import threading
import numpy as np
import open3d as o3d
from collections import deque

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

from pointcloud_registration.srv import SaveAccumulatedCloud, SaveAccumulatedCloudResponse

def ros_pc2_to_o3d(ros_pc2):
    gen = pc2.read_points(ros_pc2, skip_nans=True, field_names=("x", "y", "z"))
    points = np.array(list(gen))
    
    o3d_pc = o3d.geometry.PointCloud()
    if points.shape[0] > 0:
        o3d_pc.points = o3d.utility.Vector3dVector(points)
    
    return o3d_pc

class PointCloudAccumulatorNode:
    def __init__(self):
        rospy.init_node('pointcloud_accumulator_node', anonymous=True)

        self.save_path = rospy.get_param('~save_path', 'pcd_files')
        self.pointcloud_topic = rospy.get_param('~pointcloud_topic', '/camera/depth_registered/points')
        self.accumulation_duration = rospy.Duration(rospy.get_param('~accumulation_seconds', 1.0))
        self.voxel_size = rospy.get_param('~voxel_size', 0.01)

        self.point_cloud_buffer = deque()
        self.lock = threading.Lock()

        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)
            rospy.loginfo("Created directory: %s", self.save_path)

        self.pc_subscriber = rospy.Subscriber(self.pointcloud_topic, PointCloud2, self.pc_callback, queue_size=10)
        self.save_service = rospy.Service('~save_cloud', SaveAccumulatedCloud, self.handle_save_request)

        rospy.loginfo("Point cloud accumulator is ready.")
        rospy.loginfo("Listening on topic: %s", self.pointcloud_topic)
        rospy.loginfo("Saving to directory: %s", self.save_path)

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

        clouds_to_process = []
        with self.lock:
            if not self.point_cloud_buffer:
                message = f"No point clouds in the buffer to save, topic=[{self.pointcloud_topic}]"
                rospy.logwarn(message)
                return SaveAccumulatedCloudResponse(success=False, message=message)
            
            clouds_to_process = [msg for _, msg in self.point_cloud_buffer]

        accumulated_cloud_o3d = o3d.geometry.PointCloud()
        for ros_pc2 in clouds_to_process:
            o3d_pc = ros_pc2_to_o3d(ros_pc2)
            accumulated_cloud_o3d += o3d_pc
        
        if not accumulated_cloud_o3d.has_points():
            message = "Accumulated point cloud is empty."
            rospy.logwarn(message)
            return SaveAccumulatedCloudResponse(success=False, message=message)

        is_default_bbox = (req.min_x == 0.0 and req.max_x == 0.0 and \
                           req.min_y == 0.0 and req.max_y == 0.0 and \
                           req.min_z == 0.0 and req.max_z == 0.0)

        if is_default_bbox:
            rospy.loginfo("No bounding box specified. Applying default box: [-1, 1] for all axes.")
            min_bound = (-1.0, -1.0, -1.0)
            max_bound = (1.0, 1.0, 1.0)
        else:
            rospy.loginfo("Using user-specified bounding box.")
            min_bound = (req.min_x, req.min_y, req.min_z)
            max_bound = (req.max_x, req.max_y, req.max_z)

        rospy.loginfo("Cropping point cloud with bounding box: min_bound=({:.2f}, {:.2f}, {:.2f}), max_bound=({:.2f}, {:.2f}, {:.2f})".format(
            min_bound[0], min_bound[1], min_bound[2], max_bound[0], max_bound[1], max_bound[2]
        ))
        
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=min_bound, max_bound=max_bound)
        cropped_cloud = accumulated_cloud_o3d.crop(bbox)

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