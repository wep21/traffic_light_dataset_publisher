#!/usr/bin/env python
#
# Copyright 2020 Tier IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

import os
from os import listdir
from os.path import isfile, join
import sys
import xml.etree.ElementTree as ET

from autoware_perception_msgs.msg import TrafficLightRoi, TrafficLightRoiArray

import cv2

from cv_bridge import CvBridge, CvBridgeError

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CameraInfo, Image

import yaml


def yaml_to_CameraInfo(yaml_fname):
    # Load data from file
    with open(yaml_fname, 'r') as file_handle:
        calib_data = yaml.safe_load(file_handle)
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.width = calib_data['image_width']
    camera_info_msg.height = calib_data['image_height']
    camera_info_msg.k = calib_data['camera_matrix']['data']
    camera_info_msg.d = calib_data['distortion_coefficients']['data']
    camera_info_msg.r = calib_data['rectification_matrix']['data']
    camera_info_msg.p = calib_data['projection_matrix']['data']
    camera_info_msg.distortion_model = calib_data['distortion_model']
    return camera_info_msg


class TrafficLightDatasetPublisher(Node):

    def __init__(self):
        super().__init__('traffic_light_dataset_publisher')

        self._cv_bridge = CvBridge()

        self._image_publisher = self.create_publisher(Image, 'out/image', 1)
        self._camera_info_publisher = self.create_publisher(CameraInfo, 'out/camera_info', 1)
        self._roi_publisher = self.create_publisher(TrafficLightRoiArray, 'out/rois', 1)

        self._period_sec = self.declare_parameter('period_sec', 0.1).value
        self._repeat_frame = self.declare_parameter('repeat_frame', 10).value
        self._count = 0
        self._expand_ratio = self.declare_parameter('expand_ratio', 0.5).value
        self._sort_files = self.declare_parameter('sort_files', True).value
        self._frame_id = self.declare_parameter('frame_id', 'camera').value

        self._image_folder = self.declare_parameter('image_folder', '').value
        if self._image_folder == '' or not os.path.exists(self._image_folder) or \
           not os.path.isdir(self._image_folder):
            self.get_logger().fatal('Invalid Image folder: {}'.format(self._image_publisher))
            sys.exit(0)
        self.get_logger().info('Reading images from {}'.format(self._image_folder))

        self._label_folder = self.declare_parameter('label_folder', '').value
        if self._label_folder == '' or not os.path.exists(self._label_folder) or \
           not os.path.isdir(self._label_folder):
            self.get_logger().fatal('Invalid label folder: {}'.format(self._label_folder))
            sys.exit(0)
        self.get_logger().info('Reading labels from {}'.format(self._label_folder))

        self._camera_info_yaml = self.declare_parameter('camera_info_yaml', '').value
        if self._camera_info_yaml == '' or not os.path.exists(self._camera_info_yaml) or \
           not os.path.isfile(self._camera_info_yaml):
            self.get_logger().fatal('Invalid camera info yaml: {}'.format(self._camera_info_yaml))
            sys.exit(0)
        self.get_logger().info('Reading camera info from {}'.format(self._camera_info_yaml))
        self._files_in_dir = [f for f in listdir(self._image_folder)
                              if isfile(join(self._image_folder, f))]
        self._camera_info = yaml_to_CameraInfo(self._camera_info_yaml)
        if self._sort_files:
            self._files_in_dir.sort()
        self._file_iter = iter(self._files_in_dir)
        self._file = self._file_iter.__next__()
        self.timer = self.create_timer(self._period_sec, self.timer_callback)

    def timer_callback(self):
        if self._count >= self._repeat_frame - 1:
            try:
                self._file = self._file_iter.__next__()
                self._count = 0
            except StopIteration:
                self._file_iter = iter(self._files_in_dir)
                self._file = self._file_iter.__next__()
        if isfile(join(self._image_folder, self._file)):
            try:
                cv_image = cv2.imread(join(self._image_folder, self._file))
                label_tree = ET.parse(join(self._label_folder,
                                           os.path.splitext(self._file)[0] + '.xml'))
                root = label_tree.getroot()
                roi_msg = TrafficLightRoiArray()
                width = int(root.find('size').find('width').text)
                height = int(root.find('size').find('height').text)
                for i, box in enumerate(root.iter('bndbox')):
                    xmin = float(box.find('xmin').text)
                    xmax = float(box.find('xmax').text)
                    ymin = float(box.find('ymin').text)
                    ymax = float(box.find('ymax').text)
                    roi = TrafficLightRoi()
                    roi.id = i
                    roi.roi.width = int(xmax - xmin)
                    roi.roi.height = int(ymax - ymin)
                    expand_size = int(max(roi.roi.width,
                                          roi.roi.height) * self._expand_ratio)
                    roi.roi.x_offset = int(max(0, xmin - expand_size))
                    roi.roi.y_offset = int(max(0, ymin - expand_size))
                    roi.roi.width = int(min(width, xmax + expand_size) - roi.roi.x_offset)
                    roi.roi.height = int(min(height, ymax + expand_size) - roi.roi.y_offset)
                    roi_msg.rois.append(roi)
                if cv_image is not None:
                    ros_msg = self._cv_bridge.cv2_to_imgmsg(cv_image, 'bgr8')
                    ros_msg.header.frame_id = self._frame_id
                    ros_msg.header.stamp = self.get_clock().now().to_msg()
                    roi_msg.header = ros_msg.header
                    self._camera_info.header = ros_msg.header
                    self._camera_info.width = ros_msg.width
                    self._camera_info.height = ros_msg.height
                    self._image_publisher.publish(ros_msg)
                    self._roi_publisher.publish(roi_msg)
                    self._camera_info_publisher.publish(self._camera_info)
                else:
                    self.get_logger().info('Invalid image file {}'.format(join(
                        self._image_folder, self._file)))
                self._count += 1
            except CvBridgeError as e:
                self.get_logger().error(e)


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDatasetPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
