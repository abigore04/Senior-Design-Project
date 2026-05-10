#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Monitor-only status bridge for SDP Nizami.

Python 2.7 / ROS Melodic compatible.

This node does not control the robot.
It only publishes simple dashboard-friendly status:
  - /metal_detector_status: std_msgs/String, values: connected / unconnected
  - /robot_status: std_msgs/String JSON with odom speed and detector status

It does NOT open the Arduino serial port, so it does not conflict with
metal_serial_mapper_v2.py. It only checks if the configured device path exists.
"""
from __future__ import print_function

import json
import os
import time

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray


class RobotStatusBridge(object):
    def __init__(self):
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyUSB0')
        self.odom_topic = rospy.get_param('~odom_topic', '/odom')
        self.metal_topic = rospy.get_param('~metal_topic', '/metal_detections')
        self.detector_status_topic = rospy.get_param('~detector_status_topic', '/metal_detector_status')
        self.robot_status_topic = rospy.get_param('~robot_status_topic', '/robot_status')
        self.metal_timeout_sec = float(rospy.get_param('~metal_timeout_sec', 5.0))

        self.linear_x = 0.0
        self.angular_z = 0.0
        self.last_odom_time = 0.0
        self.last_metal_msg_time = 0.0
        self.last_metal_raw = ''

        self.detector_pub = rospy.Publisher(self.detector_status_topic, String, queue_size=5)
        self.status_pub = rospy.Publisher(self.robot_status_topic, String, queue_size=5)

        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=10)
        rospy.Subscriber(self.metal_topic, MarkerArray, self._metal_cb, queue_size=10)

        rospy.loginfo('robot_status_bridge started')
        rospy.loginfo('serial_port=%s', self.serial_port)
        rospy.loginfo('odom_topic=%s', self.odom_topic)
        rospy.loginfo('metal_topic=%s', self.metal_topic)

    def _odom_cb(self, msg):
        self.linear_x = msg.twist.twist.linear.x
        self.angular_z = msg.twist.twist.angular.z
        self.last_odom_time = time.time()

    def _metal_cb(self, msg):
        self.last_metal_msg_time = time.time()
        self.last_metal_raw = "METAL_%d" % len(msg.markers)

    def _serial_exists(self):
        return os.path.exists(self.serial_port)

    def spin(self):
        rate = rospy.Rate(2.0)
        while not rospy.is_shutdown():
            now = time.time()
            detector_state = 'connected' if self._serial_exists() else 'unconnected'
            metal_stream_age = None
            if self.last_metal_msg_time > 0:
                metal_stream_age = now - self.last_metal_msg_time

            status = {
                'mode': 'MANUAL_MONITOR',
                'control_source': 'physical_joystick',
                'serial_port': self.serial_port,
                'metal_detector_status': detector_state,
                'metal_stream_age_sec': metal_stream_age,
                'metal_stream_recent': bool(metal_stream_age is not None and metal_stream_age <= self.metal_timeout_sec),
                'last_metal_raw': self.last_metal_raw,
                'linear_x': self.linear_x,
                'angular_z': self.angular_z,
                'odom_recent': bool(self.last_odom_time > 0 and (now - self.last_odom_time) <= 2.0),
            }

            self.detector_pub.publish(String(detector_state))
            self.status_pub.publish(String(json.dumps(status, sort_keys=True)))
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('robot_status_bridge')
    RobotStatusBridge().spin()
