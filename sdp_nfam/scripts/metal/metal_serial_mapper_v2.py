#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
metal_serial_mapper.py
------------------------------------------------------------
Reads metal-detector hits from Arduino Nano over serial and
places markers in /map at the coil position.

Expected serial line examples from Nano:
  METAL,1,723
  HIT,812
  1,700
  0,512

ROS params:
  ~port                 (/dev/ttyUSB1)
  ~baud                 (115200)
  ~frame_base           (/base_footprint)
  ~frame_map            (/map)
  ~coil_forward_offset  (0.05)   # metres in robot x
  ~coil_left_offset     (0.41)   # metres in robot y (left is +)
  ~coil_z               (0.03)
  ~hit_threshold        (600)    # used if only ADC is sent
  ~debounce_hits        (2)
  ~min_marker_separation(0.20)
  ~marker_topic         (/metal_detections)
  ~csv_path             (~/.ros/metal_hits.csv)
"""
import os
import csv
import math
import rospy
import serial
import tf

from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

class MetalSerialMapper(object):
    def __init__(self):
        rospy.init_node('metal_serial_mapper', anonymous=False)

        self.port = rospy.get_param('~port', '/dev/ttyUSB1')
        self.baud = int(rospy.get_param('~baud', 115200))
        self.frame_base = rospy.get_param('~frame_base', '/base_footprint')
        self.frame_map = rospy.get_param('~frame_map', '/map')

        self.coil_fx = float(rospy.get_param('~coil_forward_offset', 0.05))
        self.coil_fy = float(rospy.get_param('~coil_left_offset', 0.41))
        self.coil_z  = float(rospy.get_param('~coil_z', 0.03))

        self.hit_threshold = int(rospy.get_param('~hit_threshold', 600))
        self.debounce_hits = int(rospy.get_param('~debounce_hits', 2))
        self.min_sep = float(rospy.get_param('~min_marker_separation', 0.20))
        self.csv_path = os.path.expanduser(rospy.get_param('~csv_path', '~/.ros/metal_hits.csv'))

        self.tf_listener = tf.TransformListener()
        self.marker_pub = rospy.Publisher('/metal_detections', MarkerArray, queue_size=1)

        self.markers = []
        self.last_xy = None
        self.consecutive_hits = 0
        self.marker_id = 0
        self.ser = None

        self._prepare_csv()
        self._open_serial()

    def _prepare_csv(self):
        directory = os.path.dirname(self.csv_path)
        if directory and not os.path.exists(directory):
            os.makedirs(directory)
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, 'w') as f:
                writer = csv.writer(f)
                writer.writerow(['stamp', 'map_x', 'map_y', 'map_z', 'raw_line'])

    def _open_serial(self):
        rospy.loginfo('Opening serial: %s @ %d', self.port, self.baud)
        self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
        rospy.sleep(2.0)
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass

    def _get_map_pose(self):
        self.tf_listener.waitForTransform(self.frame_map, self.frame_base, rospy.Time(0), rospy.Duration(0.5))
        (t, r) = self.tf_listener.lookupTransform(self.frame_map, self.frame_base, rospy.Time(0))
        x, y = t[0], t[1]
        yaw = tf.transformations.euler_from_quaternion(r)[2]
        return x, y, yaw

    def _coil_map_point(self):
        bx, by, yaw = self._get_map_pose()
        mx = bx + math.cos(yaw) * self.coil_fx - math.sin(yaw) * self.coil_fy
        my = by + math.sin(yaw) * self.coil_fx + math.cos(yaw) * self.coil_fy
        return mx, my, self.coil_z

    def _parse_line(self, line):
        # Returns (is_hit, adc_value_or_none)
        s = line.strip()
        if not s:
            return (False, None)

        upper = s.upper()
        parts = [p.strip() for p in s.split(',')]

        ints = []
        for p in parts:
            try:
                ints.append(int(p))
            except Exception:
                pass

        if 'METAL' in upper or 'HIT' in upper:
            if ints:
                if len(ints) >= 2:
                    return (ints[0] > 0, ints[-1])
                return (True, ints[0])
            return (True, None)

        if len(ints) >= 2:
            # format like: 1,723
            return (ints[0] > 0, ints[1])

        if len(ints) == 1:
            # support simple digital format from current sketch: 1 / 0
            if ints[0] in (0, 1):
                return (ints[0] == 1, ints[0])
            # otherwise treat it as ADC-only format
            return (ints[0] >= self.hit_threshold, ints[0])

        return (False, None)

    def _far_enough(self, x, y):
        if self.last_xy is None:
            return True
        dx = x - self.last_xy[0]
        dy = y - self.last_xy[1]
        return math.sqrt(dx*dx + dy*dy) >= self.min_sep

    def _append_marker(self, x, y, z, raw_line):
        marker = Marker()
        marker.header = Header(frame_id=self.frame_map, stamp=rospy.Time.now())
        marker.ns = 'metal_hits'
        marker.id = self.marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.scale.x = 0.10
        marker.scale.y = 0.10
        marker.scale.z = 0.10
        marker.color = ColorRGBA(1.0, 0.0, 0.0, 0.9)
        marker.lifetime = rospy.Duration(0)

        self.markers.append(marker)
        self.marker_id += 1
        self.last_xy = (x, y)

        arr = MarkerArray()
        arr.markers = self.markers
        self.marker_pub.publish(arr)

        with open(self.csv_path, 'a') as f:
            writer = csv.writer(f)
            writer.writerow([rospy.Time.now().to_sec(), x, y, z, raw_line.strip()])

        rospy.logwarn('METAL MARKED at map=(%.3f, %.3f, %.3f)', x, y, z)

    def spin(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            try:
                line = self.ser.readline()
                if isinstance(line, bytes):
                    line = line.decode('utf-8', 'ignore')
                hit, adc = self._parse_line(line)

                if hit:
                    self.consecutive_hits += 1
                else:
                    self.consecutive_hits = 0

                if self.consecutive_hits >= self.debounce_hits:
                    try:
                        x, y, z = self._coil_map_point()
                        if self._far_enough(x, y):
                            self._append_marker(x, y, z, line)
                    except Exception as e:
                        rospy.logwarn('Could not transform coil point into map: %s', str(e))
                    self.consecutive_hits = 0

            except serial.SerialException as e:
                rospy.logerr('Serial error: %s', str(e))
                rospy.sleep(1.0)
                try:
                    self._open_serial()
                except Exception:
                    pass
            except Exception as e:
                rospy.logwarn('metal_serial_mapper loop: %s', str(e))

            rate.sleep()

if __name__ == '__main__':
    try:
        MetalSerialMapper().spin()
    except rospy.ROSInterruptException:
        pass
