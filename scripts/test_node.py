#!/usr/bin/env python

"""Publish TF messages for testing"""

import math
import time
# from typing import Dict, List, Tuple

# import numpy as np
import transformations as xf

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time

import tf2_msgs.msg
import geometry_msgs.msg

# TODO move base through a circle
# TODO add a joint with an arm
# TODO add a counter-moving arm
# TODO add the camera + world + urdf + marker, and tie in vlam

# SDF and fiducial_vlam have different coordinate models
t_world_map = xf.quaternion_matrix([math.sqrt(0.5), 0, 0, -math.sqrt(0.5)])


def now() -> Time:
    """Return builtin_interfaces.msg.Time object with the current CPU time"""
    cpu_time = time.time()
    sec = int(cpu_time)
    nanosec = int((cpu_time - sec) * 1e9)
    return Time(sec=sec, nanosec=nanosec)


# Generate a ring of ArUco markers
# Marker format: [marker_num, x, y, z, roll, pitch, yaw]
def gen_ring_of_markers(num_markers, radius, z):
    marker = 0
    angle = 0
    inc = 2 * math.pi / num_markers
    while marker < num_markers:
        yield [marker, radius * math.cos(angle), radius * math.sin(angle), z, angle, -math.pi / 2, 0]
        marker += 1
        angle += inc


def rpy_to_tf_q(r, p, y) -> geometry_msgs.msg.Quaternion:
    # fixed axis to 4x4 matrix
    t_marker_world = xf.euler_matrix(r, p, y, axes='sxyz')

    # rotate into fiducial_vlam map frame
    t_marker_map = t_marker_world @ t_world_map

    # 4x4 matrix to quaternion
    xfq_marker_map = xf.quaternion_from_matrix(t_marker_map)

    # xf quaternion to tf quaternion, note order
    tfq_marker_map = geometry_msgs.msg.Quaternion(w=xfq_marker_map[0], x=xfq_marker_map[1], y=xfq_marker_map[2],
                                                  z=xfq_marker_map[3])
    return tfq_marker_map


def marker_to_tf(marker) -> geometry_msgs.msg.TransformStamped:
    tf = geometry_msgs.msg.TransformStamped()
    tf.header.stamp = now()
    tf.header.frame_id = 'map'
    tf.child_frame_id = 'marker' + str(marker[0])
    tf.transform.translation = geometry_msgs.msg.Vector3(x=marker[1], y=marker[2], z=marker[3])
    tf.transform.rotation = rpy_to_tf_q(r=marker[4], p=marker[5], y=marker[6])
    return tf


test_tree = [
    ['map', 'base_link', 0., 0., 0., 0., 0., 0.],
    ['base_link', 'camera_link', 0., 0., 0., 0., 0., 0.],
    ['camera_link', 'camera_frame', 0., 0., 0., 0., 0., 0.],
    ['bad_parent', 'base_link', 0., 0., 0., 0., 0., 0.],
    ['map', 'marker0', 0., 0., 0., 0., 0., 0.]
]


def test_to_tf(test) -> geometry_msgs.msg.TransformStamped:
    tf = geometry_msgs.msg.TransformStamped()
    tf.header.stamp = now()
    tf.header.frame_id = test[0]
    tf.child_frame_id = test[1]
    tf.transform.translation = geometry_msgs.msg.Vector3(x=test[2], y=test[3], z=test[4])
    tf.transform.rotation = rpy_to_tf_q(r=test[5], p=test[6], y=test[7])
    return tf


class TestNode(Node):

    def __init__(self):
        super().__init__('test_node')

        self._msg = tf2_msgs.msg.TFMessage()
        for test in test_tree:
            self._msg.transforms.append(test_to_tf(test))

        self._tf_pub = self.create_publisher(tf2_msgs.msg.TFMessage, '/tf')
        self.create_timer(1., self.timer_callback)
        self.get_logger().info("test_node running")

    def timer_callback(self):
        # Update timestamps for transforms
        stamp = now()
        for transform in self._msg.transforms:
            transform.header.stamp = stamp

        # Publish transforms
        self._tf_pub.publish(self._msg)


def main(args=None):
    rclpy.init(args=args)
    node = TestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("ctrl-C detected, shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
