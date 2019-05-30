#!/usr/bin/env python

"""Publish TF messages for testing"""

import geometry_msgs.msg
import math
import numpy as np
import rclpy
import sim_node  # TODO remove in Dashing
import tf2_msgs.msg
import transformations as xf


def rpy_to_q(r, p, y) -> geometry_msgs.msg.Quaternion:
    matrix = xf.euler_matrix(r, p, y, axes='sxyz')
    q = xf.quaternion_from_matrix(matrix)
    return geometry_msgs.msg.Quaternion(w=q[0], x=q[1], y=q[2], z=q[3])  # Note order


def test_to_tf(test) -> geometry_msgs.msg.TransformStamped:
    tf = geometry_msgs.msg.TransformStamped()
    tf.header.frame_id = test[0]
    tf.child_frame_id = test[1]
    tf.transform.translation = geometry_msgs.msg.Vector3(x=test[2], y=test[3], z=test[4])
    tf.transform.rotation = rpy_to_q(r=test[5], p=test[6], y=test[7])
    return tf


def pre_multiply(left: np.ndarray, tf: geometry_msgs.msg.TransformStamped) -> None:
    tfq = tf.transform.rotation
    xfq = [tfq.w, tfq.x, tfq.y, tfq.z]

    right = xf.quaternion_matrix(xfq)
    right = left @ right

    xfq = xf.quaternion_from_matrix(right)
    tfq = geometry_msgs.msg.Quaternion(w=xfq[0], x=xfq[1], y=xfq[2], z=xfq[3])
    tf.transform.rotation = tfq


class TestNode(sim_node.SimNode):

    def __init__(self):

        super().__init__('test_node')

        # test_tree = [
        #     ['map', 'base_link', 0., 0., 0., math.pi, 0., 0.],
        #     ['base_link', 'camera_link', 0., 0., 0., 0., math.pi, 0.],
        #     ['camera_link', 'camera_frame', 0., 0., 0., 0., 0., math.pi],
        #     ['bad_parent', 'base_link', 0., 0., 0., 0., 0., 0.],
        #     ['cycle1', 'cycle2', 0., 0., 0., 0., 0., 0.],
        #     ['cycle2', 'cycle1', 0., 0., 0., 0., 0., 0.],
        #     ['map', 'marker0', 0., 0., 0., 0., 0., 0.]
        # ]

        test_tree = [
            ['map', 'base_link', 0., 0., 1., 0., 0., 0.],
            ['base_link', 'segment1', 1., 0., 0., 0., 0., 0.],
            ['segment1', 'segment2', 0.5, 0., 0., 0., 0., 0.]
        ]

        self._msg = tf2_msgs.msg.TFMessage()
        for test in test_tree:
            self._msg.transforms.append(test_to_tf(test))

        self._tf_pub = self.create_publisher(tf2_msgs.msg.TFMessage, '/tf')
        self.create_timer(1., self.timer_callback)  # This will run on wall time
        self.get_logger().info("test_node running")

    def timer_callback(self):
        # Update timestamps
        stamp = self.now().to_msg()
        for transform in self._msg.transforms:
            transform.header.stamp = stamp

        # Rotate
        m_rotate_some = xf.euler_matrix(0., 0., math.pi / 8)
        m_inverse = xf.inverse_matrix(m_rotate_some)

        pre_multiply(m_rotate_some, self._msg.transforms[0])
        pre_multiply(m_rotate_some, self._msg.transforms[1])
        pre_multiply(m_inverse, self._msg.transforms[2])
        pre_multiply(m_inverse, self._msg.transforms[2])

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
