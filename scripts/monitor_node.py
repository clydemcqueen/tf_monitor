#!/usr/bin/env python

"""
ROS2 TF diagnostic tool

Transformation notation:
    t_child_parent is a transform
    vector_child = t_child_parent * vector_parent
    xxx_f_child means xxx is expressed in child frame
    xxx_pose_f_child is equivalent to t_child_xxx
    t_a_c = t_a_b * t_b_c
    e_child_parent is a fixed axis rotation, i.e., roll, pitch, yaw about X, Y, Z, per
    https://www.ros.org/reps/rep-0103.html

transformations.py expresses quaternions as [w x y z], but ROS users expect to see [x y z w]
"""

from typing import Dict, List, Tuple

import geometry_msgs.msg
import numpy as np
import rclpy
import rclpy.node
import rclpy.time
import sim_node  # TODO remove in Dashing
import tf2_msgs.msg
import transformations as xf


def t_to_str(t: List) -> str:
    return f'xyz=({t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f})'


def q_to_str(q: List) -> str:
    return f'xyzw=({q[1]:.2f}, {q[2]:.2f}, {q[3]:.2f}, {q[0]:.2f})'


def e_to_str(r: Tuple[float, float, float]) -> str:
    return f'rpy=({r[0]:.2f}, {r[1]:.2f}, {r[2]:.2f})'


def m_to_str(m: np.ndarray) -> str:
    t = xf.translation_from_matrix(m)
    q = xf.quaternion_from_matrix(m)
    e = xf.euler_from_matrix(m)
    return f'{t_to_str(t)} {q_to_str(q)} {e_to_str(e)}'


def tf_to_matrix(tf: geometry_msgs.msg.TransformStamped) -> np.ndarray:
    q = [tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z]
    m = xf.quaternion_matrix(q)
    m[:3, 3] = [tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z]
    return m


class Transform:

    def __init__(self, tf: geometry_msgs.msg.TransformStamped, parent: 'Transform' = None):
        self._parent_id = tf.header.frame_id
        self._child_id = tf.child_frame_id
        self._m_child_parent = tf_to_matrix(tf)
        self._m_parent_child = xf.inverse_matrix(self._m_child_parent)

        # Compute composite transform
        if parent is None:
            self._root = True
            self._m_child_root = self._m_child_parent
            self._m_root_child = self._m_parent_child
        else:
            self._root = False
            self._m_child_root = self._m_child_parent @ parent._m_child_root
            self._m_root_child = parent._m_root_child @ self._m_parent_child  # Multiply is cheaper than inverse

    # Print this transform
    # Return the count of transforms printed (always 1)
    def print(self, prefix: str = '') -> int:
        # Print transform and inverse
        print(f'{prefix}{self._parent_id} => {self._child_id}:')
        print(f'                      forward:           {m_to_str(self._m_child_parent)}')
        print(f'                      inverse:           {m_to_str(self._m_parent_child)}')

        # Print composite and inverse
        if not self._root:
            print(f'                      composite:         {m_to_str(self._m_child_root)}')
            print(f'                      composite inverse: {m_to_str(self._m_root_child)}')
        return 1

    # Print a tree of transforms
    # Return count of transforms printed
    def print_tree(self,
                   good: Dict[str, geometry_msgs.msg.TransformStamped],
                   prefix: str = '   ') -> int:
        # Print this transform
        num_printed = self.print(prefix)

        # Walk the list of good transforms, looking for transforms child => grandchild
        for tf in good.values():
            if tf.header.frame_id == self._child_id:
                t_grandchild_child = Transform(tf, self)
                num_printed = num_printed + t_grandchild_child.print_tree(good, prefix + '   ')
        return num_printed


class MonitorNode(sim_node.SimNode):

    def __init__(self):
        super().__init__('monitor_node')

        # tf_callback builds a dict of all transforms, keyed by (frame_id, child_frame_id)
        self._transforms: Dict[Tuple[str, str], geometry_msgs.msg.TransformStamped] = {}

        # Most nodes publish on /tf
        self.create_subscription(tf2_msgs.msg.TFMessage, '/tf', self.tf_callback)

        # robot_state_publisher (and others?) publish on /tf_static
        self.create_subscription(tf2_msgs.msg.TFMessage, '/tf_static', self.tf_callback)

        # Walk the TF tree every second
        self.create_timer(1., self.timer_callback)
        self.get_logger().info("monitor_node running")

    def tf_callback(self, msg: tf2_msgs.msg.TFMessage) -> None:
        for tf in msg.transforms:

            # Remove leading '/' from frame ids
            if tf.header.frame_id[0] == '/':
                tf.header.frame_id = tf.header.frame_id[1:]
            if tf.child_frame_id[0] == '/':
                tf.child_frame_id = tf.child_frame_id[1:]

            self._transforms[(tf.header.frame_id, tf.child_frame_id)] = tf

    def timer_callback(self) -> None:
        # Remove stale transforms
        stale_keys: List[Tuple[str, str]] = []
        for key, tf in self._transforms.items():
            if self.now() - rclpy.time.Time.from_msg(tf.header.stamp) > rclpy.time.Duration(seconds=5):
                print(f'!!! {key[0]} => {key[1]} is stale, deleting')
                stale_keys.append(key)
        for key in stale_keys:
            self._transforms.pop(key)

        # Build a dict of good transforms, keyed by child_frame_id
        good: Dict[str, geometry_msgs.msg.TransformStamped] = {}

        for tf in self._transforms.values():

            # Ignore conflicting parents
            if tf.child_frame_id in good:
                print(f'!!! {tf.header.frame_id} => {tf.child_frame_id} conflicts with',
                      f'{good[tf.child_frame_id].header.frame_id} => {tf.child_frame_id}, ignoring')
                continue

            good[tf.child_frame_id] = tf

        # Build a list of root transforms
        # Cycles don't have a root, so they will be unreachable
        roots: List[geometry_msgs.msg.TransformStamped] = []

        for tf in good.values():
            if tf.header.frame_id not in good:
                roots.append(tf)

        # Print the trees
        num_printed = 0
        for tf in roots:
            t_child_root = Transform(tf)
            num_printed = num_printed + t_child_root.print_tree(good)

        # If there are unreachable transforms there must be a cycle
        if num_printed < len(good):
            print(f'!!! cycle detected with {len(good) - num_printed} transforms')

        print('===')


def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("ctrl-C detected, shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
