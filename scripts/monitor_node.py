#!/usr/bin/env python

"""ROS2 TF diagnostic tool"""

from typing import Dict, List, Tuple

import numpy as np
import transformations as xf

import rclpy
import rclpy.time
import rclpy.node
import tf2_msgs.msg
import geometry_msgs.msg

import sim_node  # TODO remove in Dashing

# Heads up:
# transformations.py expresses quaternions as [w x y z], but ROS users expect to see [x y z w]


def t_to_str(t: geometry_msgs.msg.Vector3) -> str:
    return f'xyz=({t.x:.2f}, {t.y:.2f}, {t.z:.2f})'


def q_to_str(q: List) -> str:
    return f'xyzw=({q[1]:.2f}, {q[2]:.2f}, {q[3]:.2f}, {q[0]:.2f})'


def e_to_str(r: Tuple[float, float, float]) -> str:
    return f'rpy=({r[0]:.2f}, {r[1]:.2f}, {r[2]:.2f})'


def tf_to_matrix(tf: geometry_msgs.msg.TransformStamped) -> np.ndarray:
    q = [tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z]
    m = xf.quaternion_matrix(q)
    m[:3, 3] = [tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z]
    return m


def print_tf(tf: geometry_msgs.msg.TransformStamped,
             prefix: str = '') -> None:
    t = tf.transform.translation

    # Get the forward translation
    q = [tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z]
    m = xf.quaternion_matrix(q)
    e = xf.euler_from_matrix(m)

    m_inverse = xf.inverse_matrix(m)
    q_inverse = xf.quaternion_from_matrix(m_inverse)
    e_inverse = xf.euler_from_matrix(m_inverse)

    print(f'{prefix}{tf.header.frame_id} => {tf.child_frame_id}: {t_to_str(t)} {q_to_str(q)} {e_to_str(e)}\n'
          f'                          inverse: {q_to_str(q_inverse)} {e_to_str(e_inverse)}')


# Print children of this parent, return count of transforms printed
def print_children(parent: geometry_msgs.msg.TransformStamped,
                   frames: Dict[str, geometry_msgs.msg.TransformStamped],
                   m_parent_root: np.ndarray,
                   prefix: str = '   ') -> int:
    num_printed = 0
    for tf in frames.values():
        if parent.child_frame_id == tf.header.frame_id:
            print_tf(tf, prefix)

            m_child_parent = tf_to_matrix(tf)
            m_child_root = m_child_parent @ m_parent_root
            e_child_root = xf.euler_from_matrix(m_child_root)
            print(f'                          composite: {e_to_str(e_child_root)}')

            num_printed = num_printed + print_children(tf, frames, m_child_root, prefix + '   ') + 1
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
        frames: Dict[str, geometry_msgs.msg.TransformStamped] = {}

        for tf in self._transforms.values():

            # Ignore conflicting parents
            if tf.child_frame_id in frames:
                print(f'!!! {tf.header.frame_id} => {tf.child_frame_id} conflicts with',
                      f'{frames[tf.child_frame_id].header.frame_id} => {tf.child_frame_id}, ignoring')
                continue

            frames[tf.child_frame_id] = tf

        # Find roots
        roots: List[geometry_msgs.msg.TransformStamped] = []

        for tf in frames.values():
            if tf.header.frame_id not in frames:
                roots.append(tf)

        # Print the tree
        num_printed = 0
        for tf in roots:
            print_tf(tf)
            m_child_root = tf_to_matrix(tf)
            num_printed = num_printed + print_children(tf, frames, m_child_root) + 1

        if num_printed < len(frames):
            print(f'!!! cycle detected with {len(frames) - num_printed} transforms')

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
