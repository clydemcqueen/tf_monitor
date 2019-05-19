#!/usr/bin/env python

"""ROS2 TF diagnostic tool"""

from typing import Dict, List, Tuple

import transformations as xf

import rclpy
import rclpy.time
import rclpy.node
import tf2_msgs.msg
import geometry_msgs.msg

import sim_node  # TODO remove in Dashing


def q_to_rpy(q: geometry_msgs.msg.Quaternion) -> Tuple[float, float, float]:
    matrix = xf.quaternion_matrix([q.w, q.x, q.y, q.z])  # Note order
    return xf.euler_from_matrix(matrix, axes='sxyz')


def tf_to_str(tf: geometry_msgs.msg.TransformStamped) -> str:
    t = tf.transform.translation
    r = q_to_rpy(tf.transform.rotation)
    return f'<origin xyz="{t.x:.2f} {t.y:.2f} {t.z:.2f}" rpy="{r[0]:.2f}, {r[1]:.2f}, {r[2]:.2f}" />'


# Print children of this parent, return count of transforms printed
def print_children(parent: geometry_msgs.msg.TransformStamped,
                   frames: Dict[str, geometry_msgs.msg.TransformStamped],
                   prefix: str = '...') -> int:
    num_printed = 0
    for tf in frames.values():
        if parent.child_frame_id == tf.header.frame_id:
            print(f'{prefix}{tf.header.frame_id} => {tf.child_frame_id}: {tf_to_str(tf)}')
            num_printed = num_printed + print_children(tf, frames, prefix + '...') + 1
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
            print(f'{tf.header.frame_id} => {tf.child_frame_id}: {tf_to_str(tf)}')
            num_printed = num_printed + print_children(tf, frames) + 1

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
