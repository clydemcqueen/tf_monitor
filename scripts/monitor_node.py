#!/usr/bin/env python

"""ROS2 TF diagnostic tool"""

from typing import List, Optional

import rclpy
import rclpy.time
import rclpy.node
import tf2_msgs.msg
import geometry_msgs.msg

import sim_node  # TODO remove in Dashing

STALE_DURATION = rclpy.time.Duration(seconds=5)


def tf_to_str(tf: geometry_msgs.msg.TransformStamped) -> str:
    t = tf.transform.translation
    r = tf.transform.rotation
    return f'({t.x:.2f}, {t.y:.2f}, {t.z:.2f}), ({r.x:.2f}, {r.y:.2f}, {r.z:.2f}, {r.w:.2f})'


class Frame:

    def __init__(self, frame_id: str, tf: Optional[geometry_msgs.msg.TransformStamped] = None):
        self._frame_id = frame_id
        self._tf = tf  # None for root
        self._children: List[Frame] = []

    # Add or update a child frame
    def add_or_update_child(self, tf: geometry_msgs.msg.TransformStamped) -> None:
        for child in self._children:
            if child._frame_id == tf.child_frame_id:
                child._tf = tf
                return
        self._children.append(Frame(tf.child_frame_id, tf))

    # Return True if this frame has child frames
    def has_children(self) -> bool:
        return len(self._children) > 0

    # Find a frame in this tree, return None if not found
    def find_frame_in_tree(self, frame_id: str):  # -> Frame
        if frame_id == self._frame_id:
            return self
        for child in self._children:
            result = child.find_frame_in_tree(frame_id)
            if result is not None:
                return result
        return None

    # Remove stale frames from a tree
    def delete_stale_frames_from_tree(self, stamp: rclpy.time.Time) -> None:
        # Recurse first
        for child in self._children:
            child.delete_stale_frames_from_tree(stamp)

        # Check children
        fresh: List[Frame] = []
        for child in self._children:
            if stamp - rclpy.time.Time.from_msg(child._tf.header.stamp) > STALE_DURATION:
                print(f'{self._frame_id} => {child._frame_id} is stale, deleting')
            else:
                fresh.append(child)
        self._children = fresh

    # Print a tree
    def print_tree(self, prefix: str = '') -> None:
        for child in self._children:
            print(f'{prefix}{self._frame_id} => {child._frame_id} {tf_to_str(child._tf)}')
            child.print_tree(prefix + '...')


class MonitorNode(sim_node.SimNode):

    def __init__(self):
        super().__init__('monitor_node')
        self._roots: List[Frame] = []
        self.create_subscription(tf2_msgs.msg.TFMessage, '/tf', self.tf_callback)
        self.create_subscription(tf2_msgs.msg.TFMessage, '/tf_static', self.tf_callback)
        self.create_timer(1., self.timer_callback)
        self.get_logger().info("monitor_node running")

    def tf_callback(self, msg: tf2_msgs.msg.TFMessage) -> None:
        for tf in msg.transforms:
            if self.now() - rclpy.time.Time.from_msg(tf.header.stamp) > STALE_DURATION:
                print(f'{tf.header.frame_id} => {tf.child_frame_id} is stale, ignoring')
            else:
                # Remove leading '/' from frame ids
                if tf.header.frame_id[0] == '/':
                    tf.header.frame_id = tf.header.frame_id[1:]
                if tf.child_frame_id[0] == '/':
                    tf.child_frame_id = tf.child_frame_id[1:]

                # Find tf.header.frame_id
                parent: Optional[Frame] = None
                for root in self._roots:
                    parent = root.find_frame_in_tree(tf.header.frame_id)
                    if parent is not None:
                        # Update the child transform
                        parent.add_or_update_child(tf)
                        break

                if parent is None:
                    # This is a new child transform
                    parent = Frame(tf.header.frame_id)
                    parent.add_or_update_child(tf)
                    self._roots.append(parent)

        # Print the transform tree (or trees)
        if len(self._roots) > 1:
            print('ERROR: MULTIPLE TREES')
        for root in self._roots:
            root.print_tree()
            print('===')

    def timer_callback(self) -> None:
        # Remove stale transforms
        for root in self._roots:
            root.delete_stale_frames_from_tree(self.now())

        # Remove empty trees
        good: List[Frame] = []
        for root in self._roots:
            if root.has_children():
                good.append(root)
        self._roots = good


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
