#!/usr/bin/env python

"""Shim to handle sim time, goes away in Dashing"""

import rclpy
import rclpy.node
import rclpy.time
import rosgraph_msgs.msg


class SimNode(rclpy.node.Node):

    def __init__(self, name: str):
        super().__init__(name)

        self._sim_time = None

        use_sim_time = self.get_parameter('use_sim_time')
        if use_sim_time.value:
            self.get_logger().info('using sim time')
            self.create_subscription(rosgraph_msgs.msg.Clock, '/clock', self.clock_callback)

    def clock_callback(self, msg: rosgraph_msgs.msg.Clock) -> None:
        self._sim_time = rclpy.time.Time.from_msg(msg.clock)

    def now(self) -> rclpy.time.Time:
        if self._sim_time is not None:
            return self._sim_time
        else:
            return self.get_clock().now()
