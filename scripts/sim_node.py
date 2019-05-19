#!/usr/bin/env python

"""Shim to handle sim time, goes away in Dashing"""

import rclpy
import rclpy.time
import rclpy.node
import rosgraph_msgs.msg


class SimNode(rclpy.node.Node):

    def __init__(self, name: str):
        super().__init__(name)

        use_sim_time = self.get_parameter('use_sim_time')
        if use_sim_time.value:
            self.get_logger().info('using sim time')
            self._sim_time = rclpy.time.Time(clock_type=rclpy.time.ClockType.ROS_TIME)
            self.create_subscription(rosgraph_msgs.msg.Clock, '/clock', self.clock_callback)
        else:
            self._sim_time = None

    def clock_callback(self, msg: rosgraph_msgs.msg.Clock) -> None:
        self._sim_time = rclpy.time.Time.from_msg(msg.clock)

    def now(self) -> rclpy.time.Time:
        if self._sim_time is not None:
            return self._sim_time
        else:
            return self.get_clock().now()
