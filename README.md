# `tf_monitor`

A simple ROS2 tf2 diagnostic tool. Supports ROS2 Crystal.

Features:
* builds and displays the transform tree(s) once per second
* displays transform, inverse transform and composite transform in both rpy and quaternion formats
* alerts you to dual-parent and stale transform problems


Usage:
~~~
ros2 run tf_monitor monitor_node.py
~~~

Parameter:
* use_sim_time