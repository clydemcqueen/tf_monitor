# `tf_monitor`

A simple ROS2 tf2 diagnostic tool. Supports ROS2 Crystal.

Features:
* builds and displays the transform tree(s) once per second
* displays transform, inverse transform and composite transform
* displays rotations in quaternions and fixed axis rpy, see https://www.ros.org/reps/rep-0103.html
* alerts you to dual-parent and stale transform problems


Usage:
~~~
ros2 run tf_monitor monitor_node.py
~~~

Parameter:
* use_sim_time

Requirements:
* ROS2, tested with Crystal
* Python 3, tested with 3.6
* transformations.py, tested with 2019.4.22