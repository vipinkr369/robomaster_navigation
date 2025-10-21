# Robomaster Waypoint Navigation

This project contains a Python script to control **Robomaster** in a ROS2 environment using **AMCL localization** and **Nav2 navigation**.  

This code allows you to provide inputs such as:

robot_name → the robot we want to command (e.g., 'sk01').
command_type → 'init' for initial pose or 'goal' for navigation goal.
coordinates → a dictionary containing x, y, z, w values for the pose.
node_id → used to give unique names to nodes in threads.

These inputs can be used in **threads** for asynchronous execution.  
Use the `RobotCommand` class to publish the initial pose or send goal commands to the robot.

Commands Used
In Robot
ros2 launch rona_navigation vicon_bringup_launch.py 
In Host Computer
ros2 launch robomaster_nav2_bringup rviz_launch.py namespace:=sk01 use_namespace:=true rviz_config:=/home/vipin/rona_ws/src/RoNav_FLW_isaac_nav/robomaster_nav2_bringup/config/rviz/nav2_namespaced_view.rviz
python3 waypoint.py 
