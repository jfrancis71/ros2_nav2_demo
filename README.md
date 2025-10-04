# ros2_nav2_demo
Demonstration of how to use the ROS2 Nav2 stack for navigation

Currently in development...

## Goals

To demonstrate how to use the ROS2 Nav2 stack on a real mobile robot to perform autonomous navigation. In addition to the NAV2 stack we will be using the SLAM-Toolbox to create our map of the environment.

This demo is for a differential drive robot with a 2D Lidar.

## Creating The Environment Map

### Requirements

You should setup your robot so that it can receive TwistStamped messages on topic /cmd_vel. It should be broadcasting Lidar2D messages on topic /scan.
