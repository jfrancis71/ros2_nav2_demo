# ros2_nav2_demo
Demonstration of how to use the ROS2 Nav2 stack for navigation

Currently in development...

## Goals

To demonstrate how to use the ROS2 Nav2 stack on a real mobile robot to perform autonomous navigation. In addition to the NAV2 stack we will be using the SLAM-Toolbox to create our map of the environment.

This demo is for a differential drive robot with a 2D Lidar.

## Mapping Your House (/Office/Shed...)

### Requirements

You should setup your robot so that it can receive TwistStamped messages on topic /cmd_vel. It should be broadcasting Lidar2D messages on topic /scan.
Check odometry

``` mermaid
    flowchart LR
    Robot --/cmd_vel[TwistStamped] --> SLAM-Toolbox
    Robot --/scan[LaserScan] --> SLAM-Toolbox
    Robot --/tf odom --> SLAM-Toolbox
    SLAM-Toolbox --/map[OccupancyGrid] --> RViz2
``` 

```ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/ros2_nav2_demo/mapper_params_online_async.yaml```

The params file comes from the slam toolbox with a few alterations:
We uncomment start_pose.
base_frame is altered from base_footprint to base_link
minimum_travel_distance from 0.5 to 0.25. Not required, but I think it works better like this, especially in smaller environments.

Launch rviz2. Add topic /map. Add topic Axes, and for the Axes property set the Reference Frame to 'base_link'. This is under 'By Display Type/rviz_default_plugins/Axes' Add dialog box. I like to set the view to TopDownOrtho (and zoom in), this is under the views window.

Drive the robot around your environment, white represents clear space, black is obstacle (wall, furniture etc), grey is unknown. You will want to drive around near unknown areas to fill in the map.

Save the map (don't stop the toolbox ... use another terminal):
```ros2 run nav2_map_server map_saver_cli -f my_house```

Check the map has been saved correctly and stop slam toolbox.

## Running Autonomous Navigation

```ros2 launch nav2_bringup bringup_launch.py map:=./my_house.yaml params_file:=./src/ros2_nav2_demo/nav2_params.yaml```

Within rviz2 you must set approximately the initial pose. It's the button marked "2D Pose Estimate" in the top panel of buttons. Use the mouse to select the initial pose on the map. You click down on the mouse, and an arrow should appear. With the mouse button still held down you can rotate the arrow and you need to rotate it so that it is pointing in the current direction of your robot. When you are happy that the direction is correct, release the mouse button. This initialises the pose. If you make a mistake and initialise in the wrong location, just repeat the process.

Optional Steps:
For the next step (autonomous navigating) we will want to see the proposed path. So in rviz2 click on Add, select by topic (in the dialog box) and find and select /plan/Path. The color is green by default, you might want to change it purple to have it stand out (given that red/green/blue are used by the Axes).
Add /global_costmap/costmap/Map. Change Color Scheme property to costmap. You should see a color map indicating preferred areas of travel versus avoided areas.

To Navigate, click on 2D Goal Pose (in RViz2). Position the mouse to where you would like it to go, click and hold down mouse button. An arrow will appear, you can then move the mouse to set the orientation and release mouse button. All being well....Off you go...

Be careful of your goal poses, our 2D Lidar only "sees" in its plane, so if you have obstructions above or below, NAV2 will not know about this. I choose goal poses such that a sensible route will not be near any obstacles, apart from doors/walls which it can detect.


## References

YouTube: Easy SLAM with ROS using slam_toolbox, Articulated Robotics, Dec 10 2022, https://www.youtube.com/watch?v=ZaiA3hWaRzE&t=747s

YouTube: Making robot navigation easy with Nav2 and ROS!, Articulated Robotics, Jan 06 2023, https://www.youtube.com/watch?v=jkoGkAd0GYk&t=379s

A Concise Introduction to Robot Programming with ROS2, Francisco Martin Rico, 2023

Mastering ROS2 for Robotics Programming, L. Joseph, J. Cacace, (Fourth Edition) 2025
