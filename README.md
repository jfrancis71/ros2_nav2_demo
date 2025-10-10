# ros2_nav2_demo

Demonstration of how to use the ROS2 Nav2 stack for navigation

Currently in development...

## Goals

To demonstrate how to use the ROS2 Nav2 stack on a real mobile robot to perform autonomous navigation. In addition to the NAV2 stack we will be using the SLAM-Toolbox to create our map of the environment.

This demo is for a differential drive robot with a 2D Lidar. The assumed setup is an actual robot running ROS2 and a desktop also running ROS2 connected together over a shared Wifi network.

Nav2 is a highly configurable, and somewhat complex piece of software. This demonstration does not aim at covering Nav2 in detail, but the minimum to get a self driving robot working.

## Mapping Your House (/Office/Shed...)

### Requirements

You should setup your robot so that it can receive TwistStamped messages on topic /cmd_vel. It should be broadcasting Lidar2D messages on topic /scan.
Check odometry

``` mermaid
    flowchart LR
    TeleopTwistJoy --/cmd_vel[TwistStamped] --> Robot
    Robot --/scan[LaserScan] --> SLAM-Toolbox
    Robot --/tf:[odom->base_link] --> TF
    SLAM-Toolbox --/map[OccupancyGrid] --> RViz2
    SLAM-Toolbox --/tf:[map->odom] --> TF
    TF --/tf:[odom->base_link] --> SLAM-Toolbox
    TF --/tf:[base_link->map] --> RViz2
``` 

To start mapping:

```
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/ros2_nav2_demo/mapper_params_online_async.yaml
```

Launch rviz2. Add topic /map. Add topic Axes, and for the Axes property set the Reference Frame to 'base_link'. This is under 'By Display Type/rviz_default_plugins/Axes' Add dialog box. I like to set the view to TopDownOrtho (and zoom in), this is under the views window.

Drive the robot around your environment, white represents clear space, black is obstacle (wall, furniture etc), grey is unknown. You will want to drive around near unknown areas to fill in the map.

Save the map (don't stop the toolbox ... use another terminal):
```
ros2 run nav2_map_server map_saver_cli -f my_house
```

Check the map has been saved correctly and stop slam toolbox.

## Running Autonomous Navigation

``` mermaid
    flowchart LR
    Robot --/scan[LaserScan] --> Nav2
    Robot --/tf:[odom->base_link] --> TF
    Robot --/odom[Odometry] --> Nav2
    Nav2 --/map[OccupancyGrid] --> RViz2
    Nav2 --/tf:[map->odom] --> TF
    Nav2 --/plan[Path] --> RViz2
    Nav2 --/global_costmap/costmap[OccupancyGrid] --> RViz2
    Nav2 --/cmd_vel[TwistStamped] --> Robot
    TF --/tf:[odom->base_link] --> Nav2
    TF --/tf:[base_link->map] --> RViz2
    RViz2 --/initialpose[PoseWithCovarianceStamped] --> Nav2
    RViz2 --/goal_pose[PoseStamped] --> Nav2
``` 


To launch Nav2:

```
ros2 launch nav2_bringup bringup_launch.py map:=./my_house.yaml params_file:=./src/ros2_nav2_demo/nav2_params.yaml
```

Within rviz2:

For below steps, I'm assuming you still have the map and Axes objects present in rviz2 from the previous SLAM-Toolbox step. If not set them up again.

1. Set Initial Pose (ie start position of robot)

    Within rviz2 you must set approximately the initial pose. It's the button marked "2D Pose Estimate" in the top panel of buttons. Use the mouse to select the initial pose on the map. You click down on the mouse, and an arrow should appear. With the mouse button still held down you can rotate the arrow and you need to rotate it so that it is pointing in the current direction of your robot. When you are happy that the direction is correct, release the mouse button. This initialises the pose. If you make a mistake and initialise in the wrong location, just repeat the process.

2. Optional Steps (Useful for visualising NAV2 planning)

   * Add Path (visualises any routes created by Nav2)

     Click on Add, select by topic (in the dialog box) and find and select /plan/Path. You might want to change its color property for better contrast.
     
   * Add Global Costmap

     Add /global_costmap/costmap/Map. Change the Color Scheme property to costmap.

3. Autonomous Navigating

    To Navigate, click on 2D Goal Pose (in RViz2). Position the mouse to where you would like it to go, click and hold down mouse button. An arrow will appear, you can then move the mouse to set the orientation and release mouse button. All being well....Off you go...

Be careful of your goal poses, our 2D Lidar only "sees" in its plane, so if you have obstructions above or below, NAV2 will not know about this. I choose goal poses such that a sensible route will not be near any obstacles, apart from doors/walls which it can detect.

Have fun!

## Technical Notes

My robot does not use base_footprint frame, just base_link. Also my robot uses TwistStamped on cmd_vel.

mapper_params_online_async.yaml is a copy of /slam_toolbox/config/mapper_params_online_async.yaml with the following alterations:

* start_pose uncommented (we must uncomment either this or map_start_at_dock for slam toolbox to work). I choose start at origin.
* base_frame altered from base_footprint to base_link
* minimum_travel_distance altered from 0.5 to 0.25 (Works better for me in small environment)

nav2_params.yaml is a copy of nav2_bringup/params/nav2_params.yaml with the following alterations:

* All references to base_footprint changed to base_link
* All controllers have enable_stamped_cmd_vel set to true

## References

YouTube: Easy SLAM with ROS using slam_toolbox, Articulated Robotics, Dec 10 2022, https://www.youtube.com/watch?v=ZaiA3hWaRzE&t=747s

YouTube: Making robot navigation easy with Nav2 and ROS!, Articulated Robotics, Jan 06 2023, https://www.youtube.com/watch?v=jkoGkAd0GYk&t=379s

A Concise Introduction to Robot Programming with ROS2, Francisco Martin Rico, 2023

Mastering ROS2 for Robotics Programming, L. Joseph, J. Cacace, (Fourth Edition) 2025

Useful discussion on odometry:
https://docs.nav2.org/setup_guides/odom/setup_odom_gz_classic.html

SLAM Toolbox:
https://github.com/SteveMacenski/slam_toolbox/tree/ros2

NAV2:
https://docs.nav2.org
