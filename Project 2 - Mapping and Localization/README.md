# Project 2 - Mapping and Localization

## How to launch

### Mapping (bag1)
    ros_launch robotics_project_2 gmapping.launch

### Localization (bag2/bag3)
    ros_launch robotics_project_2 amcl.launch

### Other launch files
 - <u>static_tf.launch</u> Contains static transformations of lidars and odom
 - <u>lasers_merger.launch</u> Contains laserscan_multi_merger node
 - <u>common.launch</u> Contains common nodes that need to be launched for both mapping and localization

## Nodes

### odometryTF
This node publishes the odometry of the robot in the odom frame.

### mapTrajectory.py
This node expose a service that saves the map with trajectory in a file.

## Services

### /reset_trajectory
This service resets the trajectory of the robot.

### /save_map_with_trajectory
This service saves the map with trajectory in a file at the given path.
    
    string path
    ---

## TF

### laser_front -> base_link (static)
This transformation is used to transform the front laser scan in the base_link frame.

### laser_rear -> base_link (static)
This transformation is used to transform the rear laser scan in the base_link frame.

### base_link -> odom (dynamic)
This transformation is used to transform the base_link in the odom frame.

### odom -> map (static)
This transformation is used to transform the odom frame in the map frame.
