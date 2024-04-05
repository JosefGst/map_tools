# Template ROS sub pub pkg cpp

## Contents

- [Run](#run)
- [Subscribed Topics](#subscribed-topics)


Tools to help during the demo setup. Currently only includes waypoint recording utilities.

## Run

Record the current amcl_pose and orientation in quaternion of the robot and save it to a file.

    rosrun map_tools pose.py
    rosrun map_tools pose.py -o my_pose.txt -e

Subscribe to the clicked_pose topic and save it to a file.

    rosrun map_tools clicked_pose.py
    rosrun map_tools clicked_pose.py -o my_pose.txt -e

## Subscribed Topics

- amcl_pose [(geometry_msgs/PoseWithCovariance)](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseWithCovariance.html)


