#!/bin/sh

rosbag record /ros_imu /tf /odom  /camera/rgb/image_rect_color/compressed /velodyne_points /map /submap_list
