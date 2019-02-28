#!/bin/sh

rosbag record /kinect_head/rgb/image_color/compressed /tf /torso_lift_imu/data /base_scan /tilt_scan /robot_pose_ekf/odom_combined
