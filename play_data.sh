#!/usr/bin/env bash
rosbag play -l ~/catkin_ws/src/report2/camera_3D_lw.bag /camera/color/image_raw_throttle:=/camera/color/image_raw /camera/depth_registered/points_throttle:=/camera/depth_registered/points

