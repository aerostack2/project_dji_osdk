#!/bin/bash

drone_namespace="drone0"

mkdir rosbags 
cd rosbags &&\
ros2 bag record \
"/$drone_namespace/self_localization/odom" \
"/$drone_namespace/actuator_command/thrust" \
"/$drone_namespace/actuator_command/twist" \
"/$drone_namespace/motion_reference/trajectory" \
"/$drone_namespace/image_raw" \
"/$drone_namespace/aruco_gate_detector/gate_img_topic" 


