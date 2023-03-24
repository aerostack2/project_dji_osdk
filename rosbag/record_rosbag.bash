#!/bin/bash

drone_namespace=$1
drone_namespace=${drone_namespace:="drone0"}

echo $(pwd)

reliability_override_file=$(pwd)/rosbag/reliability_override.yaml
if [[ "$drone_namespace" == "M200" ]] 
then
    reliability_override_file=$(pwd)/rosbag/reliability_override_M200.yaml
elif [[ "$drone_namespace" == "M300" ]] 
then
    reliability_override_file=$(pwd)/rosbag/reliability_override_M300.yaml
fi

mkdir rosbag/rosbags 2>/dev/null
cd rosbag/rosbags &&\
ros2 bag record \
"/$drone_namespace/platform/info" \
"/$drone_namespace/self_localization/pose" \
"/$drone_namespace/self_localization/twist" \
"/$drone_namespace/actuator_command/twist" \
"/$drone_namespace/sensor_measurements/imu" \
"/$drone_namespace/sensor_measurements/odom" \
"/$drone_namespace/sensor_measurements/gps" \
"/tf" \
"/tf_static" \
--qos-profile-overrides-path $reliability_override_file --include-hidden-topics



