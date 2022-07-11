#!/bin/bash

if [ "$#" -le 0 ]; then
	echo "usage: $0 [drone_namespace] "
	exit 1
fi

# Arguments
drone_namespace=$1
drone_namespace=${drone_namespace:="drone0"}

source ./launch_tools.bash

new_session $drone_namespace

new_window 'DJI interface' "ros2 launch dji_matrice_platform dji_matrice_platform_launch.py \
    drone_id:=$drone_namespace \
    simulation_mode:=false"

new_window 'controller_manager' "ros2 launch controller_manager controller_manager_launch.py \
    drone_id:=$drone_namespace"

new_window 'trajectory_generator' "ros2 launch trajectory_generator trajectory_generator_launch.py  \
    drone_id:=$drone_namespace "

new_window 'basic_behaviours' "ros2 launch as2_basic_behaviours all_basic_behaviours_launch.py \
    drone_id:=$drone_namespace "

new_window 'static_transform_publisher' "ros2 launch basic_tf_tree_generator basic_tf_tree_generator_launch.py \
    drone_id:=$drone_namespace "

new_window 'gps_translator' "ros2 launch gps_utils gps_translator_launch.py"

# TODO:Check rosbag recorder
new_window 'record rosbag' "mkdir rosbags && cd rosbags &&\
    ros2 bag record \
    /$drone_namespace/self_localization/odom \
    /$drone_namespace/actuator_command/thrust \
    /$drone_namespace/actuator_command/twist \
    /$drone_namespace/motion_reference/trajectory \
    /$drone_namespace/image_raw"