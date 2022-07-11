#!/bin/bash

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
    drone_id:=$drone_namespace \
    config_takeoff:=./config/takeoff_behaviour.yaml \
    config_land:=./config/land_behaviour.yaml \
    config_goto:=./config/goto_behaviour.yaml \
    config_follow_path:=./config/follow_path_behaviour.yaml "

new_window 'static_transform_publisher' "ros2 launch basic_tf_tree_generator basic_tf_tree_generator_launch.py \
    drone_id:=$drone_namespace "

new_window 'gps_translator' "ros2 launch gps_utils gps_translator_launch.py"

if [ -n "$TMUX" ]
  # if inside a tmux session detach before attaching to the session
then
   tmux switch-client -t $session:1
    else
  tmux attach -t $session:1
fi
