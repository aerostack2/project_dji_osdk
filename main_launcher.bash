#!/bin/bash

# Arguments
drone_namespace=$1
drone_namespace=${drone_namespace:="drone0"}

record_rosbag=$2
record_rosbag=${record_rosbag:="false"}

source ./tools/launch_tools.bash

new_session $drone_namespace

use_sim_time='false'
controller="pid_speed_controller" 
behavior_type="position"

new_window 'platform' "ros2 launch as2_platform_dji_osdk as2_platform_dji_osdk_launch.py \
    namespace:=$drone_namespace \
    use_sim_time:=$use_sim_time \
    simulation_mode:=true \
    dji_app_config:=UserConfig.txt"

new_window 'alphanumeric_viewer' "ros2 run as2_alphanumeric_viewer as2_alphanumeric_viewer_node \
    --ros-args -r  __ns:=/$drone_namespace"

new_window 'controller' "ros2 launch as2_motion_controller controller_launch.py \
    namespace:=$drone_namespace \
    use_sim_time:=$use_sim_time \
    cmd_freq:=100.0 \
    info_freq:=10.0 \
    use_bypass:=true \
    plugin_name:=$controller \
    plugin_config_file:=config/${controller}.yaml"

new_window 'state_estimator' "ros2 launch as2_state_estimator state_estimator_launch.py \
    namespace:=$drone_namespace \
    use_sim_time:=$use_sim_time \
    plugin_name:=raw_odometry \
    plugin_config_file:=config/raw_odometry.yaml"

new_window 'behaviors' "ros2 launch as2_behaviors_motion motion_behaviors_launch.py \
    namespace:=$drone_namespace \
    use_sim_time:=$use_sim_time \
    follow_path_plugin_name:=follow_path_plugin_$behavior_type \
    go_to_plugin_name:=go_to_plugin_$behavior_type \
    takeoff_plugin_name:=takeoff_plugin_platform \
    land_plugin_name:=land_plugin_platform \
    go_to_threshold:=0.5 \
    takeoff_threshold:=0.5"

if [[ "$behavior_type" == "trajectory" ]]
then
    new_window 'traj_generator' "ros2 launch as2_behaviors_trajectory_generation generate_polynomial_trajectory_behavior_launch.py  \
        namespace:=$drone_namespace \
        use_sim_time:=$use_sim_time"
fi

if [[ "$record_rosbag" == "true" ]]
then
    new_window 'record rosbag' "./rosbag/record_rosbag.bash $drone_namespace"
fi

# if inside a tmux session detach before attaching to the session
if [ -n "$TMUX" ]; then
    tmux switch-client -t $drone_namespace
else
    tmux attach -t $drone_namespace:0
fi
