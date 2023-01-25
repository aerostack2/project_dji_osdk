#!/bin/bash

# Arguments
drone_namespace=$1
drone_namespace=${drone_namespace:="drone0"}

source ./launch_tools.bash

new_session $drone_namespace
use_sim_time='false'
controller="speed_controller" 
behavior_type="position"


new_window 'platform' "ros2 launch as2_platform_dji_osdk as2_platform_dji_osdk_launch.py \
    namespace:=$drone_namespace \
    use_sim_time:=$use_sim_time \
    dji_app_config:=UserConfig.txt"

new_window 'controller' "ros2 launch as2_controller controller_launch.py \
    namespace:=$drone_namespace \
    use_sim_time:=$use_sim_time \
    cmd_freq:=100.0 \
    info_freq:=10.0 \
    use_bypass:=true \
    plugin_name:=${controller} \
    plugin_config_file:=config/${controller}_config.yaml"

new_window 'state_estimator' "ros2 launch as2_state_estimator state_estimator_launch.py \
    namespace:=$drone_namespace \
    use_sim_time:=$use_sim_time \
    plugin_name:=external_odom \
    plugin_config_file:=config/external_odom_config.yaml"

new_window 'behaviors' "ros2 launch as2_behaviors_motion motion_behaviors_launch.py \
    namespace:=$drone_namespace \
    use_sim_time:=$use_sim_time \
    follow_path_plugin_name:=follow_path_plugin_$behavior_type \
    goto_plugin_name:=goto_plugin_$behavior_type \
    takeoff_plugin_name:=takeoff_plugin_platform \
    land_plugin_name:=land_plugin_platform \
    goto_threshold:=0.5 \
    takeoff_threshold:=0.5"



if [ -n "$TMUX" ]
  # if inside a tmux session detach before attaching to the session
then
   tmux switch-client -t $drone_namespace:1
    else
  tmux attach -t $drone_namespace:1
fi
