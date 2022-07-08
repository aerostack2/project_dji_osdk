#!/bin/bash

drone_namespace="drone0"

session=${USER}

UAV_MASS=1.5
UAV_MAX_THRUST=23.0 

WINDOW_ID=0
function new_window() {
  if [ $WINDOW_ID -eq 0 ]; then
    # Kill any previous session (-t -> target session, -a -> all other sessions )
    tmux kill-session -t $session
    # Create new session  (-2 allows 256 colors in the terminal, -s -> session name, -d -> not attach to the new session)
    tmux -2 new-session -d -s $session

    # send-keys writes the string into the sesssion (-t -> target session , C-m -> press Enter Button)
    tmux rename-window -t $session:0 "$1"
    tmux send-keys -t $session:0 "$2" C-m

  else
    tmux new-window -t $session:$WINDOW_ID -n "$1"
    tmux send-keys -t $session:$WINDOW_ID "$2" C-m
  fi
  WINDOW_ID=$((WINDOW_ID+1))
}

new_window 'RTPS interface' "micrortps_agent  -d /dev/ttyUSB0 -n $drone_namespace"

new_window 'pixhawk interface' "ros2 launch pixhawk_platform pixhawk_platform_launch.py \
    drone_id:=$drone_namespace \
    mass:=$UAV_MASS \
    max_thrust:=$UAV_MAX_THRUST \
    simulation_mode:=false"

new_window 'controller_manager' "ros2 launch controller_manager controller_manager_launch.py \
    drone_id:=$drone_namespace"

new_window 'trajectory_generator' "ros2 launch trajectory_generator trajectory_generator_launch.py  \
    drone_id:=$drone_namespace "

new_window 'basic_behaviours' "ros2 launch as2_basic_behaviours all_basic_behaviours_launch.py \
    drone_id:=$drone_namespace "

new_window 'aruco_gate_detector' "ros2 launch aruco_gate_detector aruco_gate_detector_real_launch.py \
    drone_id:=$drone_namespace "

new_window 'gates_to_waypoints' "ros2 launch gates_to_waypoints gates_to_waypoints_launch.py \
    drone_id:=$drone_namespace "

new_window 'static_transform_publisher' "ros2 launch basic_tf_tree_generator basic_tf_tree_generator_launch.py \
    drone_id:=$drone_namespace "

new_window  'realsense_interface' "ros2 launch realsense_interface realsense_interface_launch.py \
    drone_id:=$drone_namespace"


if [ -n "$TMUX" ]
  # if inside a tmux session detach before attaching to the session
then
   tmux switch-client -t $session:1
    else
  tmux attach -t $session:1
fi

