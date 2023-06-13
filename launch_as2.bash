#!/bin/bash

usage() {
    echo "  options:"
    echo "      -p: platform, choices: [ign_gz | dji_osdk]"    
    echo "      -g: use GPS, choices: [true | false]"
    echo "      -e: estimator_type, choices: [raw_odometry, mocap_pose]"
    echo "      -r: record rosbag"
    echo "      -t: launch keyboard teleoperation"
    echo "      -n: drone namespace, default is drone0"
}

# Arg parser
while getopts "p:e:rtn" opt; do
  case ${opt} in
    p )
      platform="${OPTARG}"
      ;;
    e )
      estimator_plugin="${OPTARG}"
      ;;
    r )
      record_rosbag="true"
      ;;
    t )
      launch_keyboard_teleop="true"
      ;;
    n )
      drone_namespace="${OPTARG}"
      ;;
    \? )
      echo "Invalid option: -$OPTARG" >&2
      usage
      exit 1
      ;;
    : )
      if [[ ! $OPTARG =~ ^[pswrt]$ ]]; then
        echo "Option -$OPTARG requires an argument" >&2
        usage
        exit 1
      fi
      ;;
  esac
done

source utils/tools.bash

# Shift optional args
shift $((OPTIND -1))

## DEFAULTS
platform=${platform:="dji_osdk"}
estimator_plugin=${estimator_plugin:="raw_odometry"}
record_rosbag=${record_rosbag:="false"}
launch_keyboard_teleop=${launch_keyboard_teleop:="false"}
drone_namespace=${drone_namespace:="drone"}
simulation_config="sim_config/gazebo_config/world.json"
use_sim_time="false"

# Generate the list of drone namespaces
drone_ns=()
num_drones=1
for ((i=0; i<${num_drones}; i++)); do
  drone_ns+=("$drone_namespace$i")
done

# Launch Gazebo
if [[ ${platform} == "ign_gz" ]]; then
  use_sim_time="true"
  estimator_plugin="ground_truth"
  tmuxinator start -n gazebo -p tmuxinator/gazebo.yml drone_namespace=${drone_namespace} use_sim_time=${use_sim_time} simulation_config=${simulation_config} &
  wait
fi

# Always gps with raw_odometry and ground_truth, unless mocap_pose is used

for ns in "${drone_ns[@]}"
do
  tmuxinator start -n ${ns} -p tmuxinator/aerostack.yml drone_namespace=${ns} platform=${platform} simulation_config=${simulation_config} estimator_plugin=${estimator_plugin} use_sim_time=${use_sim_time} &
  wait
done

if [[ ${estimator_plugin} == "mocap_pose" ]]; then
  tmuxinator start -n mocap -p tmuxinator/mocap.yml &
  wait
fi

if [[ ${record_rosbag} == "true" ]]; then
  tmuxinator start -n rosbag -p tmuxinator/rosbag.yml drone_namespace=$(list_to_string "${drone_ns[@]}") &
  wait
fi

if [[ ${launch_keyboard_teleop} == "true" ]]; then
  tmuxinator start -n keyboard_teleop -p tmuxinator/keyboard_teleop.yml simulation=true drone_namespace=$(list_to_string "${drone_ns[@]}") &
  wait
fi

# Attach to tmux session ${drone_ns[@]}, window 0
tmux attach-session -t ${drone_ns[0]}:mission