#!/bin/bash

usage() {
    echo "  options:"
    echo "      -s: simulated, choices: [true | false]"
    echo "      -g: use GPS, choices: [true | false]"
    echo "      -e: estimator_type, choices: [raw_odometry, mocap_pose]"
    echo "      -r: record rosbag"
    echo "      -t: launch keyboard teleoperation"
    echo "      -n: drone namespace, default is drone0"
}

# Arg parser
while getopts "se:rtn" opt; do
  case ${opt} in
    s )
      simulated="true"
      ;;
    g )
      gps="true"
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
      if [[ ! $OPTARG =~ ^[swrt]$ ]]; then
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
gps=${gps:="true"}
estimator_plugin=${estimator_plugin:="raw_odometry"}
record_rosbag=${record_rosbag:="false"}
launch_keyboard_teleop=${launch_keyboard_teleop:="false"}
drone_namespace=${drone_namespace:="drone"}

# Generate the list of drone namespaces
drone_ns=()
num_drones=1
for ((i=0; i<${num_drones}; i++)); do
  drone_ns+=("$drone_namespace$i")
done

for ns in "${drone_ns[@]}"
do
  if [[ ${ns} == ${drone_ns[0]} ]]; then
    base_launch="true"
  else
    base_launch="false"
  fi 

  tmuxinator start -n ${ns} -p utils/session.yml drone_namespace=${ns} gps=${gps} estimator_plugin=${estimator_plugin} &
  wait
done

if [[ ${record_rosbag} == "true" ]]; then
  tmuxinator start -n rosbag -p utils/rosbag.yml drone_namespace=$(list_to_string "${drone_ns[@]}") &
  wait
fi

if [[ ${launch_keyboard_teleop} == "true" ]]; then
  tmuxinator start -n keyboard_teleop -p utils/keyboard_teleop.yml simulation=true drone_namespace=$(list_to_string "${drone_ns[@]}") &
  wait
fi

# Attach to tmux session ${drone_ns[@]}, window 0
tmux attach-session -t ${drone_ns[0]}:mission