#!/bin/bash

# Arguments
drone_namespace=$1
drone_namespace=${drone_namespace:="drone0"}

./main_launcher.bash $drone_namespace true