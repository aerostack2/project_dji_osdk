#!/bin/bash

drone_namespace=$1
drone_namespace=${drone_namespace:="drone0"}

tmux kill-session -t $drone_namespace