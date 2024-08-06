#!/bin/bash

source ~/ros_ws/install/setup.bash

SESSION_NAME="freya_sim"


tmux new-session -d -s $SESSION_NAME

tmux rename-window -t $SESSION_NAME:0 'asv_setup'
tmux send-keys -t $SESSION_NAME:0 'ros2 launch asv_setup sim_freya.launch.py' C-m

tmux new-window -t $SESSION_NAME:1 -n 'vrx_gz'
tmux send-keys -t $SESSION_NAME:1 'ros2 launch vrx_gz competition.launch.py world:=Njord_maneuvering' C-m

tmux attach-session -t $SESSION_NAME

echo "Tmux session '$SESSION_NAME' has been created and attached. Use 'tmux kill-session -t $SESSION_NAME' to terminate."