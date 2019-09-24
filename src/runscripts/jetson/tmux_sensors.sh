#! /usr/bin/env bash

SESSION="sensors"
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

tmux new-session -d -s $SESSION -n "MotorConrol"
tmux send-keys "source $DIR/04_launch_motor_control.sh" C-m

sleep 1
tmux new-session -t $SESSION:1 -n "os1_LiDAR"
tmux send-keys "source $DIR/01_launch_os1.sh" C-m

#sleep 1
#tmux new-window -t $SESSION:2 -n "Piksi_GPS"
#tmux send-keys "source $DIR/02_launch_piksi.sh" C-m

sleep 1
tmux new-window -t $SESSION:2 -n "Webcam"
tmux send-keys "source $DIR/03_launch_webcam.sh" C-m

# TODO
# Physical Bumpers
# All Cameras
