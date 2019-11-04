#! /usr/bin/env bash

SESSION="sensors"
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

# This trap will watch what supervisor sends
trap "{ tmux kill-session -t $SESSION ; exit; }" SIGTERM SIGINT SIGKILL

tmux new-session -d -s $SESSION -n "MotorControl"
tmux send-keys -t $SESSION:0 "source $DIR/04_motor_control.sh" C-m

sleep 1
tmux new-window -t $SESSION:1 -n "os1_LiDAR"
tmux send-keys -t $SESSION:1 "source $DIR/01_os1.sh" C-m

sleep 1
tmux new-window -t $SESSION:2 -n "Webcam"
tmux send-keys -t $SESSION:2 "source $DIR/03_webcam.sh" C-m

sleep 1
tmux new-window -t $SESSION:3 -n "ps4drv"
tmux send-keys -t $SESSION:3 "source $DIR/07_ps4drv.sh" C-m

sleep 1
tmux new-window -t $SESSION:4 -n "Teleop"
tmux send-keys -t $SESSION:4 "source $DIR/08_teleop.sh" C-m

sleep 1
tmux new-window -t $SESSION:5 -n "GPS"
tmux send-keys -t $SESSION:5 "source $DIR/02_piksi.sh" C-m

# Keep it alive so we can kill it through supervisor
sleep infinity
