#! /usr/bin/env bash

SESSION="analytics"
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

# This trap will watch what supervisor sends
trap "{ tmux kill-session -t $SESSION ; exit; }" SIGTERM SIGINT SIGKILL

tmux new-session -d -s $SESSION -n "Segmentation"
tmux send-keys -t $SESSION:0 "source $DIR/09_segmentation.sh" C-m

sleep 1
tmux new-window -t $SESSION:1 -n "EncoderOdom"
tmux send-keys -t $SESSION:1 "source $DIR/10_difftf.sh" C-m

sleep 1
tmux new-window -t $SESSION:2 -n "OdomFusion"
tmux send-keys -t $SESSION:2 "source $DIR/11_odom_fusion.sh" C-m

sleep 1
tmux new-window -t $SESSION:3 -n "IMUOrientation"
tmux send-keys -t $SESSION:3 "source $DIR/12_imu_orientation.sh" C-m

# Keep it alive so we can kill it through supervisor
sleep infinity
