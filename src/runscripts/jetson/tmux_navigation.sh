#! /usr/bin/env bash

SESSION="navigation"
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

# This trap will watch what supervisor sends
trap "{ tmux kill-session -t $SESSION ; exit; }" SIGTERM SIGINT SIGKILL

tmux new-session -d -s $SESSION -n "JointPublisher"
tmux send-keys -t $SESSION:0 "source $DIR/13_tf_jointstates.sh" C-m

sleep 1
tmux new-window -t $SESSION:1 -n "OdomFusion"
tmux send-keys -t $SESSION:1 "source $DIR/11_odom_fusion.sh" C-m

sleep 1
tmux new-window -t $SESSION:2 -n "IMUOrientation"
tmux send-keys -t $SESSION:2 "source $DIR/12_imu_orientation.sh" C-m

sleep 1
tmux new-window -t $SESSION:3 -n "EncoderOdom"
tmux send-keys -t $SESSION:3 "source $DIR/10_difftf.sh" C-m

sleep 1
tmux new-window -t $SESSION:4 -n "EncoderTF"
tmux send-keys -t $SESSION:4 "source $DIR/14_encoder_tf.sh" C-m

sleep 1
tmux new-window -t $SESSION:4 -n "TwistMux"
tmux send-keys -t $SESSION:4 "source $DIR/18_twist_mux.sh" C-m

# Keep it alive so we can kill it through supervisor
sleep infinity
