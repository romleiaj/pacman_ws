#! /usr/bin/env bash

SESSION="analytics"
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

# This trap will watch what supervisor sends
trap "{ tmux kill-session -t $SESSION ; exit; }" SIGTERM SIGINT SIGKILL

tmux new-session -d -s $SESSION -n "Segmentation"
tmux send-keys -t $SESSION:0 "source $DIR/09_segmentation.sh" C-m

sleep 1
tmux new-window -t $SESSION:1 -n "PathPlanning"
tmux send-keys -t $SESSION:1 "source $DIR/16_path_planning.sh" C-m

sleep 1
tmux new-window -t $SESSION:2 -n "PathNavigation"
tmux send-keys -t $SESSION:2 "source $DIR/18_path_navigation.sh"

# Keep it alive so we can kill it through supervisor
sleep infinity
