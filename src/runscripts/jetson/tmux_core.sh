#! /usr/bin/env bash

SESSION="core"
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

tmux new-session -d -s $SESSION -n "core"
tmux send-keys "source $DIR/00_launch_core.sh" C-m

sleep 1
tmux new-session -t $SESSION:1 -n "master_discovery"
tmux send-keys "source $DIR/05_launch_discovery.sh" C-m

sleep 1
tmux new-session -t $SESSION:2 -n "master_sync"
tmux send-keys "source $DIR/06_launch_sync.sh" C-m
