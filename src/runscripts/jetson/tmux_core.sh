#! /usr/bin/env bash

SESSION="core"
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

tmux new-session -d -s $SESSION -n "core"
tmux send-keys "source $DIR/00_launch_core.sh" C-m
