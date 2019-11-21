#! /usr/bin/env bash

SESSION="core"
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

# This trap will watch what supervisor sends
trap "{ tmux kill-session -t $SESSION ; exit; }" SIGTERM SIGINT SIGKILL

tmux new-session -d -s $SESSION -n "core"
tmux send-keys -t $SESSION:0 "source $DIR/00_core.sh" C-m

#tmux new-window -t $SESSION:1 -n "VPN"
#tmux send-keys -t $SESSION:1 "source $DIR/open_tunnel.sh" C-m

# Keep process alive so we can kill it
sleep infinity
