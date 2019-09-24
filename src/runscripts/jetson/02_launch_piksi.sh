#! /usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
source "${DIR}/setup.sh"

roslaunch --wait pacman_launch piksi_multi_rover.launch \
    interface:="tcp" \
    tcp_addr:="192.168.1.53" \
    base_station_ip:="192.168.0.1"
