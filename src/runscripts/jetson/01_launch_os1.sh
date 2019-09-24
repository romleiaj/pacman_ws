#! /usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
source "${DIR}/setup.sh"

roslaunch --wait pacman_launch os1.launch \
    os1_hostname:="192.168.1.51" \
    os1_udp_dest:="192.168.1.52" \
    lidar_mode:="1024x10" \
    viz:=false \
    image:=false
