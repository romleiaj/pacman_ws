#! /usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
source "${DIR}/setup.sh"

roslaunch --wait pacman_launch os1.launch \
    lidar_address:="192.168.0.100" \
    pc_address:="192.168.0.1" \
    replay:=false \
    pointcloud_mode:="XYZIR"
