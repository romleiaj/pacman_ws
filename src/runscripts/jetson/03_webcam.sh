#! /usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
source "${DIR}/setup.sh"

roslaunch --wait pacman_launch webcam.launch \
    camera_name:="webcam" \
    set_camera_fps:=30 \
    fps:=30 \
    width:=2304 \
    height:=1296 \
    visualize:=false
