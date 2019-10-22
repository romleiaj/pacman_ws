#! /usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
source "${DIR}/setup.sh"

roslaunch --wait pacman_launch diff_motor_control.launch \
    port:="/dev/ttyACM0" \
    baud:="115200" \
    open-loop:="false"
