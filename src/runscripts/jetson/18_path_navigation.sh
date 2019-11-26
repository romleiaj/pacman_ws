#! /usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
source "${DIR}/setup.sh"
source "${DIR}/../../../devel/setup.bash"

roslaunch --wait pacman_launch path_navigation.launch 
