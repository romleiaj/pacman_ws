#! /usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
source "${DIR}/setup.sh"

roslaunch --wait hls_lfcd_lds_driver hlds_laser.launch
