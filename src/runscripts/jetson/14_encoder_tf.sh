#! /usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
source "${DIR}/setup.sh"

rosrun tf static_transform_publisher 0 0 0 0 0 0 odom encoders 20
