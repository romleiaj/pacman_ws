#! /usr/bin/env bash

#NOTE: Must call "source" to set variables

#"$( ifconfig | grep "inet " | grep -Fv 127.0.0.1 | awk '{print $2}' | head -n1 )"
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
source "${DIR}/../../../devel/setup.bash"
source "${DIR}/../../../../catkin_build_ws/install/setup.bash"
export ROS_MASTER_URI="http://localhost:11311/"
echo "ROS_MASTER_URI is '${ROS_MASTER_URI}'."
#export ROS_IP="192.168.1.10"
#echo "ROS_IP is ${ROS_IP}."
