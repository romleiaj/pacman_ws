#! /usr/bin/env bash

#NOTE: Must call "source" to set variables

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
source ${DIR}/../../../devel/setup.bash
export ROS_MASTER_URI="http://192.168.1.5:11311"
#echo "ROS_MASTER_URI is '${ROS_MASTER_URI}'."
export ROS_IP="192.168.1.5"
#echo "ROS_IP is ${ROS_IP}."
