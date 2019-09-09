#! /usr/bin/env bash

# NOTE: Must call "source" to setup environmental variables

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
source ${DIR}/../../../devel/setup.bash
export ROS_MASTER_URI=$(cat ../ROS_MASTER.txt)
echo "ROS_MASTER_URI is '${ROS_MASTER_URI}'."
export ROS_IP="192.168.119.15"
echo "ROS_IP is ${ROS_IP}."
