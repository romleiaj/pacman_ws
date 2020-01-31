<img src="/data/Documentation/cropped_poster.png" alt="poster" align="center">

PACMAN (Path Assembling Cartographer, Mapping under Autonomous Navigation) is a self-driving ground robot capable of following a path, specifically paved sidewalks, and returning real-world coordinates of that path back to a ground station (such as a remote PC). These real-world coordinates can be saved and distributed to other systems that require the absolute position of pathways. Such systems include autonomous lawnmowers, autonomous snow removers, autonomous delivery robots, and others.

## Getting Started
Since this is a ROS (Robot Operating System) workspace, you must have a functioning ROS environment on your PC. This workspace was only tested on Ubuntu 18.04 with ROS Melodic. When running on a Jetson system, ensure to set power mode to max, `sudo nvpmodel -m 0`. 
To run all required runscripts, the following packages are required:
* ros-melodic-robot-state-publisher
* ros-melodic-joint-state-publisher
* tmux
* supervisor
* python-catkin-tools

And the following Python packages are required:
* numpy 1.13.3
* scipy
* ps4drv
* yacs
* tqdm

To build all required components:
1. $ `cd ~`
2. $ `git clone --recursive https://github.com/romleiaj/pacman_ws`
3. $ `cd pacman_ws`
4. $ `catkin build`


## Components
This workspace hosts all software and utilities to enable PACMAN to operate.
PACMAN consists of the following components:
1. GOLDEN Wheelchair base
2. Jetson Xavier - 16 Gb
3. Logitech C920 Webcam
4. Ouster OS1-16 LiDAR
5. Cradlepoint Cellular Router
6. Swiftnav Duro RTK GPS
7. Two 12V 50Ah batteries in series
8. Roboteq MDC2230 Motor Controller
9. Pair of US Digital E3 Motor Encoders
10. Pair of 24V 135RPM DC Motors

## Simulation
<img src="/data/Visualization/top-down-path.png" alt="path" align="left" height="273">
<img src="/data/Visualization/pacman-sidewalk.png" alt="pacmanonpath" align="right" height="275">
&nbsp;

To simulate the pacman vehicle through Gazebo simulation software, you must download the latest version of Gazebo alongside with its ROS packages. Note: the full desktop version of ROS includes Gazebo software (therefore an install is not necessary).

To install Gazebo (Ubuntu):
```
curl -sSL http://get.gazebosim.org | sh
```
Make sure to `catkin build` within the pacman_ws folder to procede (fill in \<path to pacman_ws\> with the corresponding path to the pacman workspace). Then, source the /devel/setup.bash folder:
```
cd <path to pacman_ws>
source devel/setup.bash
```
Finally, to launch simulation, use the roslaunch command to open up the gazebo simulation:
```
roslaunch pacman_simulation my_world.launch 
```

## Operation

To run the PACMAN system, there are many bash runscripts located within `/src/runscripts/jetson/` that must be run locally on the Jetson Xavier. There are 4 main scripts, 
```
tmux_core.sh
tmux_analytics.sh
tmux_navigation.sh
tmux_sensors.sh
```
that all must be launched in order for the system to be operational. Each of these scripts starts a tmux session, and starts multiple processes in separate windows within that session. This allows for individual process monitoring, as well as restarting individual processes, and a stable environment to work in via SSH.

These scripts are currently started at boot using `supervisord`, a process control program. This launch is controlled by the `supervisor.conf` located in the same directory. This is symlinked to a file in the home directory of the Xavier, which is then trampolined up to the `/etc/supervisor/conf.d/` to dodge privileges conflicts by using version control.

## Results

<img src="/data/Visualization/orig_plus_seg.png" alt="segment" align="center" height="300">
&nbsp;
<img src="/data/Visualization/gps-track.png" alt="gps" align="center">
&nbsp;
<img src="/data/Visualization/path-indoors.png" alt="pathi" align="left" height="225">
<img src="/data/Visualization/path-planned.png" alt="pathp" align="right" height="225">
&nbsp;
<img src="/data/Visualization/rviz-nav.png" alt="rviznav" align="center">
