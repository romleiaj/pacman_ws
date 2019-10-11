# pacman_ws
The PACMAN (Path Assembling Cartographer, Mapping under Autonomous Navigation). 

## Purpose
This workspace hosts all software and utilities to enable PACMAN to operate.
PACMAN consists of the following components:
1. Wheelchair base
2. Jetson TX2
3. RGB cameras
4. Single-Line 360 degree LiDAR
5. Cradlepoint Router

## Simuation
In order to simuate the pacman vehicle through Gazebo simulation software, you must download the latest version of Gazebo alongside with its ROS packages. Note: the full desktop version of ROS include Gazebo software (therefore an intall is not necessary).

To install Gazebo (Ubuntu):
```
curl -sSL http://get.gazebosim.org | sh
```

To to run the simulation of the vehicle with this repository, you must also download the vehicle model (which is not included in the git repository). The SDF folder for the model is located within this google drive directory of Sdf Files: [SDF Files](https://drive.google.com/open?id=1xschITUsA2JVnLQphrNJv-SwWNVNDtrY). Please download the entirety of the pacman folder within this linked directory.

This folder (pacman) must be moved under the models folder: 
```
pacman_ws/src/pacman_simulation/models
```
The models directory should look similar to this:
```
models/
	pacman/
		model.config
		model.sdf
		meshes/
		materials/
```
Make sure to catkin_make within the pacman_ws folder to procede (fill in \<path to pacman_ws\> with the corresponding path to the pacman workspace). Then, source the /devel/setup.bash folder:
```
cd <path to pacman_ws>
source devel/setup.bash
```
Finally, to launch simulation, use the roslaunch command to open up the gazebo simulation:
```
roslaunch pacman_simulation my_world.launch 
```

## Operation

## Results
