# TurtleBot_Walker_Simulation
This repository implements a simple walker algorithm with Turtlebot

---
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
---

## Dependencies
The project has following dependencies.

1. ROS Kinetic
2. catkin
3. Ubuntu 16.04 
4. Turtlebot Packages

- ROS INSTALLATION : http://wiki.ros.org/kinetic/Installation

- CATKIN INSTALLATION: http://wiki.ros.org/catkin#Installing_catkin (Usually installed by default when ROS is installed)

To install the turtlebot packages, run the following after installing ROS Kinetic on your ubuntu 16.04.
```
sudo apt-get install ros-kinetic-turtlebot-*
```

This installs all the turtlebot packages in your computer.

## Build steps
 To build the given project, create and build the catkin workspace by following the given steps:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source ~/catkin_ws/devel/setup.bash
cd src/
git clone --recursive https://github.com/Gautam-Balachandran/turtleBot_walker_simulation.git
cd ~/catkin_ws/
catkin_make
source ./devel/setup.bash
```

NOTE: For running command from each new terminal, source the devel/setup.bash file in the terminal before executing any ros command.

## Run demo

After installing all the packages mentioned above, you can either run it using roslaunch or you can run it separately. You can run the program using the following command. It will start both the turtlebot gazebo simulation and the walker node in a separate terminal.
```
roslaunch turtleBot_walker_simulation walker_sim.launch
```
If you'd like to run the nodes separately, then run roscore in the terminal as given below
```
roscore
```
We need to launch the turtlebot simulation. Run the command below in a new terminal.
```
roslaunch turtlebot_gazebo turtlebot_world.launch
```
Then run the walker ros node by running the command below in a new terminal.
```
rosrun turtleBot_walker_simulation walker_sim
```
## Recording bag files with the launch file

You must first build the project using catkin_make as described earlier. You may run the command below to launch the nodes and record all the topics except the camera data. The bag file will be in the results directory once the recording is complete. By default it records for 30 seconds, but you can the record time as an optional argument as shown below.
```
roslaunch turtleBot_walker_simulation walker_sim.launch record:=true record_time:=20
```

## Playing back the bag file

First, Go to the results folder.

```
cd .../catkin_ws/src/turleBot_walker_simulation/results
```

Make sure that Gazebo is not running. Bag file will not play otherwise. To inspect the bag file, ensure that the roscore is running. Then in a new terminal, enter the command below while in the results directory.

```
rosbag play turtlebot_walker_sim.bag
```
You will be able to see the elapsed time output on the screen. It would be playing the recorded messages when the turtlebot was actually moving.

You can view the messages being published on the topic /cmd_vel_mux/input/navi.

```
rostopic echo /cmd_vel_mux/input/navi
```
