# Quadrupedal Robot with Active Tail Stability Control
RBE521 - Legged Robotics final project for Team Robot Tail:
 - John Dong
 - Kyle Erf
 - Ryan Carnemolla
 - Adam Nudelmann

Simulation of a quadrupedal robot in ROS and Gazebo. The robot includes a tail-based stabilization controller which attempts to help stabilize the robot while it's walking.

## Installation
Clone repository into the src folder of a catkin workspace, and then run rosdep to install dependencies:
```bash
rosdep update
rosdep install --from-paths src --ignore-src -y
```
Then build:
```bash
catkin_make
```

## Usage
First source your workspace:
```bash
source devel/setup.bash
```
To load the robot in Gazebo, launch the adaptive_guide_launch.py file:
```bash
roslaunch quadruped_control quadruped.launch
```
Gazebo starts paused, so unpause it after it finishes starting up.
### Walking
To make the robot walk, you can either run the walk_node:
```bash
rosrun quadruped_control walk_node
```
Which will make the robot walk forward one meter. You can also publish a goal to the gait controller action server topic: ```/gait_controller/goal```
### Tail Stability Controller
To activate the tail stability controller, publish a non-zero gain value to the ```/quadruped/stabilizer/gain``` topic. You can also put the tail at an angular offset by publishing an offset value to this topic: ```/quadruped/stabilizer/offset```. We've found that a gain of 6, and an offset of 0.707 works well. Once activated, the tail should start moving to stabilize the robot if necessary.
### Experiments
To run an experiment where the robot walks forward 1 meter while recording Foot Force Stability Margin (FFSM) data to a text file, run the following node:
```bash
rosrun quadruped_control run_experiment.py <filename>
```
Replace <filename> with a filepath for the data to be saved to.\
\
Some other relevant topics:
 - FFSM Measurements: ```/stability/ffsm```
 - IMU Measurements: ```/imu```
