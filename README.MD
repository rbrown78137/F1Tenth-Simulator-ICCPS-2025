This repository contains the F1Tenth Simulation Comparison Example Used in the paper Perception-based Quantitative Runtime Verification for Learning-enabled Cyber-Physical Systems

To build the following package install ROS Noetic and Gazebo 11. 

To run the sample trails, run:

roslaunch f1tenth_gazebo simulator.launch simulation_example:=1
or 
roslaunch f1tenth_gazebo simulator.launch simulation_example:=2

Make sure add the package to the gazebo model path such as the following example below
export GAZEBO_MODEL_PATH=/home/catkin_ws/src/F1Tenth-Simulator-ICCPS-2025

The simulator has GUI visualization disabled by default to improve runtime performance. To change this, set the GUI arguement value in simulator.launch to true.