[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-24ddc0f5d75046c5622901739e7c5dd533143b0c8e959d652212380cedb1ea36.svg)](https://classroom.github.com/a/jgFOdiBm)
## Assignment 2
This project contains skeleton code for assignment 2. The first portion of the lab requires the student to publish poses for a car clone that follows the original car around. The second portion requires the student to record a bag file of the robot driving around while being tele-operated. Then a script is written to play back the bag file such that the robot autonomously follows (approximately) the path that was recorded in the bag. See the assignment spec for more details about the assignment.

<!--
### Installation
The following packages are required:
1. ackermann_msgs: **sudo apt-get install ros-melodic-ackermann_msgs**
 
2. numpy: **sudo apt-get install python-numpy**

Once these packages have been installed, clone this package, place it in a catkin workspace, and compile using catkin_make (or whichever build mechanism you prefer).-->
  
### Usage

After implementing CloneFollower.py and CloneFollower.launch, the following command will launch the script for the first portion of the assignment: **roslaunch assignment2 CloneFollower.launch**

After implementing BagFollower.py and BagFollower.launch, the following command will launch the script for the second portion of the assignment: **roslaunch assignment2 BagFollower.launch**
