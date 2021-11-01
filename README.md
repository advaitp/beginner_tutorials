[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

# beginner tutorials
## Overview
 A tutorial that covers how to write a publisher and subscriber node in C++.
 
## Dependencies
- ROS Melodic
- Modern C++ Programming Language
- Std_Msgs Package
- Roscpp Package
- Catkin_Make build system

## Build 
1. Create Catkin Workspace
```
cd ~/catkin_ws
catkin_make clean && catkin_make
```
2. Copy the repository in src folder of catkin workspace
```
cd src 
git clone https://github.com/advaitp/beginner_tutorials.git
cd ..
catkin_make clean && catkin_make
source ./devel/setup.bash
```
## Run 
1. Make sure Roscore is running 
```
roscore
```     
2. Make sure you have sourced setup file
```
cd ~/catkin_ws
source ./devel/setup.bash
``` 
3. To run the publisher node
```
rosrun beginner_tutorials publisher
```
3. To run the subscriber node
```
rosrun beginner_tutorials subscriber
```
