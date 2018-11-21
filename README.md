[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
# Programming Assignment - Working with Gazebo 
This project implements a package called turtlebot_walker that implements the obstacle avoidance functionality to the turtlebot simulation in gazebo enviroment. For this purpose a C++ Walker class is implemented.
### Overview
Walker class has two main roles:
1. To listen to the date provided by the laser node to get the distance from the obstacle.
2. To update the bot speed and orientation according to the distance read from the sensor and publish it to the topic where
gazebo turtlebot is a subscriber.

### Requirements/Dependencies
To run the given code, Following are the system requirements:
1. Ubuntu 16.04 OS
2. ROS Kinetic
4. Gazebo (If not already installed with Kinetic)
3. Turtlebot simulator

### Installation process
#### 1. ROS Kinetic:
Install ROS Kinetic using following [link](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
#### 3. Install Gazebo
```
$ sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
```
#### 2. Turtlebot simulator
To install turtlebot simulator on ROS Kinetic, use following command:
```
$ sudo apt-get install ros-kinetic-turtlebot-gazebo 
ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
```
#### Build Catkin Workspace
Open terminal and run following command to make workspace:
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ cd ~/src
$ git clone https://github.com/srane96/turtlebot_walker.git
$ cd ..
```

#### Build the package
Then inside work directory build the package using following command
```
$ catkin_make
```

#### Run the code using lauch file
First initiate the ros master
```
$ roscore
```
Open new terminal in the workspace and run following commands to start the launch file which will run the turtlebot simulation in  gazebo.
```
$ source devel/setup.bash
$ roslaunch turtlebot_walker turtle.launch
```
You will see the turtlebot navigating the enviroment by avoiding obstacles.

#### Run the code using executable
First initiate the ros master
```
$ roscore
```
Run the gazebo turtlebot simulation using following command
```
$ sudo apt-get install ros-kinetic-turtlebot-gazebo 
ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
```
Open new terminal in the workspace and run following commands to start the walker node.
```
$ source devel/setup.bash
$ rosrun turtlebot_walker walker
```
You will see the turtlebot navigating the enviroment by avoiding obstacles.

#### Generating the bag file
If roscore is not running already then run if by using,
```
$ roscore
```
Open new terminal in the workspace and run following commands to start the launch file which will run the turtlebot simulation in  gazebo and start recording the topics in bag file.
```
$ source devel/setup.bash
$ roslaunch turtlebot_walker turtle.launch rosbagRecorder:=true
```

#### Replaying the bag file
First make sure all the nodes are stopped. Also make sure that the roscore is active. Then in the workspace go to the results directory and run the following command
```
rosbag play recorder.bag
```
All the messages published by the recoder will be received and displayed by the listener node.

#### Stop the running node
To stop the running nodes press CTRL+C
