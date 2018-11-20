[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
# ROS Publisher and Subscriber Implementation with Services, gtests and tf Broadcasting
This project implements package called beginner_tutorials that implements two nodes:
1. Publisher node - talker
2. Subscriber node - listener

### Overview
Both talker and listener nodes are written using C++. Publisher node is programmed
to publish a custom string message on the chatter topic and Subscriber node is
programmed to receive and display the string message published by the talker.
Talker node is also a server that manipulates the base string according to client
request.
Talker node broadcasts a tf frame called /talk with parent /world. The transform has a non-zero translation and a 
time varying rotation.

### Requirements/Dependencies
To run the given code, Following are the system requirements:
1. Ubuntu 16.04 OS
2. ROS Kinetic

### Installation process
#### 1. ROS Kinetic:
Install ROS Kinetic using following [link](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

#### Build Catkin Workspace
Open terminal and run following command to make workspace:
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ cd ~/src
$ git clone -b Week11_HW https://github.com/srane96/beginner_tutorials.git beginner_tutorials
$ cd ..
```

#### Build the package
Then inside work directory build the package using following command
```
$ catkin_make
```

#### Run the code
First initiate the ros master
```
$ roscore
```
Open new terminal in the workspace and run following commands to start the launch file
```
$ source devel/setup.bash
$ roslaunch beginner_tutorials service.launch rate:=<frequency value in int>
```

#### Run the service
Open another terminal in the workspace and run following commands to start the service
```
$ source devel/setup.bash
$ rosservice call manipulate_service <integer>
```

#### Display frames
To create a driagram of the frames being broadcast by talker node
```
$ rosrun tf view frames
```
It will generate frames.pdf file which shows a tree of how frames are connected.
To view the tree:
```
$ evince frames.pdf
```

#### Using rqt_tf_tree
You can use rqt_tf_tree which is a runtime tool for visualizing the tree of frames being broadcast over ROS.
```
$ rosrun rqt_tf_tree rqt_tf_tree
```

#### Using tf_echo
To view transform between /world frame and /talk being broadcast by the talker node, use following command
```
$ rosrun tf tf_echo world talk
```

#### Running test
Before running the test make sure that there are no nodes which are currently active. If there is any running node, stop it by using CTRL+C.
Go to the main workspace directory and run the gtest use following command
```
$ catkin_make run_tests_beginner_tutorials
```
This will generate a recorder.bag file in the results directory.

#### Replaying the bag file
First make sure all the nodes are stopped. Also make sure that the roscore is active. Then in the workspace start the listener node only, using the following command
```
$ rosrun beginner_tutorials listener
```
Then go to the results directory and run the following command
```
rosbag play recorder.bag
```
All the messages published by the recoder will be received and displayed by the listener node.

#### Stop the running node
To stop the running nodes press CTRL+C

