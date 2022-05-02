# ECE 417 Final project
A simple program to calculate the camera matrix K. Uses a ROS node to subscribe to a topic that has images from the webcam published to it (`/cv_camera/image_raw`).

Authors: **Ryan Kinney** and **Carter Stevens**

## Usage
Compile the programs by running the following (guide assumes you are using Bash):
```
$ source /opt/ros/noetic/setup.bash
$ cd catkin_ws
$ source devel/setup.bash
$ catkin_make
```
The ROS node can be run with the following:
```
$ roslaunch src/calibration_project/launch/calibrate.launch
```
Note, to change the parameters of the chessboard and size, modify the args in the launch file as described there.

The default dimensions are as follows:
- 6 squares wide
- 8 squares tall
- 25 mm squares
- 9 mm bezel width
