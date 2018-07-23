MYNT EYE 1.x SDK and wrapper is no longer supported. Please upgrade to 2.x SDK and firmware and use the wrapper inside.

MYNT EYE 1.x SDK and wrapper is no longer supported. Please upgrade to 2.x SDK and firmware and use the wrapper inside.

MYNT EYE 1.x SDK and wrapper is no longer supported. Please upgrade to 2.x SDK and firmware and use the wrapper inside.

MYNT EYE 2.x SDK https://github.com/slightech/MYNT-EYE-SDK-2

MYNT EYE 2.x Guide https://slightech.github.io/MYNT-EYE-SDK-2-Guide/

# MYNT EYE ROS Wrapper

MYNT EYE ROS Wrapper lets you use MYNT EYE camera with ROS. It publishs the topics of left and right images, depth map, imu, etc.

## Prerequisites

* Ubuntu 16.04
* [ROS Kinetic](http://www.ros.org/)
* [MYNT EYE SDK][]

## Getting started

* Install the latest version of the [MYNT EYE SDK][].
* Download the MYNT EYE ROS Wrapper.

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone https://github.com/slightech/MYNT-EYE-ROS-Wrapper.git
```

## Build the wrapper

The wrapper is a catkin package. You only need run `catkin_make` to build it.

```
$ cd ~/catkin_ws/
$ catkin_make
$ source ./devel/setup.bash
```

## Launch the wrapper

Run MYNT EYE camera node, and open Rviz to display:

```
$ roslaunch mynteye_ros_wrapper mynt_camera_display.launch
```

Run MYNT EYE camera node:

```
$ roslaunch mynteye_ros_wrapper mynt_camera.launch

# Or
$ rosrun mynteye_ros_wrapper mynteye_wrapper_node
```

[MYNT EYE SDK]: https://github.com/slightech/MYNT-EYE-SDK.git
