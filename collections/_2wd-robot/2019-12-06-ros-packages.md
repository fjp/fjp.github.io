---
layout: single #collection
title: Autonomous 2WD Robot - ROS Packages and Nodes
permalink: /projects/2wd-robot/ros-packages/
excerpt: "ROS Packages and Nodes for ROS Melodic running on a Raspberry Pi 4 for an autonomous 2WD Robot to act in an environment according to sensor information."
date: 2019-12-06 15:00:35 +0100
categories: [robotics]
tags: [2wd, robot, ros, melodic, raspberry, pi, autonomous, ubuntu, bionic, package, control]
comments: true
use_math: true
toc: true
classes: wide
# toc_label: "Unscented Kalman Filter"
header:
  # teaser: /assets/collections/2wd-robot/assembly/motor/08-motor-driver-power.jpg
  # overlay_image: /assets/collections/2wd-robot/assembly/motor/08-motor-driver-power.jpg
  overlay_filter: 0.5 # same as adding an opacity of 0.5 to a black background
  # caption: "Source: [**hpiracing**](http://www.hpiracing.com/de/kit/114343)"
  show_overlay_excerpt: true
sidebar:
  nav: "2wd-robot"
---

On the [file system level](https://wiki.ros.org/ROS/Concepts#ROS_Filesystem_Level), ROS uses [Packages](http://wiki.ros.org/Packages) which are part of every ROS distribution such as ROS Melodic and defined as follows:

Packages are the main unit for organizing software in ROS. A package may contain ROS runtime processes ([nodes](http://wiki.ros.org/Nodes)), a ROS-dependent library, datasets, configuration files, or anything else that is usefully organized together. Packages are the most atomic build item and release item in ROS. Meaning that the most granular thing you can build and release is a package.
{: .notice }


For this autonomous 2WD robot project the nodes run on a Raspberry Pi 4 to act in an environment according to sensor information.

## Naming Conventions

ROS provides [naming conventions](http://wiki.ros.org/ROS/Patterns/Conventions) for 
packages ([REP-144](https://www.ros.org/reps/rep-0144.html)) and 
[nodes](http://wiki.ros.org/action/fullsearch/ROS/Patterns/Conventions#Nodes) which we will 
use as guidance to name the packages and nodes for the 2WD robot.

When possible, the default name of a node should follow from the name of the executable used to launch the node. 
This default name can be remapped at startup to something unique.
{: .notice }

To follow [REP-144](https://www.ros.org/reps/rep-0144.html), each sensor will get its own package with one 
or more nodes to be executed. Following this scheme it will be possible to use the sensors in other projects and work 
with existing packages such as packages from the [navigation stack](http://wiki.ros.org/navigation).

## Steps to Control a Robot using ROS

The steps to control a new robot using ROS are[ref](http://shop.oreilly.com/product/0636920024736.do): 

1. Decide on the ROS message interface. 
2. Write drivers for the robot's motors. 
3. Write a model of the robot's physical structure. 
4. Extend the model with physical properties for use in simulation with Gazebo. 
5. Publish coordinate transform data via tf and visualize it with rviz. 
6. Add sensors, with driver and simulation support. 
7. Apply standard algorithms, such as navigation.



## Packages and Nodes

| Category            | Package                 | Nodes         | topic                  |
|:-------------------:|:-----------------------:|:-------------:|:----------------------:|
| actuator/control    | grove_motor_driver      | motor_driver  | /cmd                   |
| sensor/localization | grove_motor_driver      | speedsensor   | /odom                  |
| sensor/perception   | grove_ultrasonic_driver | ranger        | /distance              | 
| sensor/perception   | rpi_camera_driver       | camera        | /2wdrobotcam/image_raw |


## Using Sensors and Actuators in ROS

To integrate new sensors and actuators into the ROS ecosystem 
involves writing ROS wrappers around the APIs that we're already using to access these devices.

## Adding Sensors

If a sensor already has a Python API it is relatively straightforward to use sensors in ROS.
First you should always verify that things are working as expected before you start to wrap up a sensor in ROS. 
If you know that the sensor is working, then anything that goes wrong will be a problem with the ROS wrapper, 
which will make things easier to debug.


In ROS there are two ways to work with sensor data. 
One way is that the sensor publishes topics which other nodes can subscribe to.
Another way is that the sensor node provides measurments only when asked for using either 
a service or action call which other nodes can use and thereby act as clients.
This decision depends on how we are going to use the sensor.

Another decision to make is how we’re going to access data from the sensor, 
which depends on the sensor.

Finally we need to decide what type of ROS message our wrapper will produce.
Whenever possible we should use existing ROS message types because it allows us
to reuse the wrapped sensor with other nodes that use the same interface types.



## Create a new Catkin package

To create a new Catkin package when in the `src` folder of your [ROS workspace](http://wiki.ros.org/catkin/workspaces) use the [`catkin create pkg`](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_create.html#catkin-create-pkg) command:

```bash
$ catkin create pkg PKG_NAME [--catkin-deps [DEP [DEP ...]]]
```
For example the `grove_ultrasonic_ranger` package is created with the following command:

```bash
fjp@ubuntu:~/git/2wd-robot/ros/src$ catkin create pkg grove_ultrasonic_ranger --catkin-deps rospy roscpp sensor_msgs
Creating package "grove_ultrasonic_ranger" in "/home/fjp/git/2wd-robot/ros/src"...
Created file grove_ultrasonic_ranger/CMakeLists.txt
Created file grove_ultrasonic_ranger/package.xml
Created folder grove_ultrasonic_ranger/include/grove_ultrasonic_ranger
Created folder grove_ultrasonic_ranger/src
```


## References

[**Programming Robots with ROS** A Practical Introduction to the Robot Operating System](http://shop.oreilly.com/product/0636920024736.do)
