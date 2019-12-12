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
  teaser: /assets/collections/2wd-robot/assembly/motor/08-motor-driver-power.jpg
  overlay_image: /assets/collections/2wd-robot/assembly/motor/08-motor-driver-power.jpg
  overlay_filter: 0.5 # same as adding an opacity of 0.5 to a black background
  # caption: "Source: [**hpiracing**](http://www.hpiracing.com/de/kit/114343)"
  show_overlay_excerpt: true
sidebar:
  nav: "2wd-robot"
---

ROS Packages and Nodes for ROS Melodic running on a Raspberry Pi 4 for an autonomous 2WD Robot 
to act in an environment according to sensor information.

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

To create a new Catkin package when in your workspace use the following command:

```bash
$ catkin create pkg PKG_NAME
```
For example the `control` package is created with the following command:

```bash
fjp@ubuntu:~/git/2wd-robot/ros/src$ catkin create pkg control
Creating package "control" in "/home/fjp/git/2wd-robot/ros/src"...
Created file control/package.xml
Created file control/CMakeLists.txt
Successfully created package files in /home/fjp/git/2wd-robot/ros/src/control.
```
