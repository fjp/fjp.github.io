---
layout: single #collection
title: Autonomous 2WD Robot - ROS Packages and Nodes
permalink: /projects/diffbot/ros-packages/
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
  # teaser: /assets/collections/diffbot/assembly/motor/08-motor-driver-power.jpg
  # overlay_image: /assets/collections/diffbot/assembly/motor/08-motor-driver-power.jpg
  overlay_filter: 0.5 # same as adding an opacity of 0.5 to a black background
  # caption: "Source: [**hpiracing**](http://www.hpiracing.com/de/kit/114343)"
  show_overlay_excerpt: true
sidebar:
  nav: "diffbot"
---

On the [file system level](https://wiki.ros.org/ROS/Concepts#ROS_Filesystem_Level), ROS uses [Packages](http://wiki.ros.org/Packages) which are part of every ROS distribution such as ROS Melodic and defined as follows:

Packages are the main unit for organizing software in ROS. A package may contain ROS runtime processes ([nodes](http://wiki.ros.org/Nodes)), a ROS-dependent library, datasets, configuration files, or anything else that is usefully organized together. Packages are the most atomic build item and release item in ROS. Meaning that the most granular thing you can build and release is a package.
{: .notice }

## Best Practices

The project follows [ROS best practices](http://wiki.ros.org/BestPractices)

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

## Standard Units of Measure and Coordinate Conventions

Please see [REP 103](http://www.ros.org/reps/rep-0103.html) for documentation on the standard units of measure and coordinate conventions followed here.


## Packages and Nodes

| Category            | Package                 | Nodes         | topic                  |
|:-------------------:|:-----------------------:|:-------------:|:----------------------:|
| actuator/control    | grove_motor_driver      | motor_driver  | /cmd                   |
| sensor/localization | grove_motor_driver      | speedsensor   | /odom                  |
| sensor/perception   | grove_ultrasonic_driver | ranger        | /distance              | 
| sensor/perception   | rpi_camera_driver       | camera        | /2wdrobotcam/image_raw |






## Create a new Catkin package

To create a new Catkin package when in the `src` folder of your [ROS workspace](http://wiki.ros.org/catkin/workspaces) use the [`catkin create pkg`](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_create.html#catkin-create-pkg) command:

```bash
$ catkin create pkg PKG_NAME [--catkin-deps [DEP [DEP ...]]]
```
For example the `grove_ultrasonic_ranger` package is created with the following command:

```bash
fjp@ubuntu:~/git/diffbot/ros/src$ catkin create pkg grove_ultrasonic_ranger --catkin-deps rospy roscpp sensor_msgs
Creating package "grove_ultrasonic_ranger" in "/home/fjp/git/diffbot/ros/src"...
Created file grove_ultrasonic_ranger/CMakeLists.txt
Created file grove_ultrasonic_ranger/package.xml
Created folder grove_ultrasonic_ranger/include/grove_ultrasonic_ranger
Created folder grove_ultrasonic_ranger/src
```
