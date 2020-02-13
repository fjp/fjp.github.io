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

## Standard Units of Measure and Coordinate Conventions

Please see [REP 103](http://www.ros.org/reps/rep-0103.html) for documentation on the standard units of measure and coordinate conventions followed here.

## Steps to Control a Robot using ROS

The steps to control a new robot using ROS are[ref](http://shop.oreilly.com/product/0636920024736.do): 

1. Decide on the ROS message interface. 
2. Write drivers for the robot's motors. 
3. Write a model of the robot's physical structure. 
4. Extend the model with physical properties for use in simulation with Gazebo. 
5. Publish coordinate transform data via `tf` and visualize it with `rviz`. 
6. Add sensors, with driver and simulation support. 
7. Apply standard algorithms, such as navigation.



## ROS Message Interface

First we need to get control of the mobile base using a ROS node that communicates with the hardware
and then presents a standard ROS interface to the rest of the system. 
Doing so follows a common and core concept of ROS: abstraction. Similar acting robots can reuse standard interfaces
and thus take advantage of the large ROS ecosystem consisting of tools and libraries. 

The defining characteristics of the 2WD robot are that it is mobile and driving on the ground. 
Its kinematic configuration is similar to a tricycle:

- translate forward and backward (along its x-axis)
- yaw (rotate about its z-axis)
- combinations of the two. 

Because the robot is moving only in 2 dimensions, it cannot translate side to side (y-axis) or up and down (z-axis). 
Neither can it roll or pitch its body (rotation about its x- or y-axes, respectively).

To control it, we can make use of the [`geometry_msgs/Twist`](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html) 
(cmd_vel topic) message type interface. With this, the desired linear velocity vx along the x-axis (by convention,
positive is forward) and the rotational velocity vyaw about the z-axis (by convention, positive is counter-clockwise) 
is sent as a command to the robot.

Using the `rosmsg show` command we can see what's in the `geometry_msgs/Twist` message type:

```bash

```

Some of the fields (specifically, linear/y, linear/z, angular/x, or angular/y) are not required for the 2WD robot and
will therefore not be used.

To know how far the robot traveled, we expect to report its position and orientation in the plane as (x, y, yaw). 
The ROS community uses the [`nav_msgs/Odometry`](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html) (odom topic) 
ROS message interface to receive the position and orientation as data from the robot. 

```bash

```

To report the robot's position and orientation, we only really need to fill out the `pose/pose/ position` and 
`pose/pose/orientation` fields, ignoring the covariance fields 
(which are only needed for downstream components that reason about uncertainty). 
Within `pose/pose/position`, we only need to fill out `x` and `y`. 
Working with `pose/pose/ orientation` requires to construct a valid quaternion that represents a 3D orientation. 


<p>
The field called header, of type std_msgs/Header, is contained in many ROS messages and contains important information
for the correct interpretation of many types of data in a robot system:
<ul>
  <li> at what time the data was produced </li>
  <li> in what coordinate frame it is represented </li>
</ul>
The tf library uses this information to transform between different coordinate frames.
</p>
{: .notice }

We use these generic interfaces from ROS, 
as it allows us to used existing libraries and tools that can operate on these message types.


## Hardware Driver


## Modeling the Robot: URDF


## Simulation in Gazebo


## Verifying Transforms

Publish coordinate transform data via `tf` and visualize it with `rviz`. 


## Adding Sensors and Actuators in ROS

To integrate new sensors and actuators into the ROS ecosystem with driver and simulation support,
involves writing ROS wrappers around the APIs used to access these devices.

### Adding Sensors

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

### Adding Actuators


## Utilizing and Configuring Existing ROS Packages

Apply standard algorithms, such as control and navigation.

### ROS Control

The `ros_control` repositories provide agnostic controllers of different type, such as the `diff_drive_controller`,
that allow us to interact over a gerneric hardware interface. For a concise introduciton to `ros_control`, watch 
this [ROSCon 2014 talk](https://vimeo.com/107507546) from [Adolfo Rodríguez Tsouroukdissian](https://github.com/adolfo-rt).


### ROS Navigation


### Using `rviz` to Localize and Command a Navigating Robot


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
fjp@ubuntu:~/git/2wd-robot/ros/src$ catkin create pkg grove_ultrasonic_ranger --catkin-deps rospy roscpp sensor_msgs
Creating package "grove_ultrasonic_ranger" in "/home/fjp/git/2wd-robot/ros/src"...
Created file grove_ultrasonic_ranger/CMakeLists.txt
Created file grove_ultrasonic_ranger/package.xml
Created folder grove_ultrasonic_ranger/include/grove_ultrasonic_ranger
Created folder grove_ultrasonic_ranger/src
```


## References

[**Programming Robots with ROS** A Practical Introduction to the Robot Operating System](http://shop.oreilly.com/product/0636920024736.do)
