---
layout: single #collection
title: Autonomous 2WD Robot - Integrating a Robot into ROS
permalink: /projects/2wd-robot/ros-integration/
excerpt: "Integrating an autonomous 2WD robot into ROS."
date: 2020-02-15 15:00:35 +0100
categories: [robotics]
tags: [2wd, robot, ros, melodic, raspberry, pi, autonomous, ubuntu, package, integration]
comments: true
use_math: true
toc: true
classes: wide
# toc_label: "Unscented Kalman Filter"
header:
  teaser: /assets/collections/2wd-robot/ros/ros_control/gazebo_ros_control.png
  overlay_image: /assets/collections/2wd-robot/ros/ros_control/gazebo_ros_control.png
  overlay_filter: 0.5 # same as adding an opacity of 0.5 to a black background
  # caption: "Source: [**hpiracing**](http://www.hpiracing.com/de/kit/114343)"
  show_overlay_excerpt: true
sidebar:
  nav: "2wd-robot"
---


## Steps to Integrate a Robot into ROS

The steps to control a new robot using ROS are [[1](http://shop.oreilly.com/product/0636920024736.do)]: 

1. Decide on the ROS message interface. 
2. Write drivers for the robot's motors. 
3. Write a model of the robot's physical structure. 
4. Extend the model with physical properties for use in simulation with Gazebo. 
5. Publish coordinate transform data via `tf` and visualize it with `rviz`. 
6. Add sensors, with driver and simulation support. 
7. Apply standard algorithms, such as control and navigation.



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

To control the 2WD robot and get its sensor information we will need interfaces to its hardware.
In this project the Raspberry Pi 4 B is used as the main processing unit. 
It provides physical hardware interfaces such as GPIO pins, USB and a camera connector.
We will use these interfaces to connect the robots hardware components to the Raspberry Pi.
These interfaces operate on communication protocols where we can leverage existing libraries to work with these protocols.
For example the RPi.GPIO library provides methods to use the I2C protocol and work with hardware interrupts.

Anothe important aspect is to use the hardware interface to convert between the robot's 
native representation of commands and data and the interfaces that ROS supports. 
Here we need to consider which existing ROS packages we are going to use and what types of message interfaces they use.
Then we can apply math to the raw hardware signals to bring it in a form that is suitable for the ROS message types.

In this project the 2WD robot has two motors which operate on a voltage level value applied to them.
In combination with the wheel encoder ticks, these values need to be brought into a "ROS format". 
Specifically, ROS uses the notion of joints (revolute, rotational, continuous, ...) with standard units
for different pysical quantities, such as position (m), velocity (m/s), angle (rad), and angular velocity (rad/s).


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

The steps to use the [`diff_drive_controller`](http://wiki.ros.org/diff_drive_controller), ([repository on GitHub](https://github.com/ros-controls/ros_controllers/tree/melodic-devel/diff_drive_controller)) are:

1. Install `ros_control` on Ubuntu from the Debian packages (recommended):

```bash
sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers
```

2. URDF File
3. YAML Files
4. Write a hardware interface: [ROS Tutorial](http://wiki.ros.org/ros_control/Tutorials/Create%20your%20own%20hardware%20interface), [Slaterobots blog post](https://slaterobots.com/blog/5abd8a1ed4442a651de5cb5b/how-to-implement-ros_control-on-a-custom-robot), [ros_control_boilerplate](https://github.com/PickNikRobotics/ros_control_boilerplate) from [Dave Coleman](https://github.com/davetcoleman), [eborghi10/my_ROS_mobile_robot](https://github.com/eborghi10/my_ROS_mobile_robot/blob/e04acfd3e7eb4584ba0aab8a969a74d6c30eed34/my_robot_base/include/my_robot_hw_interface.h#L78-L99) using Joint Command Interface and Joint State Interface as these are required for the `diff_drive_controller`.

<figure>
    <a href="/assets/collections/2wd-robot/ros/ros_control/gazebo_ros_control.png"><img src="/assets/collections/2wd-robot/ros/ros_control/gazebo_ros_control.png"></a>
    <figcaption>ros control structure (Source: <a href="http://wiki.ros.org/ros_control">ROS.org ros_control</a>).</figcaption>
</figure>

### ROS Navigation


### Using `rviz` to Localize and Command a Navigating Robot



## References

[**Programming Robots with ROS** A Practical Introduction to the Robot Operating System](http://shop.oreilly.com/product/0636920024736.do)
