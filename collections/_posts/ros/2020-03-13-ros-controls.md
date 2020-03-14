---
layout: single
title:  "ROS Control, An overview"
date:   2020-03-13 19:31:41 +0200
excerpt: "ROS Control combines a set of packages that include controller interfaces, controller managers, transmissions and hardware_interfaces."
permalink: /posts/ros/ros-kalman-filter/
categories: [robotics, state estimation, Kalman filter, ros]
tags: [robotics, state estimation, Kalman filter, c++, ros]
comments: true
use_math: true
toc: true
classes: wide
# toc_label: "Kalman filter"
header:
  teaser: /assets/collections/2wd-robot/ros/ros_control/gazebo_ros_control.png
  overlay_image: /assets/collections/2wd-robot/ros/ros_control/gazebo_ros_control.png #keep it square 200x200 px is good
---


ROS Control combines a set of packages that include controller interfaces, controller managers, transmissions and hardware_interfaces.


## Big Picture and Goals

Let's start with a typical scenario: you have a robot, or even more general, a piece of hardware, and you would 
like to run an application that leverages ROS specific libraries like the Navigation stack or MovieIt!. 
All these libraries are well tested and you don't want to rewrite them. On the other hand you probably have a 
computer that is running a simulator like Gazebo to simulate your robot. To make ends meet we can make use of the ROS control project.

<figure>
    <a href="/assets/ros/ros-control/ros-control-overview.png"><img src="/assets/ros/ros-control/ros-control-overview.png"></a>
    <figcaption>Overview of ROS control project (Source: <a href="http://wiki.ros.org/ros_control">ROS.org ros_control</a>).</figcaption>
</figure>


ROS control helps you writing controllers including real-time constraints, transmissions and joint limits. 
It provides tools for modeling the robotic in software, represented by the robot hardware interface (`hardware_interface::RobotHW`), and comes 
with ready to use out of the box controllers for different applications which then talk to third-party tools. 
The important aspects to highlight is that the hardware access to the robot is de-coupled the so we you don't expose how we talk to our robot.


## Ros Control and Friends


### Setting up a Robot

### Controllers

### The Control Loop





## Reference

- [ROSCon 2014 talk](https://vimeo.com/107507546)
