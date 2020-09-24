---
layout: single #collection
title: Autonomous 2WD Robot - ROS Noetic
permalink: /projects/diffbot/ros-noetic/
excerpt: "ROS Noetic Setup on Raspberry Pi 4 for an autonomous 2WD Robot running ROS melodic to sense and act in an environment."
date: 2019-12-05 09:00:35 +0100
categories: [robotics]
tags: [2wd, robot, ros, noetic, raspberry, pi, autonomous, ubuntu, focal fossa]
comments: true
use_math: true
toc: true
classes: wide
# toc_label: "Unscented Kalman Filter"
header:
  teaser: /assets/images/noetic.jpg
  overlay_image: /assets/images/noetic.jpg
  overlay_filter: 0.5 # same as adding an opacity of 0.5 to a black background
  caption: "Source: [**ROS Noetic**](http://wiki.ros.org/noetic)"
  show_overlay_excerpt: true
sidebar:
  nav: diffbot
---

The following guide shows how to setup [ROS Noetic](http://wiki.ros.org/noetic) and other important tools specific to ROS. 

## Setup ROS Melodic

The ROS distribution Noetic is supported by Ubuntu 20.04 which is why we are going to install it.
Instructions can be found on the offical [ROS documentation](http://wiki.ros.org/melodic/Installation/Ubuntu).
Just follow these instructions and choose this configuration: `Desktop-Full Install: (Recommended)`. 
Although it is overkill, it will provide all examples that you might want to try.

{% include_relative {{ page.collection }}/docs/ros-setup.md %}


## ROS Style Guide

The project follows [ROS conventions](http://wiki.ros.org/ROS/Patterns/Conventions) and the [style guide](http://wiki.ros.org/PyStyleGuide) from ROS in writing Python code.

