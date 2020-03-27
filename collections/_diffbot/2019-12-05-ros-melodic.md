---
layout: single #collection
title: Autonomous 2WD Robot - ROS Melodic
permalink: /projects/{{ label }}/ros-melodic/
excerpt: "ROS Melodic Setup on Raspberry Pi 4 for an autonomous 2WD Robot running ROS melodic to sense and act in an environment."
date: 2019-12-05 09:00:35 +0100
categories: [robotics]
tags: [2wd, robot, ros, melodic, raspberry, pi, autonomous, ubuntu, bionic]
comments: true
use_math: true
toc: true
classes: wide
# toc_label: "Unscented Kalman Filter"
header:
  teaser: /assets/images/melodic.jpg
  overlay_image: /assets/images/melodic.jpg
  overlay_filter: 0.5 # same as adding an opacity of 0.5 to a black background
  caption: "Source: [**ROS Melodic**](http://wiki.ros.org/melodic)"
  show_overlay_excerpt: true
sidebar:
  nav: "{{ label }}"
---

The following guide shows how to setup [ROS Melodic](http://wiki.ros.org/melodic) and other important tools specific to ROS. 

## Setup ROS Melodic

The ROS distribution Melodic is supported by Ubuntu 18.04 which is why we are going to install it.
Instructions can be found on the offical [ROS documentation](http://wiki.ros.org/melodic/Installation/Ubuntu).
Just follow these instructions and choose this configuration: `Desktop-Full Install: (Recommended)`. 
Although it is overkill, it will provide all examples that you might want to try.

{% include_relative {{ site.collections.label }}/docs/ros-setup.md %}


## ROS Style Guide

The project follows [ROS conventions](http://wiki.ros.org/ROS/Patterns/Conventions) and the [style guide](http://wiki.ros.org/PyStyleGuide) from ROS in writing Python code.

