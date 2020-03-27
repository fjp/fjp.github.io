---
layout: single #collection
title: Autonomous 2WD Robot - Robot Description Package
permalink: /projects/{{ label }}/ros-packages/robot-description/
excerpt: "ROS Robot Description Package for ROS Melodic running on a Raspberry Pi 4 for an autonomous 2WD Robot to act in an environment according to sensor information."
date: 2019-12-05 08:00:35 +0100
categories: [robotics]
tags: [2wd, differential drive, robot, ros, melodic, raspberry, pi, autonomous, ubuntu, bionic, package, urdf, xacro]
comments: true
use_math: true
toc: true
classes: wide
# toc_label: "Unscented Kalman Filter"
header:
  teaser: /assets/collections/{{ label }}/ros/urdf/robot.png
  overlay_image: /assets/collections/{{ label }}/ros/urdf/robot.png
  overlay_filter: 0.5 # same as adding an opacity of 0.5 to a black background
  # caption: "Source: [**hpiracing**](http://www.hpiracing.com/de/kit/114343)"
  show_overlay_excerpt: true
sidebar:
  nav: "{{ label }}"
---

ROS Robot Description Package for ROS Melodic running on a Raspberry Pi 4 for an autonomous 2WD Robot 
to act in an environment according to sensor information.


{% include_relative diffbot/docs/robot-description.md %}
