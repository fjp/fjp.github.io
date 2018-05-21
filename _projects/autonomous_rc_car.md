---
layout: collection #collection
title: Autonomous RC Car
permalink: /projects/autonomous-rc-car/
excerpt: "Equipped with ROS and Arduino"
header:
  overlay_image: /assets/projects/autonomous-rc-car/hpi-racing-bmw-m3.png
  overlay_filter: 0.5 # same as adding an opacity of 0.5 to a black background
  caption: "Source: [**hpiracing**](http://www.hpiracing.com/de/kit/114343)"
  show_overlay_excerpt: true
sidebar:
  nav: "autonomous_rc_car"
---

## Introduction
This project aims to build a self driving rc car equipped with a Raspberry Pi 3 B+ running ROS and an Arduino MKR1000 to control the motor and the servo for steering. To find its way around it uses a camera. The project is split into four parts, to adress the following main components of the robot.

- Arduino MKR1000 responsible for the low level control.
- Raspbery Pi 3 B+ running ROS to perceive the environment and act accordingly.
- The rc vehicle and its parts
- System integration: combining the components.


In this part I will explore the MKR1000 and its capabilities using the Serial connection and a library called [rosserial](http://wiki.ros.org/rosserial). This library provides a client library called rosserial_arduino. It allows users to get ros nodes up and running easily.

## Prerequisites

If you followed the previous tutorial to setup a ubuntu 16.04 guest virtual machine on virtualbox running on macOS, you are ready to follow this tutorial. However, it’s not required to have ubuntu running on a virtualbox. Feel free to install ubuntu 16.04 directly on your machine to get the best experience regarding performance.

The reason we use ubuntu 16.04 is that it is an LTS version and recommended by the [ROS Kinetic](http://wiki.ros.org/ROS/Installation) distribution which is itself has LTS, supported until April, 2021.
