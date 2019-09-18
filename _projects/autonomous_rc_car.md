---
layout: single #collection
title: Autonomous RC Car
permalink: /projects/autonomous-rc-car/
excerpt: "Autonomous RC Car equipped with a Raspberry Pi running ROS and Arduino to control its servos."
header:
  overlay_image: /assets/projects/autonomous-rc-car/hpi-racing-bmw-m3.png
  overlay_filter: 0.5 # same as adding an opacity of 0.5 to a black background
  caption: "Source: [**hpiracing**](http://www.hpiracing.com/de/kit/114343)"
  show_overlay_excerpt: true
sidebar:
  nav: "autonomous_rc_car"
---

## Introduction
This project aims to build a self driving rc car equipped with a Raspberry Pi 3 B+ (use a Raspberry Pi 4 B for increased performance) running ROS and an Arduino MKR1000 to control the motor and the servo for steering. To find its way around it uses a camera. The project is split into four parts, to adress the following main components of the robot.

- Arduino MKR1000 responsible for the low level control.
- [Raspberry Pi 3 B+](https://de.aliexpress.com/item/32858825148.html?spm=a2g0o.productlist.0.0.5d232e8bvlKM7l&algo_pvid=2c45d347-5783-49a6-a0a8-f104d0b78232&algo_expid=2c45d347-5783-49a6-a0a8-f104d0b78232-0&btsid=0100feb4-37d7-453a-8ff8-47a0e2fbdef7&ws_ab_test=searchweb0_0,searchweb201602_9,searchweb201603_52) or even better a [Rapberry 4 B](https://de.aliexpress.com/item/4000054868537.html?spm=a2g0o.productlist.0.0.2ade7babA4VMjD&algo_pvid=883b9cf1-2bc7-49e3-8407-e71950629c5e&algo_expid=883b9cf1-2bc7-49e3-8407-e71950629c5e-0&btsid=af279edc-561a-4435-a3de-704beddbdadf&ws_ab_test=searchweb0_0,searchweb201602_9,searchweb201603_52) running ROS to perceive the environment and act accordingly.
- The rc vehicle and its parts
- System integration: combining the components.


In this part I will explore the MKR1000 and its capabilities using the Serial connection and a library called [rosserial](http://wiki.ros.org/rosserial). This library provides a client library called rosserial_arduino. It allows users to get ros nodes up and running easily.

## Prerequisites

If you followed the previous tutorial to setup a ubuntu 16.04 guest virtual machine on virtualbox running on macOS, you are ready to follow this tutorial. However, it’s not required to have ubuntu running on a virtualbox. Feel free to install ubuntu 16.04 directly on your machine to get the best experience regarding performance.

The reason we use ubuntu 16.04 is that it is an LTS version and recommended by the [ROS Kinetic](http://wiki.ros.org/ROS/Installation) distribution which is itself has LTS, supported until April, 2021.
