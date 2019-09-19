---
layout: single
title:  "Arduino MKR1000 and rosserial servo control"
permalink: /projects/autonomous-rc-car/rosserial/
excerpt: "Use the Arduino MKR1000 together with rosserial for steering a servo."
date:   2018-06-21 22:56:20 +0200
categories: [arduino, ros, servos, control]
tags: [arduino, mkr1000, ros, servos, control, actuator]
comments: true
use_math: true
toc: true
#classes: wide
# toc_label: "Unscented Kalman Filter"
header:
  teaser: /assets/projects/autonomous-rc-car/futabaS3003.jpg
  overlay_image: /assets/projects/autonomous-rc-car/pwm-duty.png
redirect_from:
  - /projects/autonomous-rc-car/
---


## Introduction

This project aims to build a self driving rc car equipped with a Raspberry Pi 3 B+ running ROS and an Arduino MKR1000 to control the motor and the servo for steering. To find its way around it uses a camera. The project is split into four parts, to adress the following main components of the robot.

Arduino MKR1000 responsible for the low level control.
Raspbery Pi 3 B+ running ROS to perceive the environment and act accordingly.
The rc vehicle and its parts
System integration: combining the components.

In this part I will explore the MKR1000 and its capabilities using the Serial connection with a library called [rosserial](http://wiki.ros.org/rosserial). This library provides a client library called rosserial_arduino. It allows us to get ros nodes up and running easily.

## Prerequisites

Ubuntu 16.04 including Arduino IDE and ROS Kinetic.

For a How-To on installing the Arduino IDE, connecting the Arduino MKR1000 to your computer and uploading the rosserial hello world sketch, checkout [this blog post](/projects/autonomous-rc-car/arduino-mkr1000/).


## Hardware

I am going to work with two servos:

- Tower Pro Micro Servo 99 SG90 [datasheet](http://www.ee.ic.ac.uk/pcheung/teaching/DE1_EE/stores/sg90_datasheet.pdf).
- Futaba S3003 [datasheet](http://www.es.co.th/schemetic/pdf/et-servo-s3003.pdf).

## Servo Example

To our convenience there is already a servo example available from the arduino rosserial library. To start the upload, go to the following menu.

{% include figure image_path="/assets/posts/2018-06-21-arduino-mkr1000-ros-serial/image13.png" caption="Servo - Arduino rosserial" %}
