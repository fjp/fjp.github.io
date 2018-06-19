---
layout: single
title:  "MinIMU 9 v5 and RTIMULib2"
permalink: /projects/autonomous-rc-car/minimu9v5/
excerpt: "Use the Pololu MinIMU 9 v5 with the RTIMULib2."
date:   2018-06-19 20:55:35 +0200
categories: [rc-car, autonomous, sensors]
tags: [imu, gyroscope, gyro, accelerometer, accel, magnetometer, mag, compass, interal measurement unit]
comments: true
use_math: true
toc: true
classes: wide
# toc_label: "Unscented Kalman Filter"
header:
  teaser: https://a.pololu-files.com/picture/0J7057.1200.jpg?1b03058e38d92f82f95abf7a0aa39315 #/assets/projects/autonomous-rc-car/hpi-racing-bmw-m3_thumb.png
  overlay_image: https://a.pololu-files.com/picture/0J7057.1200.jpg?1b03058e38d92f82f95abf7a0aa39315 #/assets/projects/autonomous-rc-car/hpi-racing-bmw-m3.png
redirect_from:
  - /projects/autonomous-rc-car/
---


## Introduction

This blog post describes how to connect the [Pololu MinIMU 9 v5](https://www.pololu.com/product/2738) to a Rasperry Pi 3 B+ and use
the [RTIMULib2]() to visualize the inertial measurement unit (imu) data such as orientation and acceleration.

## Prerequisites

This post requires that you have setup a Rasperry Pi running Linux Mate described in another blog post or the standard [Raspbian OS](https://www.raspberrypi.org/downloads/raspbian/).

## Hardware Connection

The MinIMU has six pins, which are shown in the following figure.

{% include figure image_path="https://a.pololu-files.com/picture/0J7068.1200.jpg?b556a123006d9828b014751124c74296" caption="Pololu MinIMU-9 v5" %}

To communicate with the IMU the GPIO pins of the raspberrypi can be used. By taking a look at the

{% include figure image_path="/assets/posts/2018-06-19-minimu9v5-rtimulib2/rpiblusleaf.png" caption="Rasberry Pi Bus leaf" %}


### Standard Parts

The transmitter and the receiver are not required to drive autonomously and therefore not explained further.
However, if we wanted to interact with the car remotely we need to take those two parts into account.

#### Servo

Requires voltage to operate which is established by two wires that can be looked up in its data sheet.
The third cable is used to command the servo to its desired angle position using a pulse width modulated signal.

#### Electronic Speed Control (ESC) and Electric Motor

The ESC is connected to the battery and drives the motor through plus and minus connections by varying the supplied voltage.

## Parts for Autonomous Mode

To operate autonomously we require the parts that we setup previously and some additional power supply hardware to operate them.

- Arduino MKR1000
- Raspberry Pi 3 B+
- Battery Pack
- Voltage regulator to have a steady 5V power supply from the battery pack.

Although it would be possible to use power from the car's battery, I decided to use a separate battery pack to power the "brain"
(Raspberry and Arduino) of the car. This decision is made to not waste the the car's main energy source and it has enough power
for to carry this additional weight.
