---
layout: single
title:  "Autonomous RC Car Parts"
permalink: /projects/autonomous-rc-car/rc-car-parts/
excerpt: "This project aims to build a self driving rc car equipped with a Raspberry Pi 3 B+ running ROS and an Arduino MKR1000 to control the motor and the servo for steering."
date:   2018-04-25 17:31:41 +0200
categories: [rc-car, autonomous]
tags: [rc-car, hpi, hpiracing, battery pack, camera]
comments: true
use_math: true
toc: true
classes: wide
# toc_label: "Unscented Kalman Filter"
header:
  teaser: /assets/images/hpi-racing-bmw-m3_thumb.png
  overlay_image: /assets/images/hpi-racing-bmw-m3.png
redirect_from:
  - /projects/autonomous-rc-car/
---


## Introduction

This is the third part of the self driving rc-car project. For a better overview of
the whole project, read the [introduction](/projects/autonomous-rc-car/).
I will list the parts that I will use to build a self driving rc-car.

## Prerequisites

This post requires that you have setup a Rasperry Pi and Arduino MKR1000 which make up the "brain" of
the autonomous rc car. I will list these components in the following sections and explain how I connected them to the car.

## RC Electric Car

I am using a [Ready to Run (RTR)](https://en.wikipedia.org/wiki/Radio-controlled_car).
RC cars come in different sizes compared to their real world originals.
At [HPI Racing](https://www.hpiracing.com) I found an electric on-road [BMW M3](https://www.hpiracing.com/en/kit/114343) which is 1:10 of the original size.
However, your free to choose any other type of rc vehicle to work through this project.


An rc car is made up of the following components

- **Transmitter** to control the car manually.
- **Receiver** (usually 3 channels) including antenna to receive commands from the transmitter.
- **Servo** is connected to one of the receiver channels and used to steer the rc car.
- **Electronic Speed Control (ESC)** is hooked to one channel of the receiver and uses the received signals to control the motor.
- **Electric Motor** is connected with its power source cables to the ESC and drives the car depending on the supplied power.
- **Battery** to power the car's components.

How such standard components are hooked up together can be seen in the following figure.

TODO

In the following sections I will explain the standard components and explore the main electronic components that are used to drive autonomously.

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
