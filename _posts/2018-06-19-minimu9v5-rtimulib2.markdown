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

{% include figure image_path="/assets/posts/2018-06-16-minimu9v5-rtimulib2/rpiblusleaf.png" caption="Rasberry Pi Bus leaf" %}

## RTIMULib2

The RTIMULib2 did not support the LSM6DS33 and the LIS3MDL which are on the MinIMU-9 v5 PCB. Therefore, I wrote a driver to support the imu which
can be found in [this github repository](https://github.com/fjp/RTIMULib2).

To install the library, just clone the repository

{% highlight bash %}
git clone https://github.com/fjp/RTIMULib2
{% endhighlight %}

Then `cd` into the new RTIMULib2 folder and build and install the library

{% highlight bash %}
cd RTIMULib2/RTIMULib
mkdir build
cd build
cmake ..
make
sudo make install
{% endhighlight %}

The last command copies the library into the shared library path `LD_LIBRARY_PATH` of linux.

To run a test program, change into the Linux folder and build the example programs in there.

{% highlight bash %}
cd RTIMULib2/Linux
mkdir build
cd build
cmake ..
make
{% endhighlight %}

These commands will create new folders inside the build directory which contain the program executables.

The visual demo can be executed with the following lines of bash code:

{% highlight bash %}
cd RTIMULib2/Linux/build/RTIMULibDemoGL
make
./RTIMULibDemoGL
{% endhighlight %}

This should open a Qt GUI that lets you see the orientation of the imu.
The first start time you start the GUI a file called RTIMULib.ini is created which is used to store the imu settings (address of the chips, fusion state)
and calibration data. Furthermore, the IMU is not calibrated and will result in poor tracking behavior while rotating.
Therefore the sensors (magnetometer, accelerometer and gyroscope) need to be calibrated which I will explain in the next section.

### Calibration

Calibrate the magnetometer and the accelerometer using the buttons on top of the gui.
A tutorial on how to calibrate these sensors is given in the Calibration.pdf in the repository.
