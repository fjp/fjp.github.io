---
layout: single #collection
title: Autonomous 2WD Robot - Components
permalink: /projects/2wd-robot/components/
excerpt: "Components of an autonomous 2WD Robot equipped with a Raspberry Pi 4 B running ROS melodic to sense and act in an environment."
date: 2019-11-17 09:00:35 +0100
categories: [robotics]
tags: [2wd, robot, ros, melodic, raspberry, pi, autonomous, sensors]
comments: true
use_math: true
toc: true
# classes: wide
# toc_label: "Unscented Kalman Filter"
header:
  teaser: /assets/collections/2wd-robot/components/components.jpg
  overlay_image: /assets/collections/2wd-robot/components/components.jpg
  overlay_filter: 0.5 # same as adding an opacity of 0.5 to a black background
  # caption: "Source: [**hpiracing**](http://www.hpiracing.com/de/kit/114343)"
  show_overlay_excerpt: true
sidebar:
  nav: "2wd-robot"
---

Part list and assembly of the robot platform and the components.

| Category     | Hardware          | Part Number                                       | Data Sheet & Info       |
|:------------:|:-----------------:|:-------------------------------------------------:|:-----------------------:|
| Accessories                                 |
|              | Micro SD Card     | SanDisk 64GB Class 10                             | [SanDisk](), Ubuntu 18.04 Image |
|              | Robot Car Kit 2WD | [robot05](https://joy-it.net/en/products/robot05) | [Instructions manual](https://joy-it.net/files/files/Produkte/robot05/Robot05-Anleitung.pdf) |
|              | Power bank        | Intenso Powerbank S10000                          | [Intenso](https://www.intenso.de/en/products/powerbanks/powerbank-s10000) |
| Actuator                                                                                  |
|              | Gearbox motor     | DC Gearbox motor - "TT Motor" - 200RPM - 3 to 9VDC | [Adafruit](https://www.adafruit.com/product/3777) |
| Board                                                                                    |
|              | Raspberry Pi 4 B  | Raspberry Pi 4 B - 4 GB                           | [OEM Website](https://www.raspberrypi.org/products/raspberry-pi-4-model-b/) |
| Cables                                                                                   |
|             | Jumper - Female to Female |                                           |                         |
|              | Jumper - Male to Male     |                                           |                         |
|              | Micro USB - USB Cable     |                                           |                         |
|              | Camera extension cable    |                                           |                         |
|              | I2C 4 pin cable           |                                           |                         |
| Electronics                                                                                    |
|              | I2C motor driver          | Grove - I2C Motor Driver                  | [Seeed Studio](http://wiki.seeedstudio.com/Grove-I2C_Motor_Driver_V1.3/) |
|              | I2C Hub                   | Grove - I2C Hub                           | [Seeed Studio](http://wiki.seeedstudio.com/Grove-I2C_Hub/) |
| Human Machine Interface                   |
|              | OLED Display   | Grove OLED Display 0.96"                  | [Seeed Studio](http://wiki.seeedstudio.com/Grove-OLED_Display_0.96inch/) |
|              | LED Ring                  | NeoPixel Ring 12x5050 RGB LED             |  [Adafruit](https://www.adafruit.com/product/1643) |
| Sensors                                                                                   |
|              | Camera module             | Raspberry Pi - camera module v2.1         | [Raspberry Pi](https://www.raspberrypi.org/documentation/usage/camera/) |
|              | Ultrasonic ranger         | Grove - Ultrasonic Ranger                 | [Seeed Studio](http://wiki.seeedstudio.com/Grove-Ultrasonic_Ranger/) | 
|              | IMU                       | Adafruit 9-DOF Absolute Orientation IMU Fusion Breakout - BNO055 | [Adafruit](https://www.adafruit.com/product/2472) |
|              | Odometry                  | Joy-IT - LM393 Speed Sensor with H206 slot-type opto interrupter | [Joy-IT](https://joy-it.net/en/products/SEN-Speed) |



## Accessories and Electronics

### Robot Base

The [Robot Car Kit 2WD](https://joy-it.net/en/products/robot05) from Joy-IT (article no.: robot05) is used as the base for the autonomous mobile robot. 

<figure>
    <a href="/assets/collections/2wd-robot/car-kit05.jpg"><img src="/assets/collections/2wd-robot/car-kit05.jpg"></a>
    <figcaption>Parts of the 2WD Robot Car Kit 05 from Joy-IT.</figcaption>
</figure>

The acrylic chassis has many holes which allow to mount a mounting plate that can hold different development boards.
It allows also to mount a Raspberry Pi 4 B, which will be used in this project. Two wheels, hence 2WD, are included in the kit which can be attached to the motors that are provided too. A third caster wheel is provided which allows the robot to spin on the spot. This means the robot can be described by a holonomic model. 

The motors operate in a range between 3 to 9 Volts DC and make it possible to mount a punched disk for speed measurements. 
With that punched disk and additional speed sensors it is possible to implement odometry in ROS. 
To power the motors a battery compartment is available together with a switch to turn the robot on or off.


The following image shows how I mounted the robot car kit. Notice that I placed the battery compartment on the bottom of 
the acrylic chassis instead of on top of it as suggested in the [instructions manual](https://joy-it.net/files/files/Produkte/robot05/Robot05-Anleitung.pdf) (seems to be only available in german). 
The reason is that I will use an additional USB-C powerbank to power the Raspberry Pi 4 B.

TODO figure

To assemble the robot car kit it is important to mount the motors on the correct side of the acrylic plate becasue otherwise
the mounting plate where the RPi will be attached will not fit to the pre drilled holes.

### Power Supplies

As mentioned the robot will be equipped with a USB-C powerbank to supply the RPi 4 B with 5 V. 
To power the motors the provided battery compartment will be used, which holds four AA batteries $4 \cdot 1.5\text{V} = 6\text{V}$.

## Sensors

Sensors are used to sense the environment and to collect information of the current state.
For this 2WD robot the sensors are categorized into perception and localization which are explained in the following two sections.

### Perception

Perception sensors of the 2WD robot will be used to avoid collisions using ultrasonic rangers.
Another use case is to detect and follow objects using a camera.

#### Ultrasonic Ranger

To avoid obstacles the robot will carry a [Grove - Ultrasonic Ranger](http://wiki.seeedstudio.com/Grove-Ultrasonic_Ranger/)
at the front. It is a non-contact distance measurement module which works at 40KHz and can be interfaced via a single [GPIO](https://www.raspberrypi.org/documentation/usage/gpio/). For example [physical pin 11](https://pinout.xyz/pinout/pin11_gpio17) of the Raspberry Pi connected to the `SIG` pin on the sensor can provide the PWM communication.

|Parameter|	Value/Range|
|:------|:------------------|
|Operating voltage|	3.2~5.2V|
|Operating current|	8mA|
|Ultrasonic frequency|	40kHz|
|Measuring range|	2-350cm|
|Resolution|	1cm|
|Output|PWM|
|Size|50mm X 25mm X 16mm|
|Weight|13g|
|Measurement angle|15 degree|
|Working temperature|-10~60 degree C|
|Trigger signal|10uS TTL|
|Echo signal|TTL|

The code that will be used to wrap this sensor as a ROS node can be found in the [Grove Raspberry Pi](https://github.com/Seeed-Studio/Grove-RaspberryPi/blob/master/Grove%20-%20Ultrasonic%20Ranger/ultrasonic.py) repository on GitHub. The code 

As an alternative we could use the [HC SR04](https://www.seeedstudio.com/blog/2019/11/04/hc-sr04-features-arduino-raspberrypi-guide/).

#### Camera

<figure>
    <a href="/assets/collections/2wd-robot/components/rpi-camera.jpg"><img src="/assets/collections/2wd-robot/components/rpi-camera.jpg"></a>
    <figcaption>RPi Camera v2.</figcaption>
</figure>


### Localization

#### Odometry

To estimate the change in position over time ([odometry](https://en.wikipedia.org/wiki/Odometry)) the robot will
utilize an [optical speed sensor](https://en.wikipedia.org/wiki/Wheel_speed_sensor#Optical_sensor). 
Specifically the LM393 ([datasheet](http://www.ti.com/lit/ds/symlink/lm2903-n.pdf)) [comperator](https://en.wikipedia.org/wiki/Comparator) combined with a H206 slot-type opto interrupter. [Joy-IT Speed Sensor](https://joy-it.net/en/products/SEN-Speed).

<figure class="half">
    <a href="/assets/collections/2wd-robot/components/speed-sensor-front.jpg"><img src="/assets/collections/2wd-robot/components/speed-sensor-front.jpg"></a>
    <a href="/assets/collections/2wd-robot/components/speed-sensor-back.jpg"><img src="/assets/collections/2wd-robot/components/speed-sensor-back.jpg"></a>
    <figcaption>LM393 Speed Sensor from Joy-IT.</figcaption>
</figure>

References:
https://dronebotworkshop.com/robot-car-with-speed-sensors/


#### Inertial Measurement Unit

An intertial measurement unit (IMU) measures the acceleration and orientation through gyroscopes directly.
Other states such as the velocity can then be calculated.
For this the [Adafruit 9-DOF Absolute Orientation IMU Fusion Breakout - BNO055]((https://www.adafruit.com/product/2472)) is used.

<figure>
    <a href="/assets/collections/2wd-robot/components/bno055.jpg"><img src="/assets/collections/2wd-robot/components/bno055.jpg"></a>
    <figcaption>9-DOF Absolute Orientation IMU Fusion Breakout - BNO055 from Adafruit.</figcaption>
</figure>

## Actuators

- [Grove - I2C Motor Driver V1.3](http://wiki.seeedstudio.com/Grove-I2C_Motor_Driver_V1.3/)

### Control

To drive the two motors of the car kit we use the 
[Grove - I2C Motor Driver V1.3](http://wiki.seeedstudio.com/Grove-I2C_Motor_Driver_V1.3/) from Seeed Studio.

<figure>
    <a href="/assets/collections/2wd-robot/components/motor-driver.jpg"><img src="/assets/collections/2wd-robot/components/motor-driver.jpg"></a>
    <figcaption>Grove - I2C Motor Driver.</figcaption>
</figure>


### Brushed Gearbox Motor

<figure>
    <a href="/assets/collections/2wd-robot/components/gearbox-motor-close.jpg"><img src="/assets/collections/2wd-robot/components/gearbox-motor-close.jpg"></a>
    <figcaption>DC Gearbox motor - "TT Motor" - 200RPM - 3 to 9VDC.</figcaption>
</figure>

## Human Machine Interface (HMI)

### OLED Display

http://wiki.seeedstudio.com/Grove-OLED_Display_0.96inch/

Connected to the RPi via i2c on the physical pins 27 (scl) and 28 (sda), refere to the [pinout](https://pinout.xyz/pinout/i2c).

[Library](https://github.com/DexterInd/GrovePi/blob/master/Software/Python/grove_i2c_oled_128_64/grove_128_64_oled.py)
