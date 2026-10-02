---
layout: single #collection
title: FPV Glossar
permalink: /projects/fpv/glossar
excerpt: "Glossar for FPV terms"
categories: [fpv, rc, quad]
tags: [fpv, rc, quad, getfpv, motors, brushless, esc, props, flightcontroller, antennas, camera, goggles, frsky, fatshark]
comments: true
use_math: true
toc: false
classes: wide
header:
  #overlay_image: /assets/projects/autonomous-rc-car/hpi-racing-bmw-m3.png
  #overlay_filter: 0.5 # same as adding an opacity of 0.5 to a black background
  #caption: "Source: [**hpiracing**](http://www.hpiracing.com/de/kit/114343)"
  #show_overlay_excerpt: true
sidebar:
  nav: "fpv"
author_profile: false
---

## ACCESS

ACCESS (Advanced Communication Control, Elevated Spread Spectrum) is the radio protocol [FrSky](/projects/fpv/glossar#frsky) introduced in 2019
as successor of its ACCST protocol. It is supported by the [ISRM](/projects/fpv/glossar#isrm) module of the [Taranis](/projects/fpv/glossar#taranis) X9D Plus SE 2019
and by ACCESS [receivers](/projects/fpv/glossar#receiver) such as the [R-XSR](/projects/fpv/glossar#r-xsr) with ACCESS [firmware](/projects/fpv/glossar#firmware).
Before an ACCESS receiver can be [bound](/projects/fpv/glossar#bind) it needs to be [registered](/projects/fpv/glossar#register) with the transmitter.

## Analog

In the context of [FPV](/projects/fpv/glossar#fpv) analog refers to the classic way of transmitting the live video from the [FPV camera](/projects/fpv/glossar#fpv-camera)
over a [VTX](/projects/fpv/glossar#vtx), usually in the 5.8 GHz band using the PAL or NTSC standard.
Analog video has a very low latency and its image quality degrades gradually with increasing distance or obstacles instead of freezing,
which is why it is popular for racing. Digital HD systems provide a much sharper image in exchange for a slightly higher latency.

## Battery

Batteries in a [FPV](/projects/fpv/glossar/#fpv) drone are connected to the [PDB](/projects/fpv/glossar/#pdb) to power the components. For drones [LiPo](/projects/fpv/glossar/#lipo) batteries are used 
because of their high energy density, which makes them weigh less and therefore improve the flight time.

A battery has two important measures. Its capacity and cell count that specify their voltage level.

Different LiPo battery packs are made up of individual cells with one cell having 3.7V. Typical cell counts are defined as:

| Cell count | Voltage |
|:----------:|:-------:|
| 2S         |  7.4 V  |
| 3S         |  11.1 V |
| 4S         |  14.8 V |

## BEC

Abbreviation for Battery Elimination Circuit which describes [ESCs](/projects/fpv/glossar#esc) that provide a portion of battery power (which the ESC is connected with) over their communication cable (red cable of the [servo](/projects/fpv/glossar#servo) connector) to other devices such as a [receiver](/projects/fpv/glossar#receiver) and servos attached to the receiver. 

## BetaFlight

The BetaFlight open source [Flight Controller](/projects/fpv/glossar#flight-controller) [firmware](/projects/fpv/glossar#firmware) project (found on [GitHub](https://github.com/betaflight)) provides
[firmwares](/projects/fpv/glossar#firmware) for [FCs](/projects/fpv/glossar#flight-controller). To configure and [flash](/projects/fpv/glossar#flash) a firmware onto a Flight Controller we use 
the [BetaFlight Configurator](/projects/fpv/glossar#betaflight-configurator).

BetaFlight is a fork of the [CleanFlight](/projects/fpv/glossar/#cleanflight) project, which is considered less experimental. However, modern Flight Controllers are supported mostly by BetaFlight. 

## BetaFlight Configurator

Part of the [BetaFlight](/projects/fpv/glossar#betaflight) open source [Flight Controllers](/projects/fpv/glossar#flight-controller) [firmware](/projects/fpv/glossar#firmware) project.
It is a cross platform (runs on most operating systems, Windows, Linux, MacOS) configuration tool for the BetaFlight [firmware](/projects/fpv/glossar#firmware).

## Bind

Bind is referred to binding a [receiver](/projects/fpv/glossar#receiver) with a [transmitter](/projects/fpv/glossar#transmitter).

## BLHeli

[Firmware](/projects/fpv/glossar#firmware) for [ESCs](/projects/fpv/glossar#esc) that is used to process the input data from a [Flight Controller](/projects/fpv/glossar/#flight-controller) and translate that into suitable control commands for the motor. Although the firmware was originally developed for helicopters it is also used for multicopters. 
Another commonly used firmware for ESCs is called [SimonK](/projects/fpv/glossar#simonk).

## CFK

German abbreviation for "Carbonfaserverstärkter Kunststoff", which is known as
[carbon fiber reinforced polymer](https://en.wikipedia.org/wiki/Carbon_fiber_reinforced_polymer) (CFRP) in English.
Because of its high stiffness and low weight it is the most common material for [FPV](/projects/fpv/glossar#fpv) [frames](/projects/fpv/frame) and [propellers](/projects/fpv/propeller).

## CleanFlight

Open source [Flight Controller](/projects/fpv/glossar#flight-controller) [firmware](/projects/fpv/glossar#firmware) (found on [GitHub](https://github.com/cleanflight)),
which started as a fork of the BaseFlight project. [BetaFlight](/projects/fpv/glossar#betaflight) was in turn forked from CleanFlight.

## DFU

DFU stands for "Device Firmware Update" which is a mode of a [Flight Controller](/projects/fpv/glossar#flight-controller) 
device that is required to [flash](/projects/fpv/glossar#flash) a new [Firmware](/projects/fpv/glossar#firmware) onto a [FC](/projects/fpv/glossar#flight-controller). It depends on the FC board how to get the device into the DFU mode. 
Either you need to short the BL or BOOT pads (or press and hold the BOOT tactile button if your FC board has one) while 
plugging the USB into the Flight Controller board.

## ESC

The Electronic Speed Controller (ESC) is connected to the [PDB](/projects/fpv/glossar#pdb) and 
controls the speed of a motor by adjusting its [rpm](/projects/fpv/glossar#rpm) (revolutions per minute). 
A quadcopter uses four ESCs which can be part of the [Flight Controller](/projects/fpv/glossar/#flight-controller). 
The input signal to an ESC comes from the Flight Controller, which tells the ESC at which speed a motor should run.
More details are found on the [ESC page](/projects/fpv/esc).

## EU LBT

EU [LBT](/projects/fpv/glossar#lbt) stands for European Union Listen Before Talk (or Transmit) and is a [firmware](/projects/fpv/glossar#firmware) version for [receivers](/projects/fpv/glossar/#receiver) and [transmitter](/projects/fpv/glossar#transmitter) modules, which is allowed in the geographical region of the EU. Another firmware version is the [FCC](/projects/fpv/glossar#fcc) version which can be used outside the EU.
Reference: [Brushless Whoop](https://brushlesswhoop.com/frsky-eu-lbt-vs-fcc/).

## FCC

Stands for Federal Communications Commission which regulates use of radio frequencies within the United States.
Reference: [Brushless Whoop](https://brushlesswhoop.com/frsky-eu-lbt-vs-fcc/).

## Firmware

In the context of [FPV](/projects/fpv/glossar/#fpv) a [firmware](https://en.wikipedia.org/wiki/Firmware) is the software that runs on the [Flight Controller](/projects/fpv/glossar/#flight-controller).
Other devices such as [ESCs](/projects/fpv/glossar#esc), [receivers](/projects/fpv/glossar#receiver) and [transmitters](/projects/fpv/glossar#transmitter) run their own firmware too.

## Flash

[Flashing](https://en.wikipedia.org/wiki/Firmware#Flashing) means to update a Software, 
also referred to as [Firmware](/projects/fpv/glossar#firmware), that runs on a device,
such as a [Flight Controller](/projects/fpv/glossar#flight-controller) board. The term "to flash" comes from the 
Flash storage component of a device where the Firmware is stored.

## Flight Controller

[Micro Controller](https://en.wikipedia.org/wiki/Microcontroller) board that contains input and output (I/O) pins and a processing unit (microchip), 
which runs a Flight Controller [firmware](/projects/fpv/glossar#firmware). 
The Flight Controller acts as the brain of a drone.
By processing [sensor](/projects/fpv/glossar#sensor) input signals the Flight Controller is used to compute output signals for external or internal [ESCs](/projects/fpv/glossar/#esc) to keep level flight. Other input signals are used to adjust the [pose](/projects/fpv/glossar/#pose) of the quad in the air such as the [receiver](/projects/fpv/glossar/#receiver) and other internal or external sensors. A Flight Controller usually
contains multiple internal [sensors](/projects/fpv/glossar/#sensor) such as [IMUs](/projects/fpv/glossar#imu).

## FPort

[Rx](/projects/fpv/glossar#rx) protocol that acts as a communication interface between [receiver](/projects/fpv/glossar#receiver) and [flight controller](/projects/fpv/glossar#flight-controller). FPort is developed by [Betaflight's](/projects/fpv/glossar#betaflight) developer team and [FrSky](/projects/fpv/glossar#frsky) for its [receivers](/projects/fpv/glossar#receiver).

- FPort combines [SBUS](/projects/fpv/glossar#sbus) and [Smartport](/projects/fpv/glossar#smartport) [Telemetry](/projects/fpv/glossar#telemetry) into one single wire
  - Simplify cable management and soldering
  - Save a UART port because SBUS and Smartport take up two separate UART’s
- FPort is an uninverted protocol, which should avoid doing “uninversion hacks” on F4 FC in future Frsky receivers
- FPort is slightly faster than SBUS
- [RSSI](/projects/fpv/glossar#rssi) works automatically (no need to pass through a channel)

References:

- [Oscar Liang - Setup FrSky FPort](https://oscarliang.com/setup-frsky-fport/)

## FPV

Abbreviation for first person view, where the live image from a flying quad is viewed through an [analog](/projects/fpv/glossar/#analog) video receiving system. This can be either a [fpv goggle](/projects/fpv/glossar/#goggle) or monitor.

## FPV Camera

Small and lightweight camera mounted at the front of the quad that captures the live image for the pilot.
FPV cameras are optimized for low latency and quickly changing light conditions rather than for a high resolution.
The video signal is sent to the [VTX](/projects/fpv/glossar#vtx), which transmits it to the [goggles](/projects/fpv/glossar#goggle).
The quad in this project uses a RunCam Swift 2 (see [components](/projects/fpv/components)).

## FrSky

[FrSky](https://www.frsky-rc.com/) is a Chinese company that manufactures modules for [rc](/projects/fpv/glossar#rc) toys such as [transmitters](/projects/fpv/glossar/#transmitter), [receivers](/projects/fpv/glossar#receiver) or [Flight Controllers](/projects/fpv/glossar#flight-controller).
Their transmitters and receivers are among the most common in the FPV scene.
At their homepage [https://www.frsky-rc.com/](https://www.frsky-rc.com/) you can see all the products and download manuals and firmware updates. 

## Goggle

Used to view the [analog](/projects/fpv/glossar/#analog) live image captured by the camera on the quad, which is transmitted with the video transmitter that sits also on the quad.

## IMU

An [inertial measurement unit](https://en.wikipedia.org/wiki/Inertial_measurement_unit) (IMU) is a [sensor](/projects/fpv/glossar#sensor) that measures
angular rates with a gyroscope and accelerations with an accelerometer. The [Flight Controller](/projects/fpv/glossar#flight-controller) uses these measurements
to estimate and control the [pose](/projects/fpv/glossar#pose) of the quad.

## ISRM

Name of the internal [transmitter](/projects/fpv/glossar#transmitter) module of [FrSky's](/projects/fpv/glossar#frsky) 2019 radios such as the [Taranis](/projects/fpv/glossar#taranis) X9D Plus SE 2019.
It supports the [ACCESS](/projects/fpv/glossar#access) protocol and the older ACCST D16 protocol, depending on the selected mode and [firmware](/projects/fpv/glossar#firmware).
How to update it is explained on the [Taranis page](/projects/fpv/taranis#update-internal-module-firmware-optional).

## LBT

LBT stands for Listen Before Talk or Listen Before Transmit and describes the version of a 
[firmware](/projects/fpv/glossar/#firmware) for [transmitters](/projects/fpv/glossar/#transmitter) and 
[receivers](/projects/fpv/glossar/#receiver). The LBT version also referred to as EU LBT is the allowed version 
in the European Union. Most of the [FrSky](/projects/fpv/glossar#frsky) 
receivers and transmitters are sometimes referenced as EU or non EU or [EU LBT](/projects/fpv/glossar/#eu-lbt) 
and [FCC](/projects/fpv/glossar/#fcc).

Reference: [Brushless Whoop](https://brushlesswhoop.com/frsky-eu-lbt-vs-fcc/).

## LED

[Light Emitting Diodes](https://en.wikipedia.org/wiki/Light-emitting_diode) are used as visual guidance for a quad copter.

## LiPo

Refers to a type of [battery](/projects/fpv/glossar#battery) and is the abbreviation for [__li__thium __po__lymer](https://en.wikipedia.org/wiki/Lithium_polymer_battery).

## Motor

Quads use brushless DC motors that are driven by an [ESC](/projects/fpv/glossar#esc) and spin the [propellers](/projects/fpv/propeller).
How they work and how to read their specifications is explained on the [motor page](/projects/fpv/motor).

## OpenTX

Open source [firmware](/projects/fpv/glossar#firmware) for RC [transmitters](/projects/fpv/glossar#transmitter) such as the [Taranis](/projects/fpv/glossar#taranis) X9D Plus.
It is configured on the radio itself or with [OpenTX Companion](/projects/fpv/glossar#opentx-companion).
Reference: [OpenTX](https://www.open-tx.org/).

## OpenTX Companion

Desktop application for Windows, Linux and MacOS that is used to configure models and settings of an [OpenTX](/projects/fpv/glossar#opentx) radio,
to back them up and to [flash](/projects/fpv/glossar#flash) new OpenTX [firmware](/projects/fpv/glossar#firmware) onto the radio.

## PDB

The Power Distribution Board (PDB) acts as the heart of an [FPV](/projects/fpv/glossar#fpv) quad. 
It is connected to the [Battery](/projects/fpv/glossar#battery) and distributes its power to other components of the quad. The main components it is connected to are the [ESCs](/projects/fpv/glossar/#esc) to power the 
[motors](/projects/fpv/glossar#motor). A PDB usually has additional voltage outputs such as 5V and 12V to power 
[sensors](/projects/fpv/glossar/#sensor) or [LEDs](/projects/fpv/glossar#led).

## Pose

The pose describes the position and orientation (attitude) of the quad in space.
The orientation is commonly expressed with the roll, pitch and yaw angles.

## PWM

Short for [pulse width modulation](https://en.wikipedia.org/wiki/Pulse-width_modulation).

## Radio

The term radio is used for the [transmitter](/projects/fpv/glossar#transmitter) device.
It comes from the fact that [radio frequency](https://en.wikipedia.org/wiki/Radio_frequency) is used as a communication medium between the transmitter and receiver of [rc](/projects/fpv/glossar#rc).

## RC

Short for [radio](/projects/fpv/glossar#radio) controlled.

## Receiver

The receiver (also referred to as [`Rx`](/projects/fpv/glossar#rx)) is installed in the quad or radio controlled (rc) vehicle and communicates with the [transmitter](/projects/fpv/glossar#transmitter).

## Redundancy

Describes a type of [receiver](/projects/fpv/glossar#receiver) which can be used in combination with other receivers 
of the same type to provide redundancy in case of a receiver failure.

## Register

Registering is a step required by the [ACCESS](/projects/fpv/glossar#access) protocol before a [receiver](/projects/fpv/glossar#receiver) can be [bound](/projects/fpv/glossar#bind).
During registration the receiver gets a name and stores the owner ID of the [transmitter](/projects/fpv/glossar#transmitter).
The steps for the R-XSR are shown on the [R-XSR page](/projects/fpv/r-xsr).

## RPM

Abbreviation for revolutions per minute, which is the unit used for the rotational speed of a [motor](/projects/fpv/glossar#motor).

## RSSI

Received Signal Strength Indicator. It indicates how good the radio link between [transmitter](/projects/fpv/glossar#transmitter) and [receiver](/projects/fpv/glossar#receiver) is
and is usually shown in the [goggles](/projects/fpv/glossar#goggle) or reported via [telemetry](/projects/fpv/glossar#telemetry) to warn before the link is lost.

## Rx

Short for [receiver](/projects/fpv/glossar#receiver).

## R-XSR

[Redundancy](/projects/fpv/glossar#redundancy) [receiver](/projects/fpv/glossar/#receiver) produced by [FrSky](/projects/fpv/glossar#frsky).
Its setup is explained on the [R-XSR page](/projects/fpv/r-xsr).

## SBUS

Serial bus protocol originally developed by Futaba and used by [FrSky](/projects/fpv/glossar#frsky) [receivers](/projects/fpv/glossar#receiver)
to transmit up to 16 channels over a single wire to the [Flight Controller](/projects/fpv/glossar#flight-controller).
SBUS is an inverted serial signal, which is why some Flight Controllers need a dedicated or inverted UART for it.

## Sensor

A sensor measures a physical quantity and provides it to the [Flight Controller](/projects/fpv/glossar#flight-controller).
Common sensors on a quad are the [IMU](/projects/fpv/glossar#imu) (gyroscope and accelerometer), a barometer to measure altitude
and a current sensor on the [PDB](/projects/fpv/glossar#pdb) to measure the current draw.

## Servo

A [servo](https://en.wikipedia.org/wiki/Servo_(radio_control)) is an actuator with position control used in RC models, for example to move control surfaces of a plane.
Its three wire connector (signal, power and ground) is the standard connector for RC equipment such as [receivers](/projects/fpv/glossar#receiver) and [ESCs](/projects/fpv/glossar#esc).

## SimonK

[Firmware](/projects/fpv/glossar#firmware) for [ESCs](/projects/fpv/glossar#esc) that is used to process the input 
data from a [Flight Controller](/projects/fpv/glossar/#flight-controller) and translate that into suitable control 
commands for the motor. This firmware was developed by Simon Kirby and its intended to be used in multicopters. 
Its source code can be found on [Simon Kirby's GitHub repository](https://github.com/sim-/tgy).
Another commonly used firmware for ESCs is [BLHeli](/projects/fpv/glossar#blheli).

## SmartPort

SmartPort (S.Port) is [FrSky's](/projects/fpv/glossar#frsky) bidirectional single wire protocol to transmit [telemetry](/projects/fpv/glossar#telemetry) data
from the [Flight Controller](/projects/fpv/glossar#flight-controller) and other sensors over the [receiver](/projects/fpv/glossar#receiver) to the [transmitter](/projects/fpv/glossar#transmitter).
It is also used to [flash](/projects/fpv/glossar#flash) firmware onto receivers. Like [SBUS](/projects/fpv/glossar#sbus) it is an inverted signal.

## Taranis

Series of RC [transmitters](/projects/fpv/glossar#transmitter) from [FrSky](/projects/fpv/glossar#frsky) running [OpenTX](/projects/fpv/glossar#opentx).
This project uses the [Taranis X9D Plus SE 2019](/projects/fpv/taranis).

## TBS

Team BlackSheep (TBS) is a manufacturer of FPV equipment such as the TBS Unify Pro [video transmitters](/projects/fpv/glossar#vtx) and the TBS Crossfire long range radio system.

## Telemetry

Telemetry is data sent from the quad back to the pilot, such as battery voltage, current draw or [RSSI](/projects/fpv/glossar#rssi).
With [SmartPort](/projects/fpv/glossar#smartport) or [FPort](/projects/fpv/glossar#fport) the [receiver](/projects/fpv/glossar#receiver) sends it to the [transmitter](/projects/fpv/glossar#transmitter),
where it can be displayed or used for alarms. The setup is explained on the [telemetry page](/projects/fpv/telemetry).

## Transmitter

Also known as [radio](/projects/fpv/glossar#radio) is the radio controlled ([rc](/projects/fpv/glossar#rc)) part that communicates with a [receiver](/projects/fpv/glossar/#receiver). The term [`Tx`](/projects/fpv/glossar#tx) is commonly referred to transmitting units.
Such units can be external or internal in transmitter devices. External devices can be swapped. 

One of the most common manufacturers for [FPV](/projects/fpv/glossar#fpv) quad transmitters is [FrSky](/projects/fpv/glossar#frsky).

## Tx

Short for [transmitter](/projects/fpv/glossar#transmitter).

## UBEC

If the electronic components in a copter should be powered independently of the ESCs or if only optocoupler ESCs are used, there exist other ways to power the flight controller and other electronic components such as LEDs: It's possible to use an UBEC (Universal Battery Elimination Circuit). This device is connected to the battery and can be used to provide constant output voltage for electronic components.

## VTX

The video transmitter (VTX) sends the live image of the [FPV camera](/projects/fpv/glossar#fpv-camera) to the [goggles](/projects/fpv/glossar#goggle),
usually in the 5.8 GHz band. Its output power and channel can often be changed from the [Flight Controller](/projects/fpv/glossar#flight-controller),
for example with the TBS SmartAudio protocol of the [TBS](/projects/fpv/glossar#tbs) Unify Pro used in this project.
