---
layout: single #collection
title: FPV Glossar
permalink: /projects/fpv/glossar
excerpt: "Glossar for FPV terms"
categories: [fpv, rc, quad]
tags: [fpv, rc, quad, getfpv, motors, brushless, esc, props, flightcontroller, antennas, camera, goggles, frsky, fatshark]
comments: true
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

## Analog

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

The BetaFlight open source [Flight Controller](/projects/fpv/glossar#flight-controller) [frimware](/projects/fpv/glossar#firmware) project (found on [GitHub](https://github.com/betaflight)) provides
[frimwares](/projects/fpv/glossar#firmware) for [FCs](/projects/fpv/glossar#flight-controller). To configure and [flash](/projects/fpv/glossar#flash) a firmware onto a Flight Controller we use 
the [BetaFlight Configurator](/projects/fpv/glossar#betaflight-configurator).

BetaFlight is a fork of the [CleanFlight](/projects/fpv/glossar/#clean-flight) project, which is considered less experimental. However, modern Flight Controllers are supported mostly by BetaFlight. 

## BetaFlight Configurator

Part of the [BetaFlight](/projects/fpv/glossar#betaflight) open source [Flight Controllers](/projects/fpv/glossar#flight-controller) [frimware](/projects/fpv/glossar#firmware) project.
It is a cross platform (runs on most operating systems, Windows, Linux, MacOS) configuration tool for the BetaFlight [frimware](/projects/fpv/glossar#firmware).

## Bind

Bind is referred to binding a [receiver](/projects/fpv/glossar#receiver) with a [transmitter](/projects/fpv/glossar#transmitter).

## BLHeli

[Frimware](/projects/fpv/glossar#firmware) for [ESCs](/projects/fpv/glossar#esc) that is used to process the input data from a [Flight Controller](/projects/fpv/glossar/#flight-controller) and translate that into suitable control commands for the motor. Although the firmware was originally developed for helicopters it is also used for multicopters. 
Another commonly used firmware for ESCs is called [SimonK](/prjects/fpv/glossar#simonk).

## CleanFlight



## DFU

DFU stands for "Device Firmware Update" which is a mode of a [Flight Controller](/projects/fpv/glossar#flight-controller) 
device that is required to [flash](/projects/fpv/glossar#flash) a new [Firmware](/project/fpv/glossar#firmware) onto a [FC](/projects/fpv/glossar#flight-controller). It depends on the FC board how to get the device into the DFU mode. 
Either you need to short the BL or BOOT pads (or press and hold the BOOT tactile button if your FC board has one) while 
plugging the USB into the Flight Controller board.

## EU LBT

EU [LBT](/projects/fpv/glossar#lbt) stands for European Union Listen Before Talk (or Transmit) and is a [firmware](/projects/fpv/glossar#firmware) version for [receivers](/projects/fpv/glossar/#receiver) and [transmitter](/projects/fpv/glossar#transmitter) modules, which is allowed in the geographical region of the EU. Another firmware version is the [FCC](/projects/fpv/glossar#fcc) version which can be used outside the EU.
Reference: [Brushless Whoop](https://brushlesswhoop.com/frsky-eu-lbt-vs-fcc/).

## FCC

Stands for Federal Communications Commission which regulates use of radio frequencies within the United States.
Reference: [Brushless Whoop](https://brushlesswhoop.com/frsky-eu-lbt-vs-fcc/).

## Flash

[Flashing](https://en.wikipedia.org/wiki/Firmware#Flashing) means to update a Software, 
also refered to as [Firmware](/projects/fpv/glossar#firmware), that runs on a device,
such as a [Flight Controller](/projects/fpv/glossar#flight-controller) board. The term "to flash" comes from the 
Flash storage component of a device where the Firmware is stored.

## FrSky

[FrSky](https://www.frsky-rc.com/) is a chinese company that manufactures modules for [rc](/projects/fpv/glossar#rc) toys such as [transmitters](/projects/fpv/glossar/#transmitter), [receivers](/projects/fpv/glossar#receiver) or [Flight Controllers](/projects/fpv/glossar#flight-controller). 

## FPort

[Rx](/projects/fpv/glossar#rx) protocol that acts as a communication interface between [receiver](/projects/fpv/glossar#receiver) and [flight controller](/projects/fpv/glossar#flight-controller). FPort is developed by [Betaflight's](/projects/fpv/glossar#betaflight) developer team and [FrSky](/projects/fpv/glossar#frsky) for its [receivers](/projects/fpv/glossar#receiver). FPort (possibly stands for “Frsky Port”?) is developed by both Betaflight Dev Team and Frsky

1. FPort combines [SBUS](/projects/fpv/glossar#sbus) and [Smartport](/projects/fpv/glossar#smartport) [Telemetry](/projects/fpv/glossar#telemetry) into one single wire
  - Simplify cable management and soldering
  - Save a UART port because SBUS and Smartport take up two separate UART’s
- FPort is an uninverted protocol, which should avoid doing “uninversion hacks” on F4 FC in future Frsky receivers
- FPort is slightly faster than SBUS
- [RSSI](/projects/fpv/glossar#rssi) works automatically (no need to pass through a channel)

References:

- [Oscar Liang - Setup FrSky FPort](https://oscarliang.com/setup-frsky-fport/)

## ESC

The Electronic Speed Controller (ESC) is conected to the [PDB](/projects/fpv/glossar#pdb) and 
controls the speed of a motor by adjusting its rpms (rotation per minutes). 
A quadcopter uses four ESCs which can be part of the [Flight Controller](/projects/fpv/glossar/#flight-controller). 
The input signal to an ESC comes from the Flight Controller, which tells the ESC at which speed a motor should run.

## Flight Controller

[Micro Controller]() board that contains input and output (I/O) pins and a processing unit (microchrip), 
which runs a Flight Controller [frimware](/projects/fpv/glossar#firmware). 
The Flight Controller acts as the brain of a drone.
By processing [sensor](/projects/fpv/glossar#sensor) input signals the Flight Controller is used to compute output signals for external or internal [ESCs](/projects/fpv/glossar/#esc) to keep level flight. Other input signals are used to adjust the [pose](/projects/fpv/glossar/#pose) of the quad in the air such as the [receiver](/projects/fpv/glossar/#receiver) and other internal or external sensors. A Flight Controller usually
contains multiple internal [sensors](/projects/fpv/glossar/#sensor) such as [IMUs](/projects/fpv/glossar#imu).

## Firmware

In the context of [FPV](/projects/fpv/glossar/#fpv) a [firmware](https://en.wikipedia.org/wiki/Firmware) is the software that runs on the [Flight Controller](/projects/fpv/glossar/#flight-controller)

## FPV

Abbreviation for first person view, where the live image from a flying quad is view through an [analog](/projects/fpv/glossar/#analog) video receiving system. This can be either a [fpv goggle](/projects/fpv/glossar/#goggle) or monitor.

## FrSky

Company that produces radio controlled [transmitters](/projects/fpv/glossar/#transmitter) and [receivers](/projects/fpv/glossar/#receiver), which are most common in the FPV scene. At their hompage [https://www.frsky-rc.com/](https://www.frsky-rc.com/) you can see all the products and download manuals and firmware updates. 

## Goggle

Used to view the [analog](/projects/fpv/glossar/#analog) live image captured by the camera on the quad, which is transmitted with the video transmitter that sits also on the quad.

## LBT

LBT stands for Listen Before Talk or Listen Before Transmit and describes the version of a 
[firmware](/projects/fpv/glossar/#firmware) for [transmitters](/projects/fpv/glossar/#transmitter) and 
[receivers](/projects/fpv/glossar/#transmitter). The LBT version also refered to as EU LBT is the allowed version 
in the European Union. Most of the [FrSky](/projects/fpv/glossar#frsky) 
receivers and transmitters are sometimes referenced as EU or non EU or [EU LBT](/projects/fpv/glossar/#eu-lbt) 
and [FCC](/projects/fpv/glossar/#fcc).

Reference: [Brushless Whoop](https://brushlesswhoop.com/frsky-eu-lbt-vs-fcc/).

## LED

## LiPo

Refers to a type of [battery](/projects/fpv/glossar#battery) and is the abbreviation for [__li__thium-ion __po__lymere](https://en.wikipedia.org/wiki/Lithium_polymer_battery)

[Light Emitting Diodes](https://en.wikipedia.org/wiki/Light-emitting_diode) are used as visual guidance for a quad copter.

## PDB

The Power Distribution Board (PDB) acts as the heart of an [FPV](/projects/fpv/glossar#fpv) quad. 
It is connected to the [Battery](/projects/fpv/glossar#battery) and distributes its power to other components of the quad. The main components its is connected to are the [ESCs](/projects/fpv/glossar/#esc) to power the 
[motors](/projects/fpv-quad/fpv-glossar#motor). A PDB usually has additional voltage outputs such as 5V and 12V to power 
[sensors](/projects/fpv/glossar/#sensor) or [LEDs](/projects/fpv/glossar#led).

## PWM

Short for [pulse width modulation](https://en.wikipedia.org/wiki/Pulse-width_modulation).

## Radio

The term radio is used for the [transmitter](/projects/fpv/glossar#transmitter) device.
It comes from the fact that [radio frequency](https://en.wikipedia.org/wiki/Radio_frequency) is used as a communication medium between the transmitter and receiver of [rc](/projects/fpv/glossar#rc)

## RC

Short for [radio](/projects/fpv/glossar#radio) controlled.

## Receiver

The receiver (also refered to as `Rx`) is installed in the quad or radio controlled (rc) vehicle and communicates with the [transmitter](/projects/fpv/glossar#transmitter)

## Redundancy

Describes a type of [receiver](/projects/fpv/glossar#receiver) which can be used in combination with other receivers 
of the same type to provide redundancy in case of a receiver failure.

## R-XSR

[Redundancy](/projects/fpv/glossar#redundancy) [receiver](/projects/fpv/glossar/#receiver) produced by [FrSky](/projects/fpv/glossar#frsky).


## Sensor

## SimonK

[Frimware](/projects/fpv/glossar#firmware) for [ESCs](/projects/fpv/glossar#esc) that is used to process the input 
data from a [Flight Controller](/projects/fpv/glossar/#flight-controller) and translate that into suitable control 
commands for the motor. This frimware was developed by Simon Kirby and its intended to be used in multicopters. 
Its source code can be found on [Simon Kirby's GitHub repository](https://github.com/sim-/tgy).
Another commonly used firmware for ESCs is [BLHeli](/prjects/fpv/glossar#blheli).

## Transmitter

Also known as [radio](/projects/fpv/glossar#radio) is the radio controlled ([rc](/projects/fpv/glossar#rc)) part that communicates with a [receiver](/projects/fpv/glossar/#receiver). The term `Tx` is commonly referred to transmitting units.
Such units can be external or internal in transmitter devices. External devices can be swapped. 

One of the most common manufacturers for [FPV](/projects/fpv/glossar#fpv) quad transmitters is [FrSky](/projects/fpv/glossar#frsky)

## UBEC

If the electronic components in a copter should be powered independently of the ESCs or if only optocoupler ESCs are used, there exist other ways to power the flight controller and other electronic components such as LEDs: It's possible to use an UBEC (Universial Battery Elimination Circuit). This device is connected to the battery and can be used to provide constant output voltage for electronic components.
