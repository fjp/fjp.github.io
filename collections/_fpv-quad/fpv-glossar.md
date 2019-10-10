---
layout: single #collection
title: FPV Glossar
permalink: /projects/fpv-quad/fpv-glossar/
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

## BetaFlight

The BetaFlight open source [Flight Controller](/projects/fpv-quad/fpv-glossar/#flight-controller) [frimware](/projects/fpv-quad/fpv-glossar/#firmware) project (found on [GitHub](https://github.com/betaflight)) provides
[frimwares](/projects/fpv-quad/fpv-glossar/#firmware) for [Flight Controllers](/projects/fpv-quad/fpv-glossar/#flight-controller). To configure and [flash](/projects/fpv-quad/fpv-glossar/#flash) a firmware onto a Flight Controller we use 
the [BetaFlight Configurator](/projects/fpv-quad/fpv-glossar/#betaflight-configurator).

BetaFlight is a fork of the [CleanFlight](/projects/fpv-quad/fpv-glossar/#clean-flight) project, which is considered less experimental. However, modern Flight Controllers are supported mostly by BetaFlight. 

## BetaFlight Configurator

Part of the [BetaFlight](/projects/fpv-quad/fpv-glossar/#betaflight) open source [Flight Controllers](/projects/fpv-quad/fpv-glossar/#flight-controller) [frimware](/projects/fpv-quad/fpv-glossar/#firmware) project.
It is a cross platform (runs on most operating systems, Windows, Linux, MacOS) configuration tool for the BetaFlight [frimware](/projects/fpv-quad/fpv-glossar/#firmware).

## CleanFlight

## ESC

The Electronic Speed Controller (ESC) is conected to the [PDB](/projects/fpv-quad/fpv-glossar/#pdb) and controls the speed of a motor by adjusting its rpms (rotation per minutes). A quad copter uses four ESCs which can be part of the [Flight Controller](/projects/fpv-quad/fpv-glossar/#flight-controller). The input signal to the ESC comes from the Flight Controller, which tells the ESC at which speed a motor should run.

## Flight Controller

[Micro Controller]() board that contains input and output (I/O) pins and a processing unit (microchrip), which runs a Flight Controller
[frimware](/projects/fpv-quad/fpv-glossar/#firmware), that computes output signals for external or internal [ESCs]() by processing the input signals. Input signals are the [receiver](/projects/fpv-quad/fpv-glossar/#receiver) and other external sensors. A Flight Controller usually
contains multiple internal [sensors](/projects/fpv-quad/fpv-glossar/#sensor) such as [IMUs](/projects/fpv-quad/fpv-glossar/#imu).

## Firmware

In the context of [FPV](/projects/fpv-quad/fpv-glossar/#fpv) a firmware is the software that runs on the [Flight Controller](/projects/fpv-quad/fpv-glossar/#flight-controller)

## FPV

Abbreviation for first person view, where the live image from a flying quad is view through an [analog](/projects/fpv-quad/fpv-glossar/#analog) video receiving system. This can be either a [fpv goggle](/projects/fpv-quad/fpv-glossar/#goggle) or monitor.

## FrSky

Company that produces radio controlled [transmitters](/projects/fpv-quad/fpv-glossar/#transmitter) and [receivers](/projects/fpv-quad/fpv-glossar/#receiver), which are most common in the FPV scene. At their hompage [https://www.frsky-rc.com/](https://www.frsky-rc.com/) you can see all the products and download manuals and firmware updates. 

## Goggle

Used to view the [analog](/projects/fpv-quad/fpv-glossar/#analog) live image captured by the camera on the quad, which is transmitted with the video transmitter that sits also on the quad.

## PDB

The Power Distribution Board (PDB) acts as the heart of an [FPV](/projects/fpv-quad/fpv-glossar/#fpv) quad. 
It is connected to the [Battery](/projects/fpv-quad/fpv-glossar/#battery) and distributes its power to other components of the quad. The main components its is connected to are the [ESCs](/projects/fpv-quad/fpv-glossar/#esc) to power the 
[motors](/projects/fpv-quad/fpv-glossar/#motor). A PDB usually has additional voltage outputs such as 5V and 12V to power 
[sensors](/projects/fpv-quad/fpv-glossar/#sensor) or [LEDs](/projects/fpv-quad/fpv-glossar/#led).

## Receiver

The receiver (also refered to as `Rx`) is installed in the quad or radio controlled (rc) vehicle and communicates with the [transmitter](/projects/fpv-quad/fpv-glossar/#transmitter)


## Transmitter

Also known as radio is the radio controlled (rc) part that communicates with a [receiver](/projects/fpv-quad/fpv-glossar/#receiver). The term `Tx` is commonly referred to transmitting unit.

The most common manufacturar for FPV quad transmitters is [FrSky](/projects/fpv-quad/fpv-glossar/#frsky)
