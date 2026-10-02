---
layout: single
title:  "The Flight Controller"
permalink: /projects/fpv/flightcontroller
excerpt: "The Flight Controller is the brain of a FPV quad."
categories: [fpv, quad]
tags: [fpv, quad, race, drone, flight controller, fc, betaflight]
comments: true
use_math: true
toc: true
classes: wide
sidebar:
  nav: "fpv"
# Draft: not built on GitHub Pages. Preview locally with `bundle exec jekyll serve --unpublished`.
# To publish remove this line, add a date and enable the matching entry in _data/navigation.yml.
published: false
---

Short introduction: tasks of the [Flight Controller](/projects/fpv/glossar#flight-controller) and how it connects to the other components.

## Processor

- F4, F7 and H7 microcontrollers, loop time

## Sensors

- [IMU](/projects/fpv/glossar#imu) (gyroscope and accelerometer)
- Barometer (BMP280 on the MATEKSYS F722-STD)
- Current sensor on the [PDB](/projects/fpv/pdb)

## OSD and Blackbox

- On screen display of battery voltage, current and [RSSI](/projects/fpv/glossar#rssi)
- Blackbox logging for [PID tuning](/projects/fpv/pid/#tuning)

## UARTs and Pinout

- Which UART is used for receiver, [VTX](/projects/fpv/glossar#vtx) and telemetry, see the Betaflight ports tab on the [flight controller software](/projects/fpv/flight-controller-software) page

## Mounting

- 30.5 x 30.5 mm stack with the [PDB](/projects/fpv/pdb), soft mounting against vibrations

## Flight Controller Used in this Build

- MATEKSYS F722-STD, see [components](/projects/fpv/components) and [assembly](/projects/fpv/assembly)
