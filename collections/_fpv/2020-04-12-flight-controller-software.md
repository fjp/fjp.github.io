---
layout: single
title:  "Flight Controller Software"
permalink: /projects/fpv/flight-controller-software
excerpt: "Software for a Flight Controller and its configuration aspects."
date:   2020-04-12 09:00:35 +0100
categories: [fpv, quad]
tags: [fpv, quad, race, drone, flight controller, software, betaflight, cleanflight]
comments: true
use_math: true
toc: true
classes: wide
# toc_label: "Unscented Kalman Filter"
header:
  teaser: /assets/collections/fpv/betaflight/betaflight-ports.png
  overlay_image: /assets/collections/fpv/betaflight/betaflight-ports.png
  overlay_filter: 0.5
  caption: "Source: [**Betaflight**](https://github.com/betaflight/betaflight)"
sidebar:
  nav: "fpv"
---

A [flight controller](/projects/fpv/glossar#flight-controller) can be operated with different software 
([firmware](/projects/fpv/glossar#firmware)). Popular software for FPV race copters are 
[Cleanflight](http://cleanflight.com/), its successor [Betaflight](https://betaflight.com/)
and [LibrePilot](https://www.librepilot.org/site/index.html). Another software used specifically for FPV races is
KISS from [Flyduino](https://kiss.flyduino.net/) with its related flight controller. 
We will use Betaflight because it supports a wide variety of flight controllers.

- **Cleanflight**: Simple interface which is used via a Chrome browser application window. 
- **Betaflight**: A software fork of Cleanflight and specifically optimized for race copters.
- **LibrePilot**: Software with lots of configuration options and a well structured menu to adjust the PID values.

Betaflight is an open source [flight controller](/projects/fpv/glossar#flight-controller) software (firmware)
which supports a variety of flight controllers, which comes with an easy to use and intuitive graphical user interface (GUI). Although the name implies that it is a beta version, which it get's because it is a fork of the popular Cleanflight software, it is well established and should be seen as a progression to Cleanflight. 
Betaflight's was intentionally developed for race copters which is why it provides faster communication protocols (loop time) and therefore faster and more percise control of the copter.


<figure >
    <a href="/assets/collections/fpv/betaflight/betaflight-ports.png"><img src="/assets/collections/fpv/betaflight/betaflight-ports.png"></a>
    <figcaption>Betaflight Configurator Graphical User Interface (GUI) showing the Ports tab.</figcaption>
</figure>

