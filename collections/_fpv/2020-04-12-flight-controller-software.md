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
  teaser: https://camo.githubusercontent.com/8178215d6cb90842dc95c9d437b1bdf09b2d57a7/687474703a2f2f7374617469632e726367726f7570732e6e65742f666f72756d732f6174746163686d656e74732f362f312f302f332f372f362f61393038383930302d3232382d62665f6c6f676f2e6a7067
  overlay_image: https://camo.githubusercontent.com/8178215d6cb90842dc95c9d437b1bdf09b2d57a7/687474703a2f2f7374617469632e726367726f7570732e6e65742f666f72756d732f6174746163686d656e74732f362f312f302f332f372f362f61393038383930302d3232382d62665f6c6f676f2e6a7067
  overlay_filter: 0.5
  caption: "Source: [**Betaflight**](https://github.com/betaflight/betaflight)"
redirect_from:
  - /projects/fpv/
sidebar:
  nav: "fpv"
---

A [flight controller](/projects/fpv/glossar#flight-controller) can be operated with different software 
([firmware](/projects/fpv/glossar#firmware)). Popular software for FPV race copters are 
[Cleanflight](http://cleanflight.com/), its successor [Betaflight](https://betaflight.com/)
and [LibrePilot](https://www.librepilot.org/site/index.html). Another software used specifically for FPV races is
KISS from [Flyduino](https://kiss.flyduino.net/) with its related flight controller. 
We will use Betaflight because it supports a wide variety of flight controllers.

<figure >
    <a href="/assets/collections/fpv/esc/dys-aria-escs.jpg"><img src="/assets/collections/fpv/esc/dys-aria-escs.jpg"></a>
    <figcaption>Four DYS Aria ESCs to control each motor individually.</figcaption>
</figure>

