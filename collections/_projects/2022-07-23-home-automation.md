---
layout: single
title:  "Home Automation"
permalink: /projects/home-automation/
excerpt: "Home Automation based on KNX and Home Assistant."
date:   2022-07-23 09:00:35 +0100
categories: [home automation, raspberry pi, home assistant, knx]
tags: [knx, home, home automation, home assistant, home, assistant, raspberry pi, os, raspberry, ssd, setup, argon, one, m.2, case]
comments: true
use_math: true
toc: false
classes: wide
# toc_label: "Unscented Kalman Filter"
header:
  teaser: /assets/collections/home-automation/knx-wago.jpg
  overlay_image: /assets/collections/home-automation/knx-wago.jpg
  overlay_filter: 0.5
sidebar:
  nav: "home-automation"
---

Home Automation project based on KNX and Home Assistant. The pages in the menu describe how it is set up.

The features that this project includes are:

- KNX Bus
- Weather station
- Automatic blind control for shading
- Lighting control based on [DALI](https://en.wikipedia.org/wiki/Digital_Addressable_Lighting_Interface)
- Multi-room audio based on Sonos


Home Assistant now runs on a mini PC under Proxmox. How the KNX project gets into it is described in [From ETS to Home Assistant](/projects/home-automation/ets-to-home-assistant/), and how KNX buttons drive the Velux roof windows in [Velux Roof Windows with KNX Buttons](/projects/home-automation/velux-klf200-knx/).
The first setup from 2022, on a Raspberry Pi 4 booting from an SSD, stays as an [archived page](/projects/home-automation/setup/).
