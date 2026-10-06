---
layout: single
title:  "The RC Transmitter"
permalink: /projects/fpv/transmitter
excerpt: "The RC transmitter, also known as radio, is used to control the quad."
categories: [fpv, quad]
tags: [fpv, quad, race, drone, transmitter, radio, tx, taranis]
date: 2026-10-06 09:00:00 +0200
comments: true
use_math: true
toc: true
classes: wide
header:
  teaser: /assets/collections/fpv/taranis/taranis.jpg
  overlay_image: /assets/collections/fpv/taranis/bootloader.jpg
  overlay_filter: 0.5
sidebar:
  nav: "fpv"
---

The RC [transmitter](/projects/fpv/glossar#transmitter), also called radio, is what the pilot holds in the hands.
It reads the sticks and switches and sends them to the [receiver](/projects/fpv/receiver) on the quad.
This build uses the FrSky [Taranis](/projects/fpv/taranis) X9D Plus SE 2019.

<figure>
    <a href="/assets/collections/fpv/taranis/taranis.jpg"><img src="/assets/collections/fpv/taranis/taranis.jpg" alt="FrSky Taranis X9D Plus SE 2019"></a>
    <figcaption>FrSky Taranis X9D Plus SE 2019 Carbon.</figcaption>
</figure>

## Stick Modes

The two sticks control throttle, yaw, pitch and roll. Which stick controls what is defined by the stick mode:

| Mode | Left stick | Right stick |
|:--|:--|:--|
| Mode 2 | Throttle (up and down) and yaw (left and right) | Pitch (up and down) and roll (left and right) |
| Mode 1 | Pitch (up and down) and yaw (left and right) | Throttle (up and down) and roll (left and right) |

Mode 2 is the most common mode for FPV quads. The throttle stick has no spring that centers it, so it stays where it is left.

## Firmware

The radio runs a firmware that acts as its operating system. It defines models, mixes, switches, [throttle curves](/projects/fpv/throttle-curves) and
[telemetry](/projects/fpv/telemetry) screens, and it runs Lua scripts such as the [Betaflight Lua script](/projects/fpv/pid-lua-script).
The Taranis of this build runs [OpenTX](/projects/fpv/glossar#opentx). Most radios today run its successor [EdgeTX](/projects/fpv/glossar#edgetx),
which also supports the Taranis X9D Plus 2019, see the [Taranis page](/projects/fpv/taranis).

## Internal and External Modules

The radio link is created by an RF module. The Taranis X9D Plus 2019 has an internal [ISRM](/projects/fpv/glossar#isrm) module,
which speaks FrSky's ACCESS and ACCST D16 protocols.
On its back it also has a JR module bay for external modules, for example for TBS Crossfire or [ExpressLRS](/projects/fpv/glossar#expresslrs).
This way the radio can be used with other receivers without replacing it.

## Pages in this Section

- [Taranis X9D Plus SE 2019](/projects/fpv/taranis): firmware, bootloader and internal module
- [Receiver binding R-XSR](/projects/fpv/r-xsr): receiver firmware, registration and binding
- [Throttle curves](/projects/fpv/throttle-curves)
- [RC rate and expo settings](/projects/fpv/rate-expo-settings)
- [Telemetry](/projects/fpv/telemetry)
- [PID Lua script](/projects/fpv/pid-lua-script)
