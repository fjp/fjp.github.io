---
layout: single
title:  "Taranis X9D Plus SE 2019 - Telemetry"
permalink: /projects/fpv/telemetry
excerpt: "All about telemetry and the steps to set it up on flight controller, receiver and transmitter."
date: 2019-10-30 09:00:35 +0100
categories: [fpv, quad]
tags: [fpv, quad, race, drone, betaflight, radio, transmitter, fport, telemetry, receiver]
comments: true
use_math: true
toc: true
classes: wide
# toc_label: "Unscented Kalman Filter"
header:
  teaser: /assets/collections/fpv/taranis/taranis.jpg
  overlay_image: /assets/collections/fpv/taranis/bootloader.jpg
  overlay_filter: 0.5
redirect_from:
  - /projects/fpv/
sidebar:
  nav: "fpv"
---

With [telemetry](/projects/fpv/glossar#telemetry) setup it is possible view the measured 
[sensor](/projects/fpv/glossar#sensor) data on a radio [transmitter](/projects/fpv/glossar#transmitter) 
via the [flight controller](/projects/fpv/glossar#flight-controller) and the [receiver](/projects/fpv/glossar#receiver). 
It is common that a flight controller contains multiple sensors with data that is sent to the transmitter over the receiver. 
This means that the receiver has to be capable of transmitting the telemetry data to the radio.

This post covers the telemetry setup using the [FPort](/projects/fpv/glossar#fport) of a [R-XSR](/projects/fpv/glossar#r-xsr) receiver
that is connected to a single [TX](/projects/fpv/glossar#tx) pin of the flight controller, in this case the MatekSys F722.
The flight controller runs with the [Betaflight](/projects/fpv/glossar#betaflight) software that provides the settings
to configure telemetry correctly on the flight controller, so that it can communicate the telemetry with the receiver via FPort.
As transmitter the Taranis X9D Plus SE 2019 is used which is capable of displaying the telemetry data in different screens.

<figure>
    <a href="/assets/collections/fpv/taranis/taranis.jpg"><img src="/assets/collections/fpv/taranis/taranis.jpg"></a>
    <figcaption>Taranis X9D Plus SE 2019 Carbon.</figcaption>
</figure>

## Receiver Setup

## Betaflight







