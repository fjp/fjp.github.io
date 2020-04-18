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

## Receiver and Flight Controller Connection

As mentioned the receiver and the flight controller need to be connected via a port that is capable of transmitting 
telemetry data. In this case it is the FPort protocol. 

## Betaflight Configuration

The correct communication protocol needs to be set in the flight controler software. In Betaflight this can be done
on the "Configuration" tab:

<figure>
    <a href="/assets/collections/fpv/betaflight/betaflight-config-receiver.jpg"><img src="/assets/collections/fpv/betaflight/betaflight-config-receiver.jpg"></a>
    <figcaption>Configure the Flight Controller.</figcaption>
</figure>

Here the protocol is set to FPort and the telemetry switch is enabled.

## Transmitter Setup

First, delete any telemetry.

<figure class="half">
    <a href="/assets/collections/fpv/telemetry/01-telemetry-delete-all.jpg"><img src="/assets/collections/fpv/telemetry/01-telemetry-delete-all.jpg"></a>
    <a href="/assets/collections/fpv/telemetry/02-confirm-delete-all.jpg"><img src="/assets/collections/fpv/telemetry/02-confirm-delete-all.jpg"></a>
    <figcaption>Delete all telemetry data.</figcaption>
</figure>

Deletion is required to discover new telemetry data:

<figure>
    <a href="/assets/collections/fpv/telemetry/03-discover-new-sensors.jpg"><img src="/assets/collections/fpv/telemetry/03-discover-new-sensors.jpg"></a>
    <figcaption>Discover telemetry data.</figcaption>
</figure>

The final result should look like the following:

<figure class="third">
    <a href="/assets/collections/fpv/telemetry/04-telemetry-results.jpg"><img src="/assets/collections/fpv/telemetry/04-telemetry-results.jpg"></a>
    <a href="/assets/collections/fpv/telemetry/05-telemetry-results.jpg"><img src="/assets/collections/fpv/telemetry/05-telemetry-results.jpg"></a>
    <a href="/assets/collections/fpv/telemetry/06-telemetry-results.jpg"><img src="/assets/collections/fpv/telemetry/06-telemetry-results.jpg"></a>
    <figcaption>Discovered telemetry data.</figcaption>
</figure>

Afterwards it is possible to stop the discovery and to begin the setup of different telemetry screens.





