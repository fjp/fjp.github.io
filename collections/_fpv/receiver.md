---
layout: single
title:  "The RC Receiver"
permalink: /projects/fpv/receiver
excerpt: "The RC receiver receives the commands from the transmitter."
categories: [fpv, quad]
tags: [fpv, quad, race, drone, receiver, rx, sbus, fport]
date: 2026-10-06 09:00:00 +0200
comments: true
use_math: true
toc: true
classes: wide
header:
  teaser: /assets/collections/fpv/receiver/r-xsr-top.jpg
sidebar:
  nav: "fpv"
---

The [receiver](/projects/fpv/glossar#receiver) on the quad receives the stick commands from the [transmitter](/projects/fpv/transmitter)
and passes them to the [flight controller](/projects/fpv/glossar#flight-controller).
In the other direction it sends [telemetry](/projects/fpv/telemetry) such as the battery voltage back to the radio.

<figure>
    <a href="/assets/collections/fpv/receiver/r-xsr-top.jpg"><img src="/assets/collections/fpv/receiver/r-xsr-top.jpg" alt="FrSky R-XSR receiver"></a>
    <figcaption>FrSky R-XSR receiver of this build.</figcaption>
</figure>

## Protocols

Two links are involved: the radio link between transmitter and receiver, and the wired connection between receiver and flight controller.

**Radio link.** The transmitter module and the receiver must use the same protocol.
FrSky uses ACCST D16 and its successor [ACCESS](/projects/fpv/glossar#access), TBS uses Crossfire,
and the open source [ExpressLRS](/projects/fpv/glossar#expresslrs) has become the most common link in FPV.

**Receiver to flight controller.** The receiver is connected to a UART of the flight controller:

| Protocol | Description |
|:--|:--|
| [SBUS](/projects/fpv/glossar#sbus) | FrSky's serial protocol for up to 16 channels, an inverted UART signal |
| [SmartPort](/projects/fpv/glossar#smartport) | FrSky's telemetry protocol on a separate wire |
| [FPort](/projects/fpv/glossar#fport) | Combines SBUS and SmartPort on a single wire in both directions |
| CRSF | Protocol of Crossfire and ExpressLRS for channels and telemetry in both directions |

The receiver of this build runs FPort, so a single wire carries the stick commands and the telemetry, see the [R-XSR page](/projects/fpv/r-xsr).

## Antennas

Most receivers have two antennas, which are mounted at a right angle to each other and outside of the carbon frame,
because carbon fiber shields the radio signal. See [receiver antennas](/projects/fpv/assembly#receiver-antennas) in the assembly.

## Failsafe

Failsafe defines what happens when the quad loses the radio link.
The receiver should stop sending signals in this case ("No pulses", see [failsafe mode](/projects/fpv/r-xsr#failsafe-mode)),
so that the flight controller detects the loss and Betaflight takes over in two stages
([Betaflight failsafe guide](https://betaflight.com/docs/wiki/guides/current/failsafe)):

1. **Stage 1 (guard period)**: for a short time, by default 1.5 seconds in current versions, Betaflight holds the channels at their fallback values.
   If the link comes back within this time, the quad flies on normally.
2. **Stage 2**: after the guard period Betaflight runs the selected failsafe procedure.
   By default it disarms and the quad drops, alternatively it can level out and land slowly, or fly back with GPS Rescue if a GPS is installed.

Test the failsafe on the bench with the propellers removed by switching off the radio.

## Receiver Used in this Build

This build uses the FrSky R-XSR, which is set up on the [R-XSR page](/projects/fpv/r-xsr). According to its manual it has these specifications:

| Property | Value |
|:--|:--|
| Channels | 16 (channels 1 to 16 over SBUS, 1 to 8 over CPPM) |
| Operating voltage | 3.5 to 10 V |
| Operating current | 70 mA at 5 V |
| Weight and dimensions | 1.5 g, 16 × 11 × 5.4 mm |
| Compatibility | FrSky X-series modules and radios in D16 mode, ACCESS with the ACCESS firmware |
