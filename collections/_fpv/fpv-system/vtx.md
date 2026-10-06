---
layout: single
title:  "FPV Transmitter and Receiver"
permalink: /projects/fpv/fpv-system/vtx
excerpt: "The video transmitter sends the live image to the goggles."
categories: [fpv, quad]
tags: [fpv, quad, race, drone, vtx, video transmitter, smartaudio]
date: 2026-10-06 09:00:00 +0200
comments: true
use_math: true
toc: true
classes: wide
header:
  teaser: /assets/collections/fpv/vtx/tbs-unify-pro-hv.jpg
sidebar:
  nav: "fpv"
---

The [video transmitter](/projects/fpv/glossar#vtx) (VTX) on the quad sends the image of the [camera](/projects/fpv/fpv-system/camera) to the goggles,
where a video receiver picks it up.

## Bands, Channels and Output Power

Analog FPV uses the 5.8 GHz band. Its frequencies are grouped into bands of eight channels, such as A, B, E, FatShark and Raceband.
Raceband spaces its channels 37 MHz apart, so that up to eight pilots can fly at the same time:

| Channel | R1 | R2 | R3 | R4 | R5 | R6 | R7 | R8 |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|
| Frequency (MHz) | 5658 | 5695 | 5732 | 5769 | 5806 | 5843 | 5880 | 5917 |

In the EU, and therefore in Austria, the band from 5725 to 5875 MHz can be used without a license for short-range devices
with up to 25 mW e.i.r.p. (Commission Decision 2006/771/EC as amended by [Decision (EU) 2019/1345](https://eur-lex.europa.eu/legal-content/EN/TXT/HTML/?uri=CELEX:32019D1345), band 61). Of Raceband only R3 to R6 lie inside this band.
The effective isotropic radiated power (e.i.r.p.) includes the gain of the antenna: with an antenna gain of 2.2 dBic,
which is a factor of $10^{0.22} \approx 1.66$, a VTX set to 25 mW radiates about 41 mW e.i.r.p.
Higher power levels need other permissions, for example for races.

Most VTX also have a pit mode, in which they transmit with very low power.
It allows powering up a quad on the bench or at the start without interfering with pilots who are flying.

## Control from the Flight Controller

With TBS SmartAudio the [flight controller](/projects/fpv/glossar#flight-controller) changes band, channel and power of the VTX,
for example from the Betaflight OSD menu or with the [Lua script](/projects/fpv/pid-lua-script) on the radio.
The SmartAudio wire of the VTX is connected to a TX pad of a free UART on the flight controller,
and this UART is set to VTX (TBS SmartAudio) in the Ports tab of the [Betaflight Configurator](/projects/fpv/glossar#betaflight-configurator).
Since Betaflight 4.1 the available bands, channels and power levels are defined in a VTX table.

## Video Receiver

The video receiver of this build is the rapidFIRE module in the FatShark HDO goggles, see [FPV System](/projects/fpv/fpv-system/).
It is a diversity receiver, which uses two [antennas](/projects/fpv/fpv-system/antennas) to receive the signal of the VTX.

## VTX Used in this Build

This build uses the TBS Unify Pro HV with an SMA connector, see the [assembly](/projects/fpv/assembly).
Its specifications according to the [TBS manual](https://www.team-blacksheep.com/media/files/tbs-unify-pro-5g8-manual.pdf) and [getfpv](https://www.getfpv.com/fpv/video-transmitters/tbs-unify-pro-5g8-hv-sma.html):

| Property | Value |
|:--|:--|
| Output power | 25 mW (13 dBm), 200 mW (23 dBm), 500 mW (27 dBm), 800 mW (29 dBm) |
| Input | 6 to 28 V (2S to 6S), directly from the battery |
| Control | TBS SmartAudio, pit mode |
| Camera supply | 5 V output |
| Weight | 7 g with SMA, without antenna |

<figure class="half">
    <a href="/assets/collections/fpv/vtx/tbs-unify-pro-hv.jpg"><img src="/assets/collections/fpv/vtx/tbs-unify-pro-hv.jpg" alt="TBS Unify Pro HV video transmitter with SMA pigtail"></a>
    <a href="/assets/collections/fpv/assembly/vtx/01-tbs-unify-pro-manual.jpg"><img src="/assets/collections/fpv/assembly/vtx/01-tbs-unify-pro-manual.jpg" alt="Wiring card of the TBS Unify Pro HV"></a>
    <figcaption>TBS Unify Pro HV with its SMA pigtail and the wiring card that comes with it.</figcaption>
</figure>
