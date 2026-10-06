---
layout: single
title:  "FPV System"
permalink: /projects/fpv/fpv-system/
excerpt: "Camera, video transmitter, antennas and goggles of a FPV quad."
categories: [fpv, quad]
tags: [fpv, quad, race, drone, camera, vtx, goggles, antennas]
date: 2026-10-06 09:00:00 +0200
comments: true
use_math: true
toc: true
classes: wide
redirect_from:
  - /fpv-system/
header:
  teaser: /assets/collections/fpv/components/ultimate-fatshark-hdo-antenna-bundle.jpg
sidebar:
  nav: "fpv"
---

The FPV system transmits the live image from the quad to the pilot.
The [camera](/projects/fpv/fpv-system/camera) captures the image, the [video transmitter](/projects/fpv/fpv-system/vtx) (VTX) sends it on the 5.8 GHz band
over its [antenna](/projects/fpv/fpv-system/antennas), and the video receiver in the [goggles](/projects/fpv/glossar#goggle) shows it on two small displays in front of the pilot's eyes.

## Analog and Digital

The FPV system of this build is [analog](/projects/fpv/glossar#analog): the camera outputs a PAL or NTSC video signal,
which the VTX transmits without compressing it. Analog video has almost no latency, the equipment is light and inexpensive,
and when the signal gets weaker the image becomes noisy step by step instead of freezing. Its image quality is low compared to today's cameras.

Digital HD systems encode the image and transmit it digitally. The established systems are DJI, Walksnail Avatar and HDZero.
They show a much sharper image with 720p or 1080p, but they add some latency, the image can freeze or break up at the edge of the range,
and they cost several times as much as an analog system. [Oscar Liang's overview](https://oscarliang.com/fpv-system/) compares the current systems.

## Goggles

This build uses the FatShark HDO goggles with an ImmersionRC rapidFIRE receiver module.
The HDO has two OLED displays with 960 × 720 pixels in 4:3 format and a field of view of 37°
([Oscar Liang's review](https://oscarliang.com/fatshark-hdo-fpv-goggles/)).
The receiver module sits in a bay on the front of the goggles and can be replaced, for example by a module for another system.

<figure>
    <a href="/assets/collections/fpv/components/ultimate-fatshark-hdo-antenna-bundle.jpg"><img src="/assets/collections/fpv/components/ultimate-fatshark-hdo-antenna-bundle.jpg" alt="FatShark HDO goggles with rapidFIRE module and Lumenier antennas"></a>
    <figcaption>FatShark HDO goggles with the rapidFIRE module and the Lumenier AXII 2 antennas.</figcaption>
</figure>

The rapidFIRE module is a diversity receiver with two receivers and two antennas.
It receives 48 channels in 6 bands including Raceband, and it can run in its own rapidFIRE mode, which combines the images of both receivers,
in classic diversity mode, which switches to the better receiver, or with a single receiver to save battery
([Oscar Liang's review](https://oscarliang.com/immersionrc-rapidfire-module/)).
