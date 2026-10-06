---
layout: single
title:  "FPV Camera"
permalink: /projects/fpv/fpv-system/camera
excerpt: "The FPV camera captures the live image for the pilot."
categories: [fpv, quad]
tags: [fpv, quad, race, drone, camera]
date: 2026-10-06 09:00:00 +0200
comments: true
use_math: true
toc: true
classes: wide
header:
  teaser: /assets/collections/fpv/cam/runcam-swift-2-top.jpg
sidebar:
  nav: "fpv"
---

The [FPV camera](/projects/fpv/glossar#fpv-camera) captures the live image that the pilot sees in the goggles.
For flying, low latency and a fast reaction to changing light matter more than resolution:
when the quad flies from bright sunlight into the shade of a tree, the image must stay readable.

## Image Sensor

Analog FPV cameras use CCD or CMOS image sensors.
CCD sensors such as the 1/3" Sony Super HAD II CCD of the RunCam Swift 2 were popular for their low latency and good handling of bright and dark areas in the same image.
CMOS sensors have caught up since, and most current analog cameras use them.
Wide dynamic range (WDR) processing brightens dark areas and darkens bright ones, so that both stay visible.

## Lens and Field of View

The focal length of the lens determines the field of view (FOV): the shorter the focal length, the wider the view.
A wide field of view shows more of the surroundings, but distorts the image like a fisheye and makes obstacles appear further away.
The Swift 2 is available with three lenses:

| Lens | Field of view |
|:--|--:|
| 2.5 mm (this build) | 130° |
| 2.3 mm | 150° |
| 2.1 mm | 165° |

## Video Format

Analog cameras output either PAL or NTSC. PAL has 625 lines and 50 fields per second, NTSC has 525 lines and 60 fields per second.
Camera, VTX and goggles must handle the same format, and the aspect ratio of the camera should match the displays of the goggles, which is 4:3 on the FatShark HDO.

## Camera Settings and Mounting

Most cameras have a menu for settings such as brightness, WDR and the video format, which is opened with buttons or a small joystick on the camera or its cable.
The Swift 2 also has an integrated OSD that can show the battery voltage, and a microphone that transmits the sound of the motors.

A race quad tilts forward when it flies fast, so the camera is mounted with an uptilt angle.
With a higher uptilt the horizon stays in the center of the image at higher speeds, but the camera looks at the sky when the quad hovers.

## Camera Used in this Build

This build uses the RunCam Swift 2 with the 2.5 mm lens, see the [assembly](/projects/fpv/assembly) for how it is mounted in the frame.
Its specifications according to RunCam ([archived product page](https://web.archive.org/web/20190919060634/https://shop.runcam.com/runcam-swift-2/)):

| Property | Value |
|:--|:--|
| Image sensor | 1/3" Sony Super HAD II CCD |
| Horizontal resolution | 600 TVL |
| Video format | PAL or NTSC |
| Minimum illumination | 0.01 lux at F1.2 |
| Features | Integrated OSD and microphone, D-WDR |
| Power | 5 to 36 V, 130 mA at 5 V or 70 mA at 12 V |
| Weight and dimensions | 14 g, 28.5 × 26 × 26 mm |

<figure class="half">
    <a href="/assets/collections/fpv/cam/runcam-swift-2-top.jpg"><img src="/assets/collections/fpv/cam/runcam-swift-2-top.jpg" alt="RunCam Swift 2 from the front"></a>
    <a href="/assets/collections/fpv/cam/runcam-swift-2-back.jpg"><img src="/assets/collections/fpv/cam/runcam-swift-2-back.jpg" alt="RunCam Swift 2 from the back"></a>
    <figcaption>RunCam Swift 2 from the front and the back.</figcaption>
</figure>
