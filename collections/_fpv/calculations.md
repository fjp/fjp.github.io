---
layout: single
title:  "Thrust and Flight Time"
permalink: /projects/fpv/calculations/
excerpt: "Calculations to find matching parts and to estimate the flight time."
categories: [fpv, quad]
tags: [fpv, quad, race, drone, thrust, weight, flight time, calculation]
date: 2026-10-06 09:00:00 +0200
comments: true
use_math: true
toc: true
classes: wide
sidebar:
  nav: "fpv"
---

## Introduction

Before buying parts it helps to check that they fit together: the motors have to lift the quad with enough reserve,
the ESCs and the battery have to deliver the current the motors draw, and the battery capacity determines how long the quad flies.
This page does these calculations for the parts of this build, using the thrust data that EMAX publishes for the RS2205 2300 $K_v$ motor.

## Basic Calculations {#calculations}

The electrical power $P$ is the product of voltage $U$ and current $I$, and the energy $E$ stored in a battery is its voltage times its capacity $Q$:

$$
P = U \cdot I \qquad E = U \cdot Q
$$

A 3S battery with 11.1 V and 1300 mAh stores $11.1\,\text{V} \cdot 1.3\,\text{Ah} \approx 14.4\,\text{Wh}$, see [LiPo Batteries](/projects/fpv/battery).

## Weight

The all-up weight (AUW) is the weight of the ready-to-fly quad including the battery.
The following table adds up the parts of this build with the weights given by manufacturers and reviews.
The values marked as estimate are typical values, weighing the finished quad on a kitchen scale replaces them.

| Part | Weight | Source |
|:--|--:|:--|
| iFlight iX5 frame with hardware | 86 g | [IntoFPV](https://intofpv.com/archive/index.php/thread-7268.html) |
| 4 × EMAX RS2205 2300 $K_v$ | 4 × 30 g = 120 g | [getfpv](https://www.getfpv.com/discontinued/emax-rs2205-2300kv-racespec-motor-cw.html) |
| 4 × DYS Aria 35A ESC with wires | 4 × 8.1 g = 32.4 g | [Oscar Liang](https://oscarliang.com/dys-aria-35a-esc) |
| Matek F722-STD flight controller | 7 g | [racer.lt](https://www.racer.lt/datasheets/about/matek-f722-std-532.html) |
| Matek FCHUB-6S PDB | 8.5 g | [Matek manual](https://www.mateksys.com/downloads/FCHUB-6S_manual.pdf) |
| RunCam Swift 2 camera | 14 g | [getfpv](https://www.getfpv.com/discontinued/runcam-swift-2-2-5mm-lens-orange.html) |
| TBS Unify Pro HV video transmitter | 7 g | [getfpv](https://www.getfpv.com/fpv/video-transmitters/tbs-unify-pro-5g8-hv-sma.html) |
| FrSky R-XSR receiver | 1.5 g | [FrSky manual](https://www.frsky-rc.com/wp-content/uploads/Downloads/Manual/R-XSR/R-XSR%20ACCST%20-Manual.pdf) |
| Propellers, VTX antenna, cables, XT60, battery strap | about 40 g | estimate |
| **Quad without battery** | **about 316 g** | |
| 3S 1300 mAh LiPo | about 120 g | estimate |
| **All-up weight** | **about 440 g** | |

## Thrust

According to EMAX's [thrust table](/projects/fpv/motor#relation-between-thrust-and-weight), one RS2205 2300 $K_v$ motor with a
5x4.5 bullnose propeller creates up to 712 g of thrust at 12 V, which is close to the voltage of a 3S battery.
Four motors therefore create up to $4 \cdot 712\,\text{g} = 2848\,\text{g}$, and the thrust-to-weight ratio of this build is

$$
\text{TWR} = \frac{4 \cdot T_{\max}}{m_{\text{AUW}}} = \frac{2848\,\text{g}}{440\,\text{g}} \approx 6.5
$$

A quad needs a ratio of at least 2 to fly controllably. Race and freestyle quads are built with much higher ratios,
because the reserve makes fast accelerations and climbs possible.

To hover, each motor has to carry a quarter of the weight, which is about 110 g.
The thrust table gives about 2 A per motor for this, so the quad draws about 8 A while hovering, a little more than 10 % of the current at full throttle.

## Finding Matching Motors {#motors}

The motor $K_v$, the cell count of the battery and the propeller have to fit together.
Without load the RS2205 would turn at $2300 \cdot 12 = 27\,600$ RPM at 12 V. With the 5x4.5 propeller the thrust table shows 20&nbsp;080 RPM at full throttle,
which is 73 % of the speed without load. This gives a pitch speed of $0.1143\,\text{m} \cdot 20\,080 / 60\,\text{s} \approx 38\,\text{m/s}$ or about 138 km/h,
compared to the upper bound of 175 km/h on the [propeller page](/projects/fpv/propeller#pitch).
A larger propeller or a higher pitch would load the motor more and draw more current, a battery with more cells would need a motor with a lower $K_v$.

## Finding Matching ESCs {#esc}

At full throttle one motor draws 20.7 A at 12 V according to the thrust table, and 29.9 A at 16 V on a 4S battery.
The DYS Aria ESCs are rated for 35 A, so they have enough reserve on 3S and still on 4S,
see [ampere and load capacity](/projects/fpv/esc#ampere-a-and-load-capacity).
The [PDB](/projects/fpv/pdb#the-matek-fchub-6s-of-this-build) supplies 30 A per ESC continuously, which is also enough on 3S.

All four motors together draw $4 \cdot 20.7\,\text{A} = 82.8\,\text{A}$ at full throttle, that is about 1 kW.
From a 1300 mAh battery this requires a C-rating of $82.8 / 1.3 \approx 64$.
The Lumenier 60C battery of the parts list delivers 78 A continuously, and because a quad flies at full throttle only for short moments,
the batteries of this build can deliver these peaks.

## Flight Time Calculation {#flight-time}

Only about 80 % of the capacity should be used, so that the cells don't drop below about 3.5 V under load.
With the usable capacity $0.8 \cdot Q$ and the average current $I_{\text{avg}}$ the flight time is

$$
t = \frac{0.8 \cdot Q}{I_{\text{avg}}}
$$

| Flight | Average current | Flight time with 1300 mAh |
|:--|--:|--:|
| Hovering | about 8 A | about 8 minutes |
| Calm flying | 20 A | about 3.1 minutes |
| Racing | 25 A | about 2.5 minutes |
| Aggressive racing | 30 A | about 2.1 minutes |

The average currents for flying are assumptions. The [current sensor](/projects/fpv/pdb#current-sensor) of the PDB
measures the actual consumption, which Betaflight shows in the OSD as used capacity in mAh.
