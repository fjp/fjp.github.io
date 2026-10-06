---
layout: single
title:  "Antennas"
permalink: /projects/fpv/fpv-system/antennas
excerpt: "Antennas for the video link of a FPV quad."
categories: [fpv, quad]
tags: [fpv, quad, race, drone, antennas]
date: 2026-10-06 09:00:00 +0200
comments: true
use_math: true
toc: true
classes: wide
header:
  teaser: /assets/collections/fpv/components/lumenier-axii-2.jpg
sidebar:
  nav: "fpv"
---

The antennas decide how far and how reliably the video signal reaches the goggles.
An FPV antenna has to work in all orientations of the quad, and it must cope with signals that reflect from buildings, trees and the ground.

## Polarization

The electric field of a radio wave oscillates in a certain direction, its polarization.
Linear antennas, such as the simple dipole, send and receive in one plane, so the signal weakens when the quad banks or rolls.
FPV antennas are therefore circularly polarized: the field rotates, either right-handed (RHCP) or left-handed (LHCP).
A reflection reverses the direction of rotation, so the receiving antenna rejects much of the reflected signal, which reduces interference.

The antennas on the quad and on the goggles must use the same polarization.
An RHCP antenna receives an LHCP signal only very weakly, which is also used to separate pilots who fly on neighboring channels.

## Omnidirectional and Directional Antennas

Omnidirectional antennas, such as the mushroom-shaped Lumenier AXII 2, radiate in all directions around their axis.
They are used on the quad and on the goggles. Directional antennas, such as patch antennas, concentrate the signal in one direction.
Their higher gain reaches further, but only within a narrower angle, so they are used on the goggles and point in the direction where the quad flies.

A diversity receiver combines both: one receiver uses an omnidirectional antenna for flights close by and all around,
the other one a patch antenna for flights further away.

The gain of circularly polarized antennas is given in dBic, decibels compared to an ideal isotropic antenna with circular polarization.
On the transmitter side the gain adds to the power of the VTX, see [output power](/projects/fpv/fpv-system/vtx#bands-channels-and-output-power).

## Connectors

- **SMA**: the common connector for 5.8 GHz antennas. The antenna has a thread nut with a center pin, the VTX has an outer thread with a socket.
- **RP-SMA** (reverse polarity SMA): looks the same, but pin and socket are swapped. SMA and RP-SMA screw together but don't make contact at the center.
- **U.FL** and **MMCX**: small snap-on connectors for light quads, where an SMA connector would be too heavy.

## Antennas Used in this Build

The FPV bundle of this build includes the Lumenier AXII 2 diversity antenna set: two omnidirectional AXII 2 antennas with SMA connectors,
which have a gain of 2.2 dBic and right-hand circular polarization ([getfpv](https://www.getfpv.com/fpv/antennas/lumenier-axii-2/lumenier-axii-2-5-8ghz-antenna-rhcp-2-pcs.html)),
and an AXII patch antenna, also RHCP.

<figure class="half">
    <a href="/assets/collections/fpv/components/lumenier-axii-2.jpg"><img src="/assets/collections/fpv/components/lumenier-axii-2.jpg" alt="Lumenier AXII 2 omnidirectional antennas"></a>
    <a href="/assets/collections/fpv/components/lumenier-axii-patch.jpg"><img src="/assets/collections/fpv/components/lumenier-axii-patch.jpg" alt="Lumenier AXII patch antenna"></a>
    <figcaption>Lumenier AXII 2 omnidirectional antennas and AXII patch antenna, all RHCP.</figcaption>
</figure>
