---
layout: single
title:  "The Propeller"
permalink: /projects/fpv/propeller
excerpt: "The Propeller transforms the energy from the battery into velocity."
date:   2020-05-13 09:00:35 +0100
last_modified_at: 2026-10-06
categories: [fpv, quad]
tags: [fpv, quad, race, drone, propeller, props]
comments: true
use_math: true
toc: true
classes: wide
# toc_label: "Unscented Kalman Filter"
header:
  teaser: /assets/collections/fpv/propeller/propeller-header.jpg
  overlay_image: /assets/collections/fpv/propeller/propeller-header.jpg
  overlay_filter: 0.5
  caption: "Master Airscrew BN 5x4.5 propeller of this build"
sidebar:
  nav: "fpv"
---

A race quad-copter requires propellers to translate the stored energy of the battery into kinematic energy.
The structure of propellers are comparable to the wings of an airplane, which provides uplift when moving forward
and lifts the plane off the ground.

The propeller causes a similar effect with the difference that not the whole plane needs to move forward.
Instead, the uplift is created through the rotation of the propeller similar to a helicopter.

## Size and Pitch

Propellers are specified by their diameter and pitch, both in inches, and often by the number of blades.
A 5x4.5 propeller, also written as 5045, has a diameter of 5 inches and a pitch of 4.5 inches.
A third number, as in 5x4.5x3, gives the number of blades.
This build uses two-blade Master Airscrew BN (bullnose) 5x4.5 propellers made of glass fiber reinforced polyamide, see the [components](/projects/fpv/components#props).
A set contains two propellers for each direction of rotation.

<figure class="half">
    <a href="/assets/collections/fpv/propeller/propellers-5x4.5.jpg"><img src="/assets/collections/fpv/propeller/propellers-5x4.5.jpg"></a>
    <a href="/assets/collections/fpv/propeller/propeller-5x4.5.jpg"><img src="/assets/collections/fpv/propeller/propeller-5x4.5.jpg"></a>
    <figcaption>Set of four Master Airscrew BN 5x4.5 two-blade propellers and a single propeller.</figcaption>
</figure>

### Diameter

The diameter is the size of the circle that the blade tips describe.
It is limited by the frame, because the propellers must neither touch each other nor the frame.
This is why quads are named after the propeller size they carry, for example a 5 inch quad.

A larger diameter moves more air per revolution and creates more thrust.
It also has more inertia, so the [motor](/projects/fpv/glossar#motor) needs more torque to speed it up and slow it down, and the quad reacts slower.
Larger propellers therefore need motors with more torque, which usually means a larger stator.

### Pitch

The pitch is the distance a propeller would move forward in one revolution if it were screwed through a solid material.
A higher pitch moves more air per revolution and allows higher speeds, but it needs more torque and draws more current.
A lower pitch reacts quicker and draws less current.

Pitch and [RPM](/projects/fpv/glossar#rpm) $n$ give the theoretical speed of the air pushed by the propeller, the pitch speed:

$$
v_p = p \cdot \frac{n}{60}
$$

The EMAX RS2205 2300 $K_v$ [motor](/projects/fpv/motor) of this build turns at about $2300 \cdot 11.1 \approx 25\,530$ RPM without load
on a 3S [LiPo](/projects/fpv/glossar#lipo) battery with a nominal voltage of 11.1 V. With a pitch of 4.5 inches, which is 0.1143 m:

$$
v_p = 0.1143\,\text{m} \cdot \frac{25\,530}{60\,\text{s}} \approx 48.6\,\frac{\text{m}}{\text{s}} \approx 175\,\frac{\text{km}}{\text{h}}
$$

This is an upper bound. Under load the motor turns slower, the battery voltage sags,
and a propeller in air slips instead of moving forward by its full pitch in each revolution.

### Blade Count

Propellers with more blades have more blade area. At the same diameter they create more thrust and more grip in the air,
which helps when the frame limits the diameter. On the other hand they create more drag, draw more current and are less efficient.
Three-blade propellers are the most common choice on 5 inch quads today, while this build uses two-blade propellers.

## Direction of Rotation

Propellers are made for clockwise (CW) and counterclockwise (CCW) rotation.
On a quad, two diagonally opposite motors spin clockwise and the other two counterclockwise, so that their torques cancel out and the quad doesn't yaw.
Each propeller has to match the direction of its motor: the leading edge, which is the thicker and rounded edge of a blade, points in the direction of rotation.
A propeller on a motor with the wrong direction pushes the air upwards, and the quad flips over right after takeoff.

## Plastic, Fiberglass or Carbon

Propellers can consist of different materials. Small and in general inexpensive propellers are made of conventional plastic (e.g. EPP propellers).
Better quality is achieved through the addition of carbon fiber or fiberglass. Propeller materials mixed of carbon- and fiberglass exist too.
The best quality comes with a propeller consisting of pure carbon fiber because they are very light and highly efficient. Because of its
expensive material and manufacturing process it is more expensive compared to a traditional plastic propeller. 

## References

- [MikroKopter Wiki: Propeller](https://wiki.mikrokopter.de/en/Propeller)
- [Oscar Liang: How to Choose the Best Propellers for FPV Drones](https://oscarliang.com/propellers/)
- [Wikipedia: Propeller (aeronautics)](https://en.wikipedia.org/wiki/Propeller_(aeronautics))

