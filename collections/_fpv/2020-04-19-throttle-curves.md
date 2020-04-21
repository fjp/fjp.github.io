---
layout: single
title:  "Setting up Throttle Curves"
permalink: /projects/fpv/throttle-curves
excerpt: "How to setup different throttle curves on the Taranis X9D Plus SE 2019."
date: 2020-04-18 09:00:35 +0100
categories: [fpv, quad]
tags: [fpv, quad, race, drone, betaflight, radio, transmitter, throttle, curve, taranis]
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


To add new and change existing throttle curves in your Taranis X9D Plus short press the `MENU` button an navigate
to page 7 with the `PAGE` button. This will show the following dispaly:

<figure>
    <a href="/assets/collections/fpv/throttle-curve/01-no-curves.jpg"><img src="/assets/collections/fpv/throttle-curve/01-no-curves.jpg"></a>
    <figcaption>Initial display view without curves</figcaption>
</figure>

On this page select the first free curve which will lead to the following screen:

<figure>
    <a href="/assets/collections/fpv/throttle-curve/02-initial-curve-settings.jpg"><img src="/assets/collections/fpv/throttle-curve/02-initial-curve-settings.jpg"></a>
    <figcaption>Display view of initial curve</figcaption>
</figure>

Because we want to create a throttle curve we select the Name field and enter "Thr" (Long press `ENTER` button for capital letters). Instead of the default 5 control points we select 7 to get finer control and select Smooth. 

<figure>
    <a href="/assets/collections/fpv/throttle-curve/03-thr-settings.jpg"><img src="/assets/collections/fpv/throttle-curve/03-thr-settings.jpg"></a>
    <figcaption>Settings for throttle curve</figcaption>
</figure>

The slope of this curve is low in the region around x equals zero, which is where the hover point should be located. 
The hover point is set below y equals zero to reduce the overall output power which is also why the last point is set
to 60. This curve should help beginners to find the hover point more easily.


After setting the throttle curve it is time to apply it to the throttle input on the Input page. For this use `PAGE` to page through to page 4 or `EXIT` to the main screen and short press `MENU`, then `PAGE` until you get to page 4.

<figure>
    <a href="/assets/collections/fpv/throttle-curve/04-inputs.jpg"><img src="/assets/collections/fpv/throttle-curve/04-inputs.jpg"></a>
    <figcaption>Input Page display with throttle selected.</figcaption>
</figure>

Long press `ENTER` on the Thr entry and select Edit.  

<figure>
    <a href="/assets/collections/fpv/throttle-curve/05-edit-throttle-input.jpg"><img src="/assets/collections/fpv/throttle-curve/05-edit-throttle-input.jpg"></a>
    <figcaption>Edit throttle input.</figcaption>
</figure>

This leads to the settings display for the throttle input which has a linear curve set by default:


<figure>
    <a href="/assets/collections/fpv/throttle-curve/06-initial-thr-input.jpg"><img src="/assets/collections/fpv/throttle-curve/06-initial-thr-input.jpg"></a>
    <figcaption>Initial throttle input display with linear curve.</figcaption>
</figure>

Select `Cstm` for the Curve entry and choose the previously named curve `Thr`. 

<figure>
    <a href="/assets/collections/fpv/throttle-curve/07-select-throttle-curve.jpg"><img src="/assets/collections/fpv/throttle-curve/07-select-throttle-curve.jpg"></a>
    <figcaption>Select throttle curve.</figcaption>
</figure>

Instead of using a custom throttle curve it is also possible to set `Expo` or `Func`:

<figure>
    <a href="/assets/collections/fpv/throttle-curve/08-expo-throttle-setting.jpg"><img src="/assets/collections/fpv/throttle-curve/08-expo-throttle-setting.jpg"></a>
    <figcaption>Expo throttle setting.</figcaption>
</figure>

To verify the setting on your Taranis you can `PAGE` to the channel monitor screen starting from the main menu, 
to see the individual channels.

<figure>
    <a href="/assets/collections/fpv/09-throttle-curve-channel-monitor.jpg"><img src="/assets/collections/fpv/09-throttle-curve-channel-monitor.jpg"></a>
    <figcaption>Verify throttle curve in channel monitor screen.</figcaption>
</figure>

## References

- [Oscar Liang](https://oscarliang.com/throttle-curve/)
