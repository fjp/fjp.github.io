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
    <a href="/assets/collections/fpv/throttle-curve/no-curves.jpg"><img src="/assets/collections/fpv/throttle-curve/no-curves.jpg"></a>
    <figcaption>Initial display view without curves</figcaption>
</figure>

Because we want to create a throttle curve we select the Name field and enter "Thr" (Long press `ENTER` button for capital letters). Instead of the default 5 control points we select 7 to get finer control and select Smooth. 

<figure>
    <a href="/assets/collections/fpv/throttle-curve/Thr-settings.jpg"><img src="/assets/collections/fpv/throttle-curve/Thr-settings.jpg"></a>
    <figcaption>Initial display view without curves</figcaption>
</figure>


Apply curve to the throttle input on the Input page. For this use `PAGE` to page through to page 4 or `EXIT` to the main screen and short press `MENU`, then `PAGE` until you get to page 4.

<figure>
    <a href="/assets/collections/fpv/throttle-curve/inputs.jpg"><img src="/assets/collections/fpv/throttle-curve/inputs.jpg"></a>
    <figcaption>Initial display view without curves</figcaption>
</figure>

Long press `ENTER` on the Thr entry and select Edit.  

<figure>
    <a href="/assets/collections/fpv/throttle-curve/edit-throttle-input.jpg"><img src="/assets/collections/fpv/throttle-curve/edit-throttle-input.jpg"></a>
    <figcaption>Edit throttle input.</figcaption>
</figure>

Select Cstm for the Curve entry and choose the previously named curve `Thr`. 

<figure>
    <a href="/assets/collections/fpv/throttle-curve/select-throttle-curve.jpg"><img src="/assets/collections/fpv/throttle-curve/select-throttle-curve.jpg"></a>
    <figcaption>Select throttle curve.</figcaption>
</figure>

Instead of using a custom throttle curve it is also possible to set `Expo` or `Func`:

<figure>
    <a href="/assets/collections/fpv/throttle-curve/expo-throttle-setting.jpg"><img src="/assets/collections/fpv/throttle-curve/expo-throttle-setting.jpg"></a>
    <figcaption>Expo throttle setting.</figcaption>
</figure>

To verify the setting on your Taranis you can `PAGE` to the channel monitor screen starting from the main menu, 
to see the individual channels.

<figure>
    <a href="/assets/collections/fpv/throttle-curve/channel-monitor.jpg"><img src="/assets/collections/fpv/throttle-curve/channel-monitor.jpg"></a>
    <figcaption>Verify throttle curve in channel monitor screen.</figcaption>
</figure>

## References

- [Oscar Liang](https://oscarliang.com/throttle-curve/)
