---
layout: single
title:  "Taranis X9D Plus SE 2019 - OpenTX and EU-LBT/FCC Firmware"
permalink: /projects/fpv/taranis
excerpt: "All about the bootloader, OpenTX and firmware of the Taranis X9D Plus SE 2019."
date: 2019-10-30 09:00:35 +0100
categories: [fpv, quad]
tags: [fpv, quad, race, drone, frame]
comments: true
use_math: true
toc: true
classes: wide
# toc_label: "Unscented Kalman Filter"
header:
#  teaser: /assets/projects/autonomous-rc-car/hpi-racing-bmw-m3_thumb.png
#  overlay_image: /assets/projects/autonomous-rc-car/hpi-racing-bmw-m3.png
redirect_from:
  - /projects/fpv/
sidebar:
  nav: "fpv"
---

This post covers how to flash the latest version of [OpenTX](/projects/fpv/glossar/#opentx) (currently 2.3.1), which
is a hightly configurable open source firmware for [radios](/projects/fpv/glossar/#radio) that acts as the operating
system on [Taranis](/projects/fpv/glossar/#taranis) [transmitters](/projects/fpv/glossar/#transmitter).
The post shows to to [flash](/projects/fpv/glossar/#flash) the [firmware](/projects/fpv/glossar/#firmware) of the internal [transmitter](/projects/fpv/glossar/#transmitter) module of the Taranis into
the [EU-LBT](/projects/fpv/glossar/#eu-lbt) mode and how to [register](/projects/fpv/glossar/#register) and [bind](/projects/fpv/glossar/#bind) an [ACCESS](/projects/fpv/glossar/#access) [receiver](/projects/fpv/glossar/#receiver).



<figure>
    <a href="/assets/collections/fpv/taranis.jpg"><img src="/assets/collections/fpv/taranis.jpg"></a>
    <figcaption>Taranis X9D Plus SE 2019 Carbon.</figcaption>
</figure>

## OpenTX

To begin with the update of the OpenTX firmware on the Taranis, download the latest version of the 
[OpenTX Companion](/projects/fpv/glossar/#opentx-companion) software (available for Windows, Mac and Linux) from [open-tx.org](https://www.open-tx.org/2019/10/05/opentx-2.3.1). On this webpage you also find the [sdcard content](https://downloads.open-tx.org/2.3/release/sdcard/) which we will need to put on a seperate sd card that will be inserted in
the Taranis radio.

Note that the Taranis X9D Plus SE 2019 doesn't ship with an sd card. However, it has an internal storage for the bootloader
and the firmware (OpenTX) and the firmware of the internal transmitter module.
{: .notice}

After downloading OpenTX Companion and installing on your operating system, execute the application and open the settings,
shown in the following screenshots. Here you can see the settings in the `Radio Profile` tab. 
As `Radio Type` choose the `FrSky Taranis X9D+ 2019` and the following `Build Options`:

- `lua` enables lua custom script screen
- `noheli` which disables the Heli menu page on the Taranis in the case you are not flying a helicopter.
- `flexr9m` or `eu` which can be both used to flash the internal transmitter module from the FCC to the EU-LBT firmware.

Use `flexr9m` if you have an external transmitter module such as the FrSky R9M and want to flash its firmware.
In case you ordered from a vendor that shiped the Taranis X9D with the FCC firmware on the internal transmitter module and you don't have an external module choose either `flexr9m` or `eu`. `flexr9m` lets you also flash the EU-LBT firmware. 
Note that you cannot check both, so you have to choose between `flexr9m` or `eu`.
{: .note}

<figure class="half">
    <a href="/assets/collections/fpv/opentx.jpg"><img src="/assets/collections/fpv/opentx.jpg"></a>
    <a href="/assets/collections/fpv/opentx/opentx-companion-settings.png"><img src="/assets/collections/fpv/opentx/opentx-companion-settings.png"></a>
    <figcaption>OpenTX Companion.</figcaption>
</figure>

Also choose an `SD Structure path` on your Mac or PC that will mirror the internal storage of the Taranis to your local machine. 
This is the path where we will place the previously downloaded sd card content that will be uploaded to the Taranis radio.

If you want you can change the `Default Stick Mode` or the `Default Channel Order`. For convenience, select the two options:

- `Append version number to FW file name`
- `Offer to write FW to Tx after download`



