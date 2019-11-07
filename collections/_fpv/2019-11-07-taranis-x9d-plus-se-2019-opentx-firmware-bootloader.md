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

With the OpenTX Companion software it is possible to backup your radio using the following button, which lets you store
a `*.bin` file on your computer:

<figure>
    <a href="/assets/collections/fpv/opentx/backup.jpg"><img src="/assets/collections/fpv/opentx/backup/taranis.jpg"></a>
    <figcaption>Backup the Settings and Models on your Taranis before you proceed.</figcaption>
</figure>

It is advised that you backup your radio settings and models before you proceed with the next steps.
This way you can restore your backup using the `Write Backup to Radio` button, right above the `Backup Radio to File` button. 

### OpenTX Settings

After downloading OpenTX Companion and installing on your operating system, execute the application and open the settings,
shown in the following screenshots. 

<figure class="half">
    <a href="/assets/collections/fpv/opentx/opentx.jpg"><img src="/assets/collections/fpv/opentx/opentx.jpg"></a>
    <a href="/assets/collections/fpv/opentx/opentx-companion-settings.png"><img src="/assets/collections/fpv/opentx/opentx-companion-settings.png"></a>
    <figcaption>OpenTX Companion.</figcaption>
</figure>

Here you can see the settings in the `Radio Profile` tab. 
As `Radio Type` choose the `FrSky Taranis X9D+ 2019` and the following `Build Options`:

- `lua` enables lua custom script screen
- `noheli` which disables the Heli menu page on the Taranis in the case you are not flying a helicopter.
- `flexr9m` or `eu` which can be both used to flash the internal transmitter module from the FCC to the EU-LBT firmware.

Use `flexr9m` if you have an external transmitter module such as the FrSky R9M and want to flash its firmware.
In case you ordered from a vendor that shiped the Taranis X9D with the FCC firmware on the internal transmitter module and you don't have an external module choose either `flexr9m` or `eu`. `flexr9m` lets you also flash the EU-LBT firmware. 
Note that you cannot check both, so you have to choose between `flexr9m` or `eu`.
{: .note}



Also choose an `SD Structure path` on your Mac or PC that will mirror the internal storage of the Taranis to your local machine. 
This is the path where we will place the previously downloaded sd card content that will be uploaded to the Taranis radio.

If you want you can change the `Default Stick Mode` or the `Default Channel Order`. For convenience, select the two options:

- `Append version number to FW file name`
- `Offer to write FW to Tx after download`

Save the radio profile using a `Profile Name` and press `Ok`.

### Taranis X9D Bootloader

Now that we are done with the settings of the firmware that will be flashed on the radio we need to enter the bootloader.
To do that, press both horizontal trim switches to the middle while powering the transmitter. 

Note: Don't press the power button for to long in order to enter the bootloader and not booting into the currently installed firmware.
{: .note}

<figure class="half">
    <a href="/assets/collections/fpv/taranis/enter-bootloader.jpg"><img src="/assets/collections/fpv/taranis/enter-bootloader.jpg"></a>
    <a href="/assets/collections/fpv/taranis/bootloader.jpg"><img src="/assets/collections/fpv/taranis/bootloader.jpg"></a>
    <figcaption>OpenTX Companion.</figcaption>
</figure>

### Flash OpenTX

Once in the bootloader menu, connect the Taranis with the computer via the provided USB cable, which will change the 
screen to the following mask:

<figure>
    <a href="/assets/collections/fpv/taranis/bootloader-usb.jpg"><img src="/assets/collections/fpv/taranis/bootloader-usb.jpg"></a>
    <figcaption>Bootloader with connected USB cable.</figcaption>
</figure>

Back in the OpenTX Companion application click the `Download` button, then do the following steps in the new Dialog that pops up:

![image-right]({{ site.url }}{{ site.baseurl }}/assets/collections/fpv/opentx/01-firmware-download.png){: .align-right}

- `Check for Updates`
- `Download firmware` which downloads the OpenTX firmware (this will be a `*.bin` file) with the settings you selected previously.
- Write the downloaded and stored firmware to the Taranis

<figure class="third">
    <a href="/assets/collections/fpv/opentx/02-firmware-download.png"><img src="/assets/collections/fpv/opentx/02-firmware-download.png"></a>
    <a href="/assets/collections/fpv/opentx/03-write-firmware.png"><img src="/assets/collections/fpv/opentx/03-write-firmware.png"></a>
    <a href="/assets/collections/fpv/opentx/04-final-write-settings.png"><img src="/assets/collections/fpv/opentx/04-final-write-settings.png"></a>
    <figcaption>Flashing procedure of OpenTX.</figcaption>
</figure>

After the flashing procedure, your Taranis is updated to the latest version of OpenTX (currently 2.3.1).

<figure class="half">
    <a href="/assets/collections/fpv/opentx/05-flashing-done.png"><img src="/assets/collections/fpv/opentx/05-flashing-done.png"></a>
  <a href="/assets/collections/fpv/taranis/updated-firmware.jpg"><img src="/assets/collections/fpv/taranis/updated-firmware.jpg"></a>
    <figcaption>Flashing procedure of OpenTX.</figcaption>
</figure>

Exit the bootloader, which will restart your Taranis. Power it off for the next step.

### SD Card Content

Now write the previously downloaded [sd card content](https://downloads.open-tx.org/2.3/release/sdcard/) from the OpenTX website on a seperate sd card that you plug in the back tray of your Taranis. 

<figure>
    <a href="/assets/collections/fpv/taranis/sdcard.jpg"><img src="/assets/collections/fpv/taranis/sdcard.jpg"></a>
    <figcaption>Insert an empty sd card in your Taranis.</figcaption>
</figure>

When you inserted the sd card, power on the Taranis and connect it with the USB cable to your computer. 
A message will pop up on your Taranis, where you should select `USB Storage (SD)`, as shown in the next figure. 

<figure>
    <a href="/assets/collections/fpv/taranis/usb-storage.jpg"><img src="/assets/collections/fpv/taranis/usb-storage.jpg"></a>
    <figcaption>Select `USB Storage (SD)` after you plugged in your USB cable.</figcaption>
</figure>

Next, place the content of the downloaded sd card zip file on the empty sd card and open the `SCRIPTS/TOOLS` folder 
where you copy the lua script for changing the internal module into the FCC or EU-LBT mode.

<figure class="half">
    <a href="/assets/collections/fpv/taranis/copy-sd-content.png"><img src="/assets/collections/fpv/taranis/copy-sd-content.png"></a>
    <a href="/assets/collections/fpv/taranis/copy-lua-script.png"><img src="/assets/collections/fpv/taranis/copy-lua-script.png"></a>
    <figcaption>Copy the sd card content and the lua script on the external Taranis sd card.</figcaption>
</figure>

The mentioned lua script to switch the internal module into the FCC or EU-LBT mode can be obtained from the FrSky support.
Because it is not allowed to share this script, just write a mail to the [FrSky support](mailto:frsky@frsky-rc.com) asking for the lua script. It is named `isrm_mode_1.lua`. Using this script you must choose the correct firmware that is allowed in your area.
{: .warning}

### Switching Internal Module to EU-LBT Mode

After preparing the external sd card of the Taranis unplug the USB cable (after ejecting both storage locations `TARANIS` and `EXTTARANIS` from your computer). Then power on your Taranis and enter the `RADIO SETUP` menu by long pressing the 
`MENU` button of your Taranis. Press `PAGE` repeatedly until you get to the `TOOL` menu, where you select `Change ISRM mode`
to change the internal transmitter module into the EU-LBT mode using the lua script.


<figure class="half">
    <a href="/assets/collections/fpv/taranis/change-isrm-mode.jpg"><img src="/assets/collections/fpv/taranis/change-isrm-mode.jpg"></a>
    <a href="/assets/collections/fpv/taranis/eu-lbt-mode.jpg"><img src="/assets/collections/fpv/taranis/eu-lbt-mode.jpg"></a>
    <figcaption>Change the internal transmitter module to EU-LBT mode inside the TOOS menu.</figcaption>
</figure>








