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
  teaser: /assets/collections/fpv/taranis/taranis.jpg
  overlay_image: /assets/collections/fpv/taranis/bootloader.jpg
  overlay_filter: 0.5
redirect_from:
  - /projects/fpv/
sidebar:
  nav: "fpv"
---

This post covers how to flash the latest version of [OpenTX](/projects/fpv/glossar/#opentx) (currently 2.3.1), which
is a highly configurable open source firmware for [radios](/projects/fpv/glossar/#radio) that acts as the operating
system on [Taranis](/projects/fpv/glossar#taranis) [transmitters](/projects/fpv/glossar#transmitter).
The post shows to to [flash](/projects/fpv/glossar#flash) the [firmware](/projects/fpv/glossar#firmware) of the internal [transmitter](/projects/fpv/glossar#transmitter) module of the Taranis into
the [EU-LBT](/projects/fpv/glossar#eu-lbt) mode and how to [register](/projects/fpv/glossar/#register) and [bind](/projects/fpv/glossar#bind) an [ACCESS](/projects/fpv/glossar#access) [receiver](/projects/fpv/glossar#receiver).



<figure>
    <a href="/assets/collections/fpv/taranis/taranis.jpg"><img src="/assets/collections/fpv/taranis/taranis.jpg"></a>
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
    <a href="/assets/collections/fpv/opentx/radio-backup.jpg"><img src="/assets/collections/fpv/opentx/radio-backup.png"></a>
    <figcaption>Backup the Settings and Models on your Taranis before you proceed.</figcaption>
</figure>

It is advised that you backup your radio settings and models before you proceed with the next steps.
This way you can restore your backup using the `Write Backup to Radio` button, right above the `Backup Radio to File` button. 

## OpenTX Settings

After downloading OpenTX Companion and installing it on your operating system, 
execute the application and open the settings, shown in the following screenshots. 

<figure class="half">
    <a href="/assets/collections/fpv/opentx/opentx-companion-open-settings.png"><img src="/assets/collections/fpv/opentx/opentx-companion-open-settings.png"></a>
    <a href="/assets/collections/fpv/opentx/opentx-companion-settings.png"><img src="/assets/collections/fpv/opentx/opentx-companion-settings.png"></a>
    <figcaption>OpenTX Companion.</figcaption>
</figure>

Here you can see the settings in the `Radio Profile` tab. 
As `Radio Type` choose the `FrSky Taranis X9D+ 2019` and the following `Build Options`:

- `lua` enables lua custom script screen
- `noheli` to disable the Helicoper menu page on the Taranis in the case you are not flying a helicopter.
- `flexr9m` or `eu` which can be both used to flash the internal transmitter module from the FCC to the EU-LBT firmware.

Use `flexr9m` if you have an external transmitter module such as the FrSky R9M and want to flash its firmware.
In case you ordered from a vendor that shiped the Taranis X9D with the FCC firmware on the internal transmitter module and you don't have an external module choose either `flexr9m` or `eu`. `flexr9m` lets you also flash the EU-LBT firmware. 
Note that you cannot check both, so you have to choose between `flexr9m` or `eu`.
{: .note}



Also choose an `SD Structure path` on your Mac or PC that will mirror the internal storage of the Taranis to your local machine. Note that this is NOT the path where we will place the previously downloaded sd card content.

If you want you can change the `Default Stick Mode` or the `Default Channel Order`. For convenience, select the two options:

- `Append version number to FW file name`
- `Offer to write FW to Tx after download`

Save the radio profile using a `Profile Name` and press `Ok`.

## Enter the Bootloader of Taranis X9D Plus

Now that we are done with the settings of the firmware that will be flashed on the radio we need to enter the bootloader.
To do that, press both horizontal trim switches to the middle while powering the transmitter. 

Note: Don't press the power button for to long in order to enter the bootloader and not booting into the currently installed firmware.
{: .note}

<figure class="half">
    <a href="/assets/collections/fpv/taranis/enter-bootloader.jpg"><img src="/assets/collections/fpv/taranis/enter-bootloader.jpg"></a>
    <a href="/assets/collections/fpv/taranis/bootloader.jpg"><img src="/assets/collections/fpv/taranis/bootloader.jpg"></a>
    <figcaption>OpenTX Companion.</figcaption>
</figure>

On the bootloader screen above you can also check the bootloader version which is important if you want to update the bootloader. The optional update procedure is explained at the end of this post [Update Bootloader (Optional)](/projects/fpv/taranis/#update-bootloader-(optional)).

## Flash OpenTX

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

## SD Card Content

It is required to prepare an external sd card with the content downloaded from the [OpenTX website](https://downloads.open-tx.org/2.3/release/sdcard/). For the preperation put an empty sd card in the back tray of your Taranis. 
In this post it is called `EXTTARANIS`.

<figure>
    <a href="/assets/collections/fpv/taranis/sdcard.jpg"><img src="/assets/collections/fpv/taranis/sdcard.jpg"></a>
    <figcaption>Insert an empty sd card in your Taranis.</figcaption>
</figure>

When you've inserted the sd card, power on the Taranis and connect it with the USB cable to your computer. 
A message will pop up on your Taranis, where you should select `USB Storage (SD)`, as shown in the following image: 

<figure>
    <a href="/assets/collections/fpv/taranis/usb-storage.jpg"><img src="/assets/collections/fpv/taranis/usb-storage.jpg"></a>
    <figcaption>Select `USB Storage (SD)` after you plugged in your USB cable.</figcaption>
</figure>

### Scripts Folder for ISRM lua Script

Next, place the content of the downloaded sd card zip file on the empty sd card and open the `SCRIPTS/TOOLS` folder 
where you copy the lua script for changing the internal module into the FCC or EU-LBT mode.

<figure class="half">
    <a href="/assets/collections/fpv/taranis/copy-sd-content.png"><img src="/assets/collections/fpv/taranis/copy-sd-content.png"></a>
    <a href="/assets/collections/fpv/taranis/copy-lua-script.png"><img src="/assets/collections/fpv/taranis/copy-lua-script.png"></a>
    <figcaption>Copy the sd card content and the lua script on the external Taranis sd card.</figcaption>
</figure>

The mentioned lua script to switch the internal module into the FCC or EU-LBT mode can be obtained from the FrSky support.
Because it is not allowed to share this script, just write a mail to the [FrSky support](mailto:frsky@frsky-rc.com) asking for the lua script. It is named `isrm_mode_1.lua`. Using this script you must choose the correct firmware that is allowed in your area.
{: .notice--warning}

### EEPROM Folder for Bootloader

If you would like to use the latest bootloader that comes with the OpenTX firmware then copy the `*.bin` file to the `EEPROM` folder of your external `EXTARANIS` sd card and rename it to something short `opentx-2.3.1.bin`. Remember that the `*.bin` firmware file was downloaded via OpenTX Companion.

Follow the [steps](/projects/fpv/taranis/#update-bootloader-(optional)) at the end of this post to update the bootloader.

## Update Internal Module Firmware (Optional)

In case you want to update the firmware of the internal transmitter module also refered to as [ISRM](/projects/fpv/glossar#isrm), 
download the latest firmware from the [FrSky website](https://www.frsky-rc.com/taranis-x9d-plus-se-2019/). 
Once downloaded, unzip the firmware file, for example `ISRM_S_X9_190812.frk` and place it in the `FIRMWARE` folder of
the external sd card of your Taranis.

![image-right]({{ site.url }}{{ site.baseurl }}/assets/collections/fpv/taranis/copy-isrm-firmware.png){: .align-right}

After ejecting the `Taranis` and `EXTTARANIS` from you computer, unplug the usb cable. 
Then, long press the `MENU` button and press `PAGE` to switch to the `SD-HC CARD` page. Here, enter the `[FIRMWARE]` folder.

<figure class="half">
    <a href="/assets/collections/fpv/taranis/sd-card-content-firmware.jpg"><img src="/assets/collections/fpv/taranis/sd-card-content-firmware.jpg"></a>
    <a href="/assets/collections/fpv/taranis/sd-card-isrm.jpg"><img src="/assets/collections/fpv/taranis/sd-card-isrm.jpg"></a>
    <figcaption>SD card content of the EEPROM folder.</figcaption>
</figure>

Long press the `*.frk` firmware file, for example `ISRM_S_X9_190812.frk`, to get the following pop up screen, 
where you select `Flash internal module`:

<figure class="third">
    <a href="/assets/collections/fpv/taranis/flash-isrm.jpg"><img src="/assets/collections/fpv/taranis/flash-isrm.jpg"></a>
    <a href="/assets/collections/fpv/taranis/flashing-isrm.jpg"><img src="/assets/collections/fpv/taranis/flashing-isrm.jpg"></a>
  <a href="/assets/collections/fpv/taranis/flashing-isrm-success.jpg"><img src="/assets/collections/fpv/taranis/flashing-isrm-success.jpg"></a>
    <figcaption>Flash the internal transmitter module (ISRM).</figcaption>
</figure>

After writing the new firmware, you can verify the version in the `VERSION` page, which you can get to by long pressing the `MENU` button followed by repeatedly pressing `PAGE` until you get there. Selecting `[Modules / RX version]`
will show you the isrm version and its currently set mode.

<figure class="half">
    <a href="/assets/collections/fpv/taranis/version-modules.jpg"><img src="/assets/collections/fpv/taranis/version-modules.jpg"></a>
    <a href="/assets/collections/fpv/taranis/module-version-eu-lbt.jpg"><img src="/assets/collections/fpv/taranis/module-version-eu-lbt.jpg"></a>
    <figcaption>Updated internal transmitter module to latest firmware.</figcaption>
</figure>


## Switching Internal Module between EU-LBT Mode and FCC Firmware

After preparing the external sd card of the Taranis unplug the USB cable (after ejecting both storage locations `TARANIS` and `EXTTARANIS` from your computer). Then power on your Taranis and enter the `RADIO SETUP` menu by long pressing the 
`MENU` button of your Taranis. Press `PAGE` repeatedly until you get to the `TOOLS` menu, where you select `Change ISRM mode`
to change the internal transmitter module into the EU-LBT mode using the lua script. 


<figure class="half">
    <a href="/assets/collections/fpv/taranis/change-isrm-mode.jpg"><img src="/assets/collections/fpv/taranis/change-isrm-mode.jpg"></a>
    <a href="/assets/collections/fpv/taranis/eu-lbt-mode.jpg"><img src="/assets/collections/fpv/taranis/eu-lbt-mode.jpg"></a>
    <figcaption>Change the internal transmitter module to EU-LBT mode inside the TOOLS menu.</figcaption>
</figure>

On the `VERSION` page we can see firmware version of OpenTX, the options we set inside the Settings of the OpenTX Companion application and that internal module is now in EU-LBT mode.

<figure class="half">
    <a href="/assets/collections/fpv/taranis/version.jpg"><img src="/assets/collections/fpv/taranis/version.jpg"></a>
    <a href="/assets/collections/fpv/taranis/firmware-settings.jpg"><img src="/assets/collections/fpv/taranis/firmware-settings.jpg"></a>
    <figcaption>OpenTX firmware version and its previously selected settings.</figcaption>
</figure>

The following two images show the different modes of the internal module. Once again, select the one appropriate for your region.


<figure class="half">
    <a href="/assets/collections/fpv/taranis/module-version-eu-lbt.jpg"><img src="/assets/collections/fpv/taranis/module-version-eu-lbt.jpg"></a>
    <a href="/assets/collections/fpv/taranis/module-version-fcc.jpg"><img src="/assets/collections/fpv/taranis/module-version-fcc.jpg"></a>
    <figcaption>Internal transmitter module versions. EU-LBT and FCC.</figcaption>
</figure>


## Update Bootloader (Optional)

If you have a new Taranis, you probably won't need to update its bootloader in order to flash a new version of OpenTX. However, make sure that the version of OpenTX matches the bootloader by comparing the bootloader version with the 
version of OpenTX you want to flash. The bootloader version is apparent on top of the bootloader menu.

As mentioned perviously copy and renamed the OpenTX firmware 
(downloaded via OpenTX Companion) to the `EEPROM` folder of your external sd card `EXTTARANIS` and rename it to something short, for example `opentx-2.3.1.bin`. Unplug the Taranis and enter the `RADIO SETUP` menu by long pressing the `MENU` button on your Taranis. Press `PAGE` to get to the `SD-HC CARD` page and enter the `[EEPROM]` folder. 

<figure class="half">
    <a href="/assets/collections/fpv/taranis/bl-sdcard-content.jpg"><img src="/assets/collections/fpv/taranis/bl-sdcard-content.jpg"></a>
    <a href="/assets/collections/fpv/taranis/bl-sdcard-eeprom.jpg"><img src="/assets/collections/fpv/taranis/bl-sdcard-eeprom.jpg"></a>
    <figcaption>EEPROM folder on SD card.</figcaption>
</figure>

Inside the `EEPROM` folder you should see the `opentx-2.3.1.bin` firmware, which can be used to update the bootloader by long pressing the scroll button.

<figure>
    <a href="/assets/collections/fpv/taranis/flash-bootloader.jpg"><img src="/assets/collections/fpv/taranis/flash-bootloader.jpg"></a>
    <figcaption>Flash bootloader.</figcaption>
</figure>


Select `Flash bootloader` in the menu that pops up and after the update finished, 
boot into the bootloader to verify that the update succeeded. 

<figure>
    <a href="/assets/collections/fpv/taranis/flash-bootloader.jpg"><img src="/assets/collections/fpv/taranis/bootloader-updated.jpg"></a>
    <figcaption>Check updated bootloader version.</figcaption>
</figure>







