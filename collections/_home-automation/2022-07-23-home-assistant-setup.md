---
layout: single
title:  "Home Automation - Raspberry Pi Setup"
permalink: /projects/home-automation/setup/
excerpt: "Setup Home Assistant on an Raspberry Pi with Raspberry Pi OS running on an SSD."
date:   2022-07-23 10:00:35 +0100
categories: [home automation, raspberry pi, home assistant]
tags: [home, home automation, home assistant, home, assistant, raspberry pi, os, raspberry, ssd, setup, argon, one, m.2, case]
comments: true
use_math: true
toc: true
classes: wide
# toc_label: "Unscented Kalman Filter"
header:
  teaser: /assets/collections/home-automation/components/argon-m-2.jpg
  overlay_image: /assets/collections/home-automation/components/argon-m-2.jpg
  overlay_filter: 0.5
redirect_from:
  - /projects/home-automation/
sidebar:
  nav: "home-automation"
---

This page explains how to setup a Raspberry Pi 4 with 8GB and an SSD to run [Home Assistant](https://www.home-assistant.io/).

The case I use for testing is the [Argon One m.2](https://amzn.to/3PRzwxW). Later the Home Assistant setup might be moved to another place.
For example running on a Virtual machine on a home server. However, for inital testing the Raspberry Pi is fine.

<figure >
    <a href="/assets/collections/home-automation/components/argon-one-m-2.jpg"><img src="/assets/collections/home-automation/components/argon-one-m-2.jpg"></a>
    <figcaption>Argon One m.2 case</figcaption>
</figure>

As you can see in the image the Raspberry Pi is connected via USB 3.0 to the SSD card. Therefore the Raspberry Pi needs to boot from USB instead from the SD card.

## Boot from USB

To boot from an USB the Raspberry Pi 4 firmware has to be prepared.
For this, we use [Rapberry Pi imager](https://www.raspberrypi.com/software/), which is available for Linux, MacOS and Windows and has a much simpler means to prepare a Raspberry Pi 4 / 400 for USB boot.
These instructions will set the Raspberry Pi 4 / 400 to look for a USB boot device, if none is found it will then boot from the micro SD card.

1. Download and install Raspberry Pi Imager from the Raspberry Pi website. 

2. Insert a spare micro SD card into your computer. Note that this card will be erased.

3. Launch Raspberry Pi Imager and under Operating System scroll down to Misc Utility Images and left click to open the next menu.

    <figure >
        <a href="/assets/collections/home-automation/rpi-imager/01-rpi-imager-misc-utility-images.png"><img src="/assets/collections/home-automation/rpi-imager/01-rpi-imager-misc-utility-images.png"></a>
        <figcaption>rpi imager USB boot</figcaption>
    </figure>

4. Select Bootloader and then Select USB Boot. This will return us to the main menu. 

    <figure >
        <a href="/assets/collections/home-automation/rpi-imager/rpi-imager-usb-boot.gif"><img src="/assets/collections/home-automation/rpi-imager/rpi-imager-usb-boot.gif"></a>
        <figcaption>rpi imager USB boot</figcaption>
    </figure>

5. Under Storage click on the button and select the micro SD card. Double check that you have the right drive before proceeding.

    <figure >
        <a href="/assets/collections/home-automation/rpi-imager/01-rpi-imager-misc-utility-images.png"><img src="/assets/collections/home-automation/rpi-imager/01-rpi-imager-misc-utility-images.png"></a>
        <figcaption>rpi imager USB boot</figcaption>
    </figure>

6. Click on Write to download and write a configuration image to the micro SD card. When done remove the card from your computer.

7. Insert the micro SD card into your Raspberry Pi 4 / 400 and power on.
   The green activity light will blink a steady pattern once the update has been completed.
   If you have an HDMI monitor attached, the screen will go green once the update is complete.
   Allow 10 seconds or more for the update to complete, do not remove the micro SD card until the update is complete.
   
8. Power off the Raspberry Pi and remove the micro SD card.

9. Flash Raspberry Pi OS 64 bit onto the SD card using again the Raspberry Pi Imager tool.

10. Into your Raspberry Pi, insert a micro SD card with Raspberry Pi OS and boot from micro SD to the desktop.
   This may take a little longer as the Raspberry Pi is looking for USB boot devices. It is also likely that the Raspberry Pi will reboot at least one time
   until it reaches the final Raspberry Pi OS desktop screen.

11. Launch SD Card Copier from the Accessories section of the start menu. Ensure that your SSD or Flash drive is connected to the Raspberry Pi using a USB 3 port.


    <figure >
        <a href="/assets/collections/home-automation/sd-card-copier/01-sd-card-copier.png"><img src="/assets/collections/home-automation/sd-card-copier/01-sd-card-copier.png"></a>
        <figcaption>Open SD Card Copier</figcaption>
    </figure>

12. Select the Copy From Device (micro SD card), and the Copy To Device (the SSD). Double check that the correct drives are selected and click Start to copy the files across. The process should take around ten minutes to complete.  

    <figure >
        <a href="/assets/collections/home-automation/sd-card-copier/02-sd-card-copier-from-to.png"><img src="/assets/collections/home-automation/sd-card-copier/02-sd-card-copier-from-to.png"></a>
        <figcaption>Select from where to copy the OS</figcaption>
    </figure>

13. Accept that the content on the SSD will be removed

    <figure >
        <a href="/assets/collections/home-automation/sd-card-copier/03-sd-card-copier-earase-true.png"><img src="/assets/collections/home-automation/sd-card-copier/03-sd-card-copier-earase-true.png"></a>
        <figcaption>Accept eraseing the SSD</figcaption>
    </figure>

14. Then click start and see the copying process

    <figure >
        <a href="/assets/collections/home-automation/sd-card-copier/04-sd-card-copier-prepare.png"><img src="/assets/collections/home-automation/sd-card-copier/04-sd-card-copier-prepare.png"></a>
        <figcaption>Preparation step</figcaption>
    </figure>

    <figure >
        <a href="/assets/collections/home-automation/sd-card-copier/05-sd-card-copier-copy.png"><img src="/assets/collections/home-automation/sd-card-copier/05-sd-card-copier-copy.png"></a>
        <figcaption>Copying ...</figcaption>
    </figure>

15. Wait for the process to complete

    <figure >
        <a href="/assets/collections/home-automation/sd-card-copier/06-sd-card-copier-complete.png"><img src="/assets/collections/home-automation/sd-card-copier/06-sd-card-copier-complete.png"></a>
        <figcaption>SD Card to SSD copy complete</figcaption>
    </figure>

16. Shut down the Raspberry Pi.
17. Remove the microSD card.
18. Power up the Raspberry Pi and it will boot from the USB SSD or Flash drive.

## Install the fan and power button driver

1. The fan and power button can only work with the driver. You can directly refer to the manual which comes with the case.

    <figure >
        <a href="https://www.waveshare.com/wiki/PI4-CASE-ARGON-ONE-M.2#/media/File:PI4-CASE-ARGON-ONE-14_960.jpg"><img src="https://www.waveshare.com/wiki/PI4-CASE-ARGON-ONE-M.2#/media/File:PI4-CASE-ARGON-ONE-14_960.jpg"></a>
        <figcaption>FAN jumper settings (source Waveshare)</figcaption>
    </figure>


2. Open terminal of Raspberry Pi, install the driver by the following command

    ```console
    curl https://download.argon40.com/argon1.sh | bash
    ```

    <figure >
        <a href="/assets/collections/home-automation/argon-driver/01-argon-driver.png"><img src="/assets/collections/home-automation/argon-driver/01-argon-driver.png"></a>
        <figcaption>Install argon driver</figcaption>
    </figure>
    
3. After installation, two programs are available, "configure" and "uninstall", they can be used to configure and uninstall the driver.

    ```console
    argonone-config  #configure driver
    argonone-uninstall  #uninstall driver
    ```
    
4. Configure the default fan settings

    ```console
    $ argonone-config 
    --------------------------------------
    Argon One Fan Speed Configuration Tool
    --------------------------------------
    WARNING: This will remove existing configuration.
    Press Y to continue:Y
    Thank you.

    Select fan mode:
      1. Always on
      2. Adjust to temperatures (55C, 60C, and 65C)
      3. Customize behavior
      4. Cancel
    NOTE: You can also edit /etc/argononed.conf directly
    Enter Number (1-4):2

    Please provide fan speeds for the following temperatures:
    55C (0-100 only):10
    60C (0-100 only):55
    65C (0-100 only):100
    Configuration updated.
    ```
    

    <figure >
        <a href="/assets/collections/home-automation/argon-driver/02-argon-driver-config.png"><img src="/assets/collections/home-automation/argon-driver/02-argon-driver-config.png"></a>
        <figcaption>Configure argon driver</figcaption>
    </figure>


## References

- https://www.tomshardware.com/how-to/boot-raspberry-pi-4-usb
- https://www.waveshare.com/wiki/PI4-CASE-ARGON-ONE-M.2
