---
layout: single
title:  "Home Automation - Raspberry Pi Setup"
permalink: /projects/homeautomation/setup
excerpt: "Setup Home Assistant on an Raspberry Pi with Raspberry Pi OS running on an SSD."
date:   2020-05-13 09:00:35 +0100
categories: [home automation, raspberry pi, home assistant]
tags: [home, home automation, home assistant, home, assistant, raspberry pi, os, raspberry, ssd, setup, argon, one, m.2, case]
comments: true
use_math: true
toc: true
classes: wide
# toc_label: "Unscented Kalman Filter"
header:
  teaser: /assets/collections/home-automation/components/propellers.jpg
  overlay_image: /assets/collections/home-automation/propellers.jpg
  overlay_filter: 0.5
redirect_from:
  - /projects/homeassistant/
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



## References

- https://www.tomshardware.com/how-to/boot-raspberry-pi-4-usb
