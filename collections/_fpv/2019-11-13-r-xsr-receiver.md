---
layout: single
title:  "FrSky R-XSR Receiver"
permalink: /projects/fpv/r-xsr
excerpt: "All about the R-XSR SmartPort and FPort Receiver from FrSky."
date: 2019-10-30 09:00:35 +0100
categories: [fpv, quad]
tags: [fpv, quad, race, drone, r-xsr, receiver, smartport, fport]
comments: true
use_math: true
toc: true
classes: wide
# toc_label: "Unscented Kalman Filter"
header:
  teaser: /assets/collections/fpv/receiver/r-xsr.jpg
  overlay_image: /assets/collections/fpv/receiver/r-xsr.jpg
  overlay_filter: 0.5
redirect_from:
  - /projects/fpv/
sidebar:
  nav: "fpv"
---

This post is abou the [FrSky R-XSR SmartPort Receiver](https://www.frsky-rc.com/product/r-xsr/) which is also capabale of the new [F.Port](/projects/fpv/glossar/#fport) protocol that requires only a single connection to the [Flight Controller](/projects/fpv/glossar/#flight-controller) for communication. 


<figure class="half">
    <a href="/assets/collections/fpv/receiver/r-xsr-top.jpg"><img src="/assets/collections/fpv/receiver/r-xsr-top.jpg"></a>
    <a href="/assets/collections/fpv/receiver/r-xsr-manual.jpg"><img src="/assets/collections/fpv/receiver/r-xsr-manual.jpg"></a>
    <figcaption>FrSky R-XSR Receiver.</figcaption>
</figure>

## Receiver Firmware Update

This section explains how to [flash](/projects/fpv/glossar/#flash) the latest [receiver](/projects/fpv/glossar/#receiver) [firmware](/projects/fpv/glossar/#firmware) for an R-XSR receiver and how to [bind](/projects/fpv/glossar/#receiver) it to the [Taranis](/projects/fpv/glossar/#taranis) X9D Plus 2019 [transmitter](/projects/fpv/glossar/#receiver).

Download the latest [ACCESS](/projects/fpv/glossar/#access) firmware from the 
[product page](https://www.frsky-rc.com/r-xsr/). Here you can decide between [S.Port](/projects/fpv/glossar/#smartport) and [F.Port](/projects/fpv/glossar/#fport) variants. 
If you don't know what to choose, it is recommended to select the newer F.Port firmware, as it allows you to 
connect your receiver with just a single wire to your [Flight Controller](/projects/fpv/glossar/#flight-controller).

Choose for example, `FW-RXSR-ACCESS_v1.1.4.zip`, which is, at the time of writing, the latest firmware and includes the F.Port firmware. Especially the file `RXSR-FPORT_ACCESS_191107.frsk` will be relevant for flashing. Unpack the zip file and copy the content to the `FIRMWARES` folder (create it, if it doesn't exist) on the external sd card of your Taranis.


With your Taranis powerd off, connect the R-XSR receiver to the SmartPort (S.Port) of the Taranis X9D Plus 2019. 
You find this port in the battery box of the X9DP 2019 radio. 

Note that you don't connect the receiver to the pins that are located in the back compartment where usually an external transmitter module is installed. This port was used on older models of the Taranis but cannot be used with the 2019 model.
{: .notice--warning}

After you have connected the receiver to the S.Port, located in the battery compartment, navigate to the `FIRMWARES` folder located on your Taranis, long press the dowloaded firmware `RXSR-FPORT_ACCESS_191107.frsk` file and select "Flash S.Port device".


This will flash the latest F.Port firmware onto the R-XSR receiver.


For other helpful resources on how to flash the receiver, look at the official FrSky [support page](https://www.frsky-rc.com/how-to-use-the-transmitter-to-flash-the-firmware-of-the-x8r-receiver/) and the [R-XSR manual](https://www.frsky-rc.com/wp-content/uploads/Downloads/Manual/X9DP2019/X9D%20Plus%202019%20X9D%20Plus%20SE%202019-Manual.pdf). 


## Register and Bind R-XSR to Taranis X9D Plus 2019

First register the receiver with its unique name to the your transmitter

<figure class="third">
    <a href="/assets/collections/fpv/receiver/bind/01-model-setup.jpg"><img src="/assets/collections/fpv/receiver/bind/01-model-setup.jpg"></a>
    <a href="/assets/collections/fpv/receiver/bind/02-register.jpg"><img src="/assets/collections/fpv/receiver/bind/02-register.jpg"></a>
    <a href="/assets/collections/fpv/receiver/bind/03-register-ok.jpg"><img src="/assets/collections/fpv/receiver/bind/03-register-ok.jpg"></a>
    <figcaption>Register R-XSR receiver with Taranis X9D Plus 2019.</figcaption>
</figure>


After registration you can start the binding process:

<figure class="third">
    <a href="/assets/collections/fpv/receiver/bind/04-bind.jpg"><img src="/assets/collections/fpv/receiver/bind/04-bind.jpg"></a>
    <a href="/assets/collections/fpv/receiver/bind/05-select-rx.jpg"><img src="/assets/collections/fpv/receiver/bind/05-select-rx.jpg"></a>
    <a href="/assets/collections/fpv/receiver/bind/06-bind-ok.jpg"><img src="/assets/collections/fpv/receiver/bind/06-bind-ok.jpg"></a>
    <figcaption>Bind R-XSR receiver with Taranis X9D Plus 2019.</figcaption>
</figure>

The final result will look like this:

<figure>
    <a href="/assets/collections/fpv/receiver/bind/07-result.jpg"><img src="/assets/collections/fpv/receiver/bind/07-result.jpg"></a>
    <figcaption>Successful binding of R-XSR receiver with Taranis X9D Plus 2019.</figcaption>
</figure>


## Failsafe Mode

With an X-Series receiver it is possible to set the Failsafe just in the transmitter. Set it to No pulses.


## References

https://blog.seidel-philipp.de/how-to-flash-a-frsky-smartport-receiver-with-taranis-9xd-or-qx7s/
[How To Set Up FrSky Taranis and Betaflight / Cleanflight Configuration](https://youtu.be/Z6ZfTTmDTrc?t=325)
