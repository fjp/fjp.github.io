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

[FrSky R-XSR SmartPort FPort Receiver](https://www.frsky-rc.com/product/r-xsr/). 




<figure>
    <a href="/assets/collections/fpv/receiver/r-xsr-top.jpg"><img src="/assets/collections/fpv/receiver/r-xsr-top.jpg"></a>
    <figcaption>FrSky R-XSR Receiver.</figcaption>
</figure>

## Receiver Firmware Update

This section explains how to [flash](/projects/fpv/glossar/#flash) the latest [receiver](/projects/fpv/glossar/#receiver) [firmware](/projects/fpv/glossar/#firmware) for an R-XSR receiver and how to [bind](/projects/fpv/glossar/#receiver) it to the [Taranis](/projects/fpv/glossar/#taranis) X9D Plus 2019 [transmitter](/projects/fpv/glossar/#receiver).

Download the latest [ACCESS](/projects/fpv/glossar/#access) firmware from the 
[product page](https://www.frsky-rc.com/r-xsr/). Here you can decide between [S.Port](/projects/fpv/glossar/#smartport) and [F.Port](/projects/fpv/glossar/#fport) variants. 
If you don't know what to choose, it is recommended to select the newer F.Port firmware, as it allows you to 
connect your receiver with just a single wire to your [flight controller](/projects/fpv/glossar/#flight-controller).

Choose for example, `FW-RXSR-ACCESS_v1.1.4.zip` and unpack the zip file.
Copy the content to the external sd card of your Taranis and place it in the `FIRMWARES` folder.
Especially the file `RXSR-FPORT_ACCESS_191107.frsk` will be relevant. 

With your Taranis powerd off, connect the R-XSR receiver to the SmartPort (S.Port) of the Taranis X9D Plus 2019. 
You find this port in the battery box of the X9DP 2019 radio. 

Note that you don't connect the receiver to the pins that are located in the back compartment where usually an external transmitter module is installed. This port was used on older models of the Taranis but cannot be used with the 2019 model.
{: .notice--warning}

After you have connected the receiver to the S.Port, located in the battery compartment, navigate to the `FIRMWARES` folder located on your Taranis, long press the dowloaded firmware `RXSR-FPORT_ACCESS_191107.frsk` file and select "Flash S.Port device".


This will flash the firmware of the R-XSR receiver.


## How to use the Taranis X9D Plus Transmitter to Flash the Receiver Firmware

https://www.frsky-rc.com/how-to-use-the-transmitter-to-flash-the-firmware-of-the-x8r-receiver/


## References

https://blog.seidel-philipp.de/how-to-flash-a-frsky-smartport-receiver-with-taranis-9xd-or-qx7s/
