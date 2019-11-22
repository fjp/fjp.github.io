---
layout: single
title:  "Race Quad Components"
permalink: /projects/fpv/components
excerpt: "Components of the fpv race quad."
date:   2019-09-18 20:41:35 +0200
categories: [fpv, quad]
tags: [getfpv, fpv, quad, race, drone, camera, props, motors, esc, brushless, goggles]
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

The following sections outline the main components that are required to start with [FPV](/projects/fpv/glossar/#fpv) 
and to build your first drone. The list is not complete and some parts maybe outdated when you read this page.
It may be even wiser to buy a single component that combines multiple parts listed below, depending on what you desire.
However, this list should guide you what the important parts of a race quad are and what I would've liked to know
when I started this hobby. The shop links show where you can get the individual parts and are most of the time affiliate links. When you buy the parts using this link it does not cost you more but you support me. I provide the links to 
the original equipement manufacturer (OEM) where you can find datasheets and further informations such as installation guides. If you would like to learn more about the operation of each part and how they interact with each other read
the next pages which cover the theory behind FPV. On the left you see the menu to this project.


## Batteries/Charger

<figure class="third">
    <a href="/assets/collections/fpv/components/ev-peak-cq3-4x-100w-lead_2.jpg"><img src="/assets/collections/fpv/components/ev-peak-cq3-4x-100w-lead_2.jpg" width="600"></a>
    <a href="/assets/collections/fpv/components/ev-peak-cellmeter-7-battery-capacity-checker.jpg"><img src="/assets/collections/fpv/components/ev-peak-cellmeter-7-battery-capacity-checker.jpg" width="600"></a>
    <figcaption>EV-Peak CQ3 Multi Charger 4x 100W NiMH / LiPO with Built-in Balance and EV-Peak Cellmeter-7 Battery Capacity Checker</figcaption>
</figure>

| Component         | Description                            | Shops                 | OEM Link            | Comment |
|:-----------------:|:--------------------------------------:|:---------------------:|:-------------------:|         |
| Charger           | EV-Peak CQ3 Multi Charger 4x 100W NiMH / LiPO with Built-in Balance | [getfpv](https://www.getfpv.com/ev-peak-cq3-multi-charger-4x-100w-nimh-lipo-with-built-in-balance.html?cmid=eHZ3Y2tBWGYrQWM9&afid=TVZmU1BzYnlObnc9&ats=WDA0ZG1qK1ZCcW89)  | [EV-Peak](https://www.ev-peak.com/prodcuts-item/ev-peak-cq3/) | See OEM link for the manual | 
| Battery Checker   | EV-Peak Cellmeter-7 Battery Capacity Checker | [getfpv](https://www.getfpv.com/ev-peak-cellmeter-7-battery-capacity-checker.html?cmid=eHZ3Y2tBWGYrQWM9&afid=TVZmU1BzYnlObnc9&ats=WDA0ZG1qK1ZCcW89), [Banggood](https://www.banggood.com/CellMeter-7-Battery-Capacity-Checker-Tester-LiPo-LiFe-Li-ion-NiMH-NiCd-p-85223.html?rmmds=search&cur_warehouse=CN&p=GQ230138854743201909&custlinkid=609282) | [EV-Peak](http://www.ev-peak.com.hk/page179?product_id=2404) |  |
| Battery        | Lumenier 1300mah 3S 60C Lipo Battery (XT60) | [getfpv](https://www.getfpv.com/lumenier-1300mah-3s-60c-lipo-battery-xt60.html?cmid=eHZ3Y2tBWGYrQWM9&afid=TVZmU1BzYnlObnc9&ats=WDA0ZG1qK1ZCcW89) | OEM Link |    |


## Flight Controller (FC) and Power Distribution Board (PDB)

<figure class="half">
    <a href="/assets/collections/fpv/components/mateksys-F722-STD.jpg"><img src="/assets/collections/fpv/components/mateksys-F722-STD.jpg"></a>
    <a href="/assets/collections/fpv/components/mateksys-fchub-6s.jpg"><img src="/assets/collections/fpv/components/mateksys-fchub-6s.jpg"></a>
    <figcaption>Flight Controller MATEKSYS-F722 STD and Power Distribution Board MATEKSYS FCHUB-6S</figcaption>
</figure>

| Component         | Description                            | Shops                 | OEM Link            | Comment |
|:-----------------:|:--------------------------------------:|:---------------------:|:-------------------:|         |
| Flight Controller | MATEKSYS F722 STD STM32 Built in OSD BMP280 Barometer Blackbox | [getfpv](https://www.getfpv.com/matek-systems-f722-std-flight-controller-w-f7-32k-gyro-bfosd-barometer.html?cmid=eHZ3Y2tBWGYrQWM9&afid=TVZmU1BzYnlObnc9&ats=WDA0ZG1qK1ZCcW89), [Banggood](https://www.banggood.com/Matek-Systems-F722-STD-STM32F722-Flight-Controller-Built-in-OSD-BMP280-Barometer-Blackbox-for-RC-Drone-p-1225166.html?rmmds=myorder&cur_warehouse=UK&p=GQ230138854743201909&custlinkid=604567)  | [MATEKSYS](http://www.mateksys.com/?portfolio=f722-std) | See OEM link for the datasheet | 
| PDB       | MATEKSYS FCHUB-6S W/ CURRENT SENSOR 184A, BEC 5V & 10V | [getfpv](https://www.getfpv.com/matek-fchub-6s-pdb.html?cmid=eHZ3Y2tBWGYrQWM9&afid=TVZmU1BzYnlObnc9&ats=WDA0ZG1qK1ZCcW89), [Banggood](https://www.banggood.com/Matek-FCHUB-6S-Hub-Power-Distribution-Board-5V-10V-BEC-Built-in-184A-Current-Sensor-p-1147591.html?rmmds=myorder&cur_warehouse=CN&p=GQ230138854743201909&custlinkid=604638) | [MATEKSYS](http://www.mateksys.com/?portfolio=fchub-6s#tab-id-1) |  |


## FPV-System (Goggles, Camera)

<figure class="third">
    <a href="/assets/collections/fpv/components/ultimate-fatshark-hdo-antenna-bundle.jpg"><img src="/assets/collections/fpv/components/ultimate-fatshark-hdo-antenna-bundle.jpg"></a>
    <a href="/assets/collections/fpv/components/runcam-swift-2.jpg"><img src="/assets/collections/fpv/components/runcam-swift-2.jpg"></a>
  <a href="/assets/collections/fpv/components/tbs-unify-pro-hv.jpg"><img src="/assets/collections/fpv/components/tbs-unify-pro-hv.jpg"></a>
    <figcaption>FPV system: FatShark HDO, RunCam Swift 2 and [VTX](/projects/fpv/glossar/#vtx) [TBS](/projects/fpv/glossar/#tbs) Unify Pro HV.</figcaption>
</figure>

| Component  | Description                            | Shops                 | OEM Link            | Comment |
|:----------:|:--------------------------------------:|:---------------------:|:-------------------:|         |
| FPV Bundle | Ultimate FPV Bundle - Fat Shark HDO, rapidFIRE, + Lumenier AXII 2 Diversity Antenna Bundle | [getfpv](https://www.getfpv.com/ultimate-fpv-bundle.html?cmid=eHZ3Y2tBWGYrQWM9&afid=TVZmU1BzYnlObnc9&ats=WDA0ZG1qK1ZCcW89),  | [FatShark](https://www.fatshark.com/product/hdo-fpv-goggles/), [immersionRC](https://www.immersionrc.com/fpv-products/rapidfire/), [Lumenier](https://www.lumenier.com/products/antennas)  | See OEM links for the datasheets | 
| Camera       | RunCam Swift 2 (2.5mm Lens)          | [getfpv](https://www.getfpv.com/runcam-swift-2-2-5mm-lens-orange.html?cmid=eHZ3Y2tBWGYrQWM9&afid=TVZmU1BzYnlObnc9&ats=WDA0ZG1qK1ZCcW89), [Banggood](https://www.banggood.com/RunCam-Swift-2-13-CCD-PAL-Micro-Camera-FOV-130150165-Degree-2_5mm2_3mm2_1mm-Integrated-OSD-MIC-p-1118948.html?rmmds=myorder&ID=226517043&cur_warehouse=UK&p=GQ230138854743201909&custlinkid=604623) | [RunCam](https://shop.runcam.com/runcam-swift-2/) |  |
| [VTX](/projects/fpv/glossar/#vtx) | [TBS](/projects/fpv/glossar/#tbs) Unify Pro HV  | [getfpv](/) | [Team Black Sheep](https://www.team-blacksheep.com/tbs-unify-pro-manual-de.pdf) |  |


## Antennas

## Motors and ESC

<figure class="half">
    <a href="/assets/collections/fpv/components/emax-rs2205.jpg"><img src="/assets/collections/fpv/components/emax-rs2205.jpg"></a>
    <a href="/assets/collections/fpv/components/dys-aria-35a-esc.jpg"><img src="/assets/collections/fpv/components/dys-aria-35a-esc.jpg"></a>
    <figcaption>EMAX RS2205/2300Kv Motors and DYS ARIA BLHELI 35A ESCs.</figcaption>
</figure>

| Component | Description                            | Shops                 | OEM Link            | Comment |
|:---------:|:--------------------------------------:|:---------------------:|:-------------------:|         |
| Motor     | EMAX RS2205/2300Kv RaceSpec Motor (CW) | [getfpv](https://www.getfpv.com/emax-rs2205-2300kv-racespec-motor-cw.html?cmid=eHZ3Y2tBWGYrQWM9&afid=TVZmU1BzYnlObnc9&ats=WDA0ZG1qK1ZCcW89), [Banggood](https://www.banggood.com/4X-Emax-RS2205-2300-Racing-Edition-CWCCW-Motor-For-FPV-Multicopter-p-1032857.html?rmmds=myorder&cur_warehouse=CN&p=GQ230138854743201909&custlinkid=604589) | [EMAX](https://emaxmodel.com/emax-rs2205-racespec-motor.html)  | See OEM Link for the Datasheet | 
| ESC       | DYS ARIA BLHELI_32BIT 35A ESC          | [getfpv](https://www.getfpv.com/dys-aria-blheli-32bit-35a-esc.html?cmid=eHZ3Y2tBWGYrQWM9&afid=TVZmU1BzYnlObnc9&ats=WDA0ZG1qK1ZCcW89), [Banggood](https://www.banggood.com/4X-DYS-Aria-BLHeli_32bit-35A-35amp-Brushless-ESC-3-6S-Dshot1200-Ready-Built-in-Current-Meter-Sensor-p-1187402.html?rmmds=myorder&cur_warehouse=CN&p=GQ230138854743201909&custlinkid=604595) | [dys](http://www.dys.hk/product/ARIA%2035A.html) |  |


## Props

- GetFPV: [MASTER AIRSCREW BN SERIES - 5X4.5 PROP SET X4 - BLACK](https://www.getfpv.com/master-airscrew-bn-series-5x4-5-prop-set-x4-black.html?cmid=eHZ3Y2tBWGYrQWM9&afid=TVZmU1BzYnlObnc9&ats=WDA0ZG1qK1ZCcW89)

## Receiver

The following receiver from [FrSky](/projects/fpv/fpv-glossar/#frsky) will be installed in the quad and should be used together with a FrSky [transmitter](/projects/fpv/fpv-glossar/#transmitter).
- GetFPV: [FRSKY R-XSR 2.4GHZ 16CH ACCST MICRO RECEIVER W/ S-BUS & CPPM](https://www.getfpv.com/frsky-r-xsr-2-4ghz-16ch-accst-micro-receiver-w-s-bus-cppm.html?cmid=eHZ3Y2tBWGYrQWM9&afid=TVZmU1BzYnlObnc9&ats=WDA0ZG1qK1ZCcW89)

