---
layout: single
title:  "Charger and Testers"
permalink: /projects/fpv/charger
excerpt: "Chargers and testers for LiPo batteries."
categories: [fpv, quad]
tags: [fpv, quad, race, drone, charger, battery, lipo]
date: 2026-10-06 09:00:00 +0200
comments: true
use_math: true
toc: true
classes: wide
header:
  teaser: /assets/collections/fpv/charger/charger-header.jpg
  overlay_image: /assets/collections/fpv/charger/charger-header.jpg
  overlay_filter: 0.5
sidebar:
  nav: "fpv"
---

[LiPo batteries](/projects/fpv/battery) need a charger that is made for them.
It charges each cell to exactly 4.2 V and keeps the cells of a pack at the same voltage, which is called balancing.
A battery checker measures the cells before and after a flight.

## Balance Charging

The charger is connected to both leads of the battery: the main lead carries the charge current and the balance lead connects to every cell.
Select the battery type (LiPo), the cell count and the charge current. 1C is the safe default, which is 1.3 A for a 1300 mAh battery.

LiPos are charged in two phases. First the charger keeps the current constant until the cells reach 4.2 V,
then it keeps the voltage constant while the current decreases until the battery is full (CC/CV charging).
During charging it balances the cells through the balance lead.
Without balancing, the cells of a pack drift apart over time, so one cell would be overcharged while another one is still not full.

Most chargers also have a storage mode, which charges or discharges a battery to 3.8 V per cell for storage.

## The EV-Peak CQ3

This build uses the [EV-Peak CQ3](https://www.ev-peak.com/product/cq3/), a charger with four independent outputs that can charge four batteries at the same time.

<figure>
    <a href="/assets/collections/fpv/components/ev-peak-cq3-4x-100w-lead_2.jpg"><img src="/assets/collections/fpv/components/ev-peak-cq3-4x-100w-lead_2.jpg" alt="EV-Peak CQ3 charger"></a>
    <figcaption>EV-Peak CQ3 multi charger.</figcaption>
</figure>

| Property | Value |
|:--|:--|
| Input | 110/220 V AC or 11 to 18 V DC |
| Outputs | 4 independent channels, each up to 100 W and 10 A |
| Battery types | LiPo, LiHV, LiFe and Li-ion with 1 to 6 cells, NiMH and NiCd with 1 to 15 cells, lead acid from 2 to 24 V |
| Memory | 20 saved charge programs |

The power of a channel limits the charge current. For a fully charged 3S battery at 12.6 V, 100 W allow up to
$100\,\text{W} / 12.6\,\text{V} \approx 7.9\,\text{A}$, far more than the 1.3 A that a 1300 mAh battery needs at 1C.

## Battery Checker

A battery checker such as the EV-Peak Cellmeter-7 plugs into the balance lead and shows the voltage of each cell,
the total voltage and an estimate of the remaining capacity. It works with LiPo, LiFe, Li-ion, NiMH and NiCd batteries with up to 7 cells.
Check the batteries before a flight, after landing and before storing them, and look for cells that drift apart.

<figure class="half">
    <a href="/assets/collections/fpv/charger/battery-checker.jpg"><img src="/assets/collections/fpv/charger/battery-checker.jpg" alt="Digital battery capacity checker"></a>
    <a href="/assets/collections/fpv/charger/battery-checker-3s.jpg"><img src="/assets/collections/fpv/charger/battery-checker-3s.jpg" alt="Battery checker connected to a 3S battery showing 12.57 V"></a>
    <figcaption>Battery checker of this build. Connected to a freshly charged 3S battery it shows 12.57 V and 99 %, which is 4.19 V per cell.</figcaption>
</figure>

## Safety

- Never leave a charging battery unattended.
- Charge on a non-flammable surface or in a LiPo bag.
- Don't charge batteries that are hot, puffed or damaged, and let a battery cool down after a flight before charging it.
- Double check the battery type and cell count before starting the charger.
