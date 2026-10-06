---
layout: single
title:  "Taranis X9D Plus SE 2019 - PID Tuning Lua Script"
permalink: /projects/fpv/pid-lua-script
excerpt: "Setting up PID Tuning via a Lua Script on the Taranis X9D Plus SE 2019."
date: 2020-04-18 09:00:35 +0100
last_modified_at: 2026-10-06
categories: [fpv, quad]
tags: [fpv, quad, race, drone, betaflight, radio, tuning, pid, lua, script]
comments: true
use_math: true
toc: true
classes: wide
# toc_label: "Unscented Kalman Filter"
header:
  teaser: /assets/collections/fpv/taranis/taranis.jpg
  overlay_image: /assets/collections/fpv/taranis/bootloader.jpg
  overlay_filter: 0.5
sidebar:
  nav: "fpv"
---

The [Betaflight TX Lua scripts](https://github.com/betaflight/betaflight-tx-lua-scripts) run on the [radio](/projects/fpv/glossar#radio)
and change Betaflight settings such as PIDs, [rates](/projects/fpv/rate-expo-settings), filters and VTX channels directly from the [Taranis](/projects/fpv/taranis),
without a computer. This makes it possible to adjust the tune between two flights at the field.
The scripts talk to the [flight controller](/projects/fpv/glossar#flight-controller) over the [telemetry](/projects/fpv/glossar#telemetry) link of the receiver.

## Requirements

- **Radio firmware**: [OpenTX](/projects/fpv/glossar#opentx) 2.3.12 or newer, or [EdgeTX](/projects/fpv/glossar#edgetx) 2.4.0 or newer.
- **Receiver with telemetry**: FrSky receivers with SmartPort or [FPort](/projects/fpv/glossar#fport) such as the [R-XSR](/projects/fpv/r-xsr) of this build,
  TBS Crossfire, [ExpressLRS](/projects/fpv/glossar#expresslrs) or ImmersionRC Ghost. Update the receiver to its latest firmware to avoid known telemetry bugs.
- **Telemetry enabled in Betaflight**: in Betaflight 4.1 this is the Telemetry feature in the Configuration tab, in newer versions it is in the Receiver tab.

<figure>
    <a href="/assets/collections/fpv/betaflight/betaflight-config-receiver.png"><img src="/assets/collections/fpv/betaflight/betaflight-config-receiver.png"></a>
    <figcaption>Betaflight 4.1.1 with the FrSky FPort receiver and the Telemetry feature enabled in the Configuration tab.</figcaption>
</figure>

## Installation

1. Download the zip file of the latest version from the [releases page](https://github.com/betaflight/betaflight-tx-lua-scripts/releases) (1.8.0 from January 2026 at the time of writing).
2. Connect the radio's SD card to the computer, either with a card reader or by connecting the radio over USB and selecting its SD card (USB storage).
3. Unzip the file and copy the contents of its `obj` folder to the SD card, so that the `SCRIPTS` folder merges with the one on the card.
   Don't copy the contents of the repository itself.
4. Check that the file `bf.lua` is in the folder `/SCRIPTS/TOOLS` on the SD card.

## Usage

Open the TOOLS page of the radio settings (on the Taranis X9D Plus with a long press on [MENU]), select "Betaflight setup" and press [ENTER].
The first start after installing or updating compiles the scripts and returns to the TOOLS page; start it again to use it.

The Taranis has a monochrome display, so the script uses these controls:

| Button | Function |
|:--|:--|
| [+] / [-] / rotary encoder | Move between fields |
| [PAGE] | Next page, long press for the previous page |
| [ENTER] | Edit the selected field, long press for the function menu |
| [EXIT] | Go back or close the script |

Changes take effect only when they are saved: long press [ENTER] to open the function menu and select "save page".
Leaving a page without saving discards its changes. Land and disarm before saving, then fly again to check the change,
and change only a few values at a time.

The function menu also reloads the VTX tables. Betaflight manages bands and channels of SmartAudio and Tramp video transmitters in VTX tables since version 4.1,
and the script downloads the table of each model the first time it connects. After changing the VTX table in Betaflight, reload it in the script.

## References

- [Betaflight TX Lua scripts on GitHub](https://github.com/betaflight/betaflight-tx-lua-scripts) (installation, controls and supported radios)
- [Releases](https://github.com/betaflight/betaflight-tx-lua-scripts/releases)
