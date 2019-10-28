# FOSSASAT-2 Band Plan
This document specifies all communication channels used on FOSSASAT-2, as well as all applicable properties of all modulations.

## LoRa
* Functions: main control interface, direct repeater, store & forward, beacon (60s period system info downlink)
* Direction: uplink & downlink
* Frequency: 436.7 MHz
* Bandwidth: 125 kHz (dual sideband)
* Output power: 20 dBm
* Spreading factor: 11 (nominal data rate 300 bps)
* Coding rate: 4/5 (parity only)
* Sync word: **TBD**
* Preamble length: **TBD**

## GFSK
* Functions: backup control interface, direct repeater, store & forward, image downlink, full ADCS and system info downlink
* Direction: uplink & downlink
* Frequency: 436.7 MHz
* Bandwidth: 10 kHz (dual sideband)
* Output power: 20 dBm
* Bit rate: 9600 bps
* Gaussian data shaping: 0.5 (bandwidth-time product)
* Sync word: **TBD**
* Preamble length: **TBD**

## CW Beacon
* Functions: beacon (60s period callsign and battery voltage downlink)
* Direction: downlink only
* Frequency: 436.7 MHz
* Bandwidth: 0.1 kHz (dual sideband)
* Output power: 20 dBm
* Encoding: Morse Code
* Data rate: 12 words per minute
