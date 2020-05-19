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
* CRC: enabled
* Sync word: 0x12
* Preamble length: **TBD**
* Listen mode length: **TBD**

## GFSK
* Functions: backup control interface, direct repeater, store & forward, image downlink, full ADCS and system info downlink
* Direction: uplink & downlink
* Frequency: 436.7 MHz
* Bandwidth: 10 kHz (dual sideband)
* Output power: 20 dBm
* Bit rate: 9600 bps
* Gaussian data shaping: 0.5 (bandwidth-time product)
* CRC: 2 bytes, initial 0x1D0F, polynomial 0x1021, inverted
* Sync word: 0x1212
* Preamble length: **TBD**
* Listen mode length: **TBD**

## CW Beacon
* Functions: beacon (60s period callsign and battery voltage downlink)
* Direction: downlink only
* Frequency: 436.7 MHz
* Bandwidth: 0.1 kHz (dual sideband)
* Output power: 20 dBm
* Encoding: ITU Morse Code (as per [M.1677](https://www.itu.int/rec/R-REC-M.1677-1-200910-I/))
* Data rate: TBD
* Preamble: Starting signal (-.-.-), repeated 3 times
