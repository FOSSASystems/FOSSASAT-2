# FOSSASAT-2 Software Functional Specification
The purpose of this document is to keep track of all functional specifications, based on [Software Requirements](https://github.com/FOSSASystems/FOSSASAT-2/blob/master/docs/Software%20Requirements.md). Any Specification(s) that fulfill(s) Software Requirement(s) SHOULD list reference to those Requirement(s).

## Deployment Sequence
Step 1: Once Deployed, The killswitch connects and starts  discharging to cpu or/and charging battery if in sunlight.
Step 2: Check EEPROM if Deploy = 0, if Deploy = 1 go to System Start
Step 3: 1m debug silent (Serial output via STLINK)
Step 4: Satellite waits 25m in Sleep 
Step 5: Check voltage, if under 3.7v: wait until you are charged for a maximum of 12 hours and then deploy. If above 3.7v: Deploy.
Step 6: Send Mosfet deploymen signal 
Step 7: Set EEPROM Deploy = 1
Step 8: System tx start

## Battery Charging and Heating


## Transmissions and Reception 


## Store & Forward Repeater


## Image Downlink


## System Sleep & Power Saving System


