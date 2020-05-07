# FOSSASAT-2 Test Specification
The purpose of this document is to define test cases which verify functionality of all subsystems of FOSSASAT-2.

## Test Setup
When possible, full satellite (or functionally equivalent circuit) shall be used for testing.

Ground station may be comprised of only ATmega328P and SX126x radio module. If possible, two ground station setups should be used, one with SX126x and the other with SX127x radio module. Station with SX127x module may only be left in passive observation mode, but must be able to receive all frames transmitted by the satellite, or the other ground station.

Satellite shall be running the debug code (FOSSASAT_DEBUG macro shall be defined). RESET_SYSTEM_INFO macro shall not be defined and all other configuration macros shall be enabled, unless the test specifically calls for a different setup. All communication responses must be verified by reception on ground station and compared with debug console output. In case the test calls for changing modulation parameters, this change shall be verified by reception, as well as by observation using SDR.

Test case shall be considered failed if any step of test case fails. All results shall be recorded into a copy of this document, with links to additional media where appropriate.

## Table of Contents
1. [Main Program](#main-program)
2. [Deployment](#deployment)
3. [Persistent Storage](#persistent-storage)
4. [Power Control](#power-control)
5. [Sensors](#sensors)
6. [Camera](#camera)
7. [ADCS](#adcs)
8. [Communication](#communication)

## Main Program
---
### MAINPROGT1 - Compilation & Upload
### Steps
1. Open flight software in Arduino IDE 1.8.11, set warning level to “All”, change board to some other than “Nucleo L452RE” (e.g. “Nucleo L476RG”) and compile software.
2. Change board to “Nucleo L452RE” and compile software with warning level set to “All”.
3. Upload software and check debug console.

### Expected Results
1. Compiling for a different board will force Arduino IDE to rebuild core (including libraries) and show all warnings. It is irrelevant whether this compilation fails or succeeds.
2. Software shall compile successfully. No warnings originating from FOSSASAT code shall be reported.
3. Software shall be uploaded successfully. Debug console shall not report any warnings or errors

### Actual Results
*  
*  
*  

### Verdict
*  
---
### MAINPROGT2 - Main loop timing
### Steps
1. Restart satellite with fully charged battery.
2. Keep satellite running for at least 20 minutes.

### Expected Results
1. All variables shall be set to default values. Satellite shall use default modem timing.
2. Satellite shall correctly switch between automated transmissions and listen modes on different modems.

### Actual Results
*  
*  
*  

### Verdict
*  

### Notes
* The following sequence is used as the default: CW beacon - GFSK full system info - LoRa basic system info - LoRa reception (40 seconds) - GFSK reception (20 seconds) - variable sleep length.

---
### MAINPROGT3 - Watchdog Timer
### Steps
1. Restart satellite.
2. Keep satellite running for at least 20 minutes.

### Expected Results
1. All variables shall be set to default values.
2. Satellite shall pet the watchdog every second. Watchdog shall not reset the satellite.

### Actual Results
*  
*  
*  

### Verdict
*  

---
### MAINPROGT4 - Integration Startup
### Steps
1. Upload flight software to satellite with wiped persistent storage and enabled RESET_SYSTEM_INFO macro.
2. Set RTC time.
3. Observe debug console output during first start.

### Expected Results
1. Satellite shall use default configuration of all parameters.
2. RTC time shall be set correctly.
3. All subsystems shall start correctly. Debug console shall not report any errors or warnings. All sensors shall output expected values.

### Actual Results
*  
*  
*  

### Verdict
*  

### Notes
* A separate sketch shall be provided to wipe persistent storage.

---

### MAINPROGT5 - Pin Mapping
### Steps
1. Check pin configuration in software matches flight hardware.

### Expected Results
1. All pins defined in software shall match the hardware.

### Actual Results
*  
*  
*  

### Verdict
*  

### Notes
* A separate sketch shall be provided to wipe persistent storage.

---

## Deployment

---

### DEPLOYT1 - Deployment Sequence
### Steps
1. Upload flight software to satellite with wiped persistent storage.
2. Check output during during first startup.
3. Reset satellite.
4. Repeat step 3 two more times.
5. Repeat step 3 three more times.
6. Force deployment sequence using CMD_DEPLOY.

### Expected Results
1. Satellite shall use default configuration of all parameters.
2. Integration debug shall be displayed during the first startup. Deployment sequence shall not be run.
3. Integration debug output shall not be displayed. Satellite shall wait until deployment voltage threshold is reached and then run the deployment sequence.
4. Deployment sequence (as described in previous expected result) shall be run each time.
5. Deployment sequence shall not be run again.
6. Deployment sequence shall be run. Satellite shall not wait to reach deployment voltage limit.

### Actual Results
*  
*  
*  

### Verdict
*  

### Notes
* A separate sketch shall be provided to wipe persistent storage.

---

## Persistent Storage

---

### PERSIST1 - System Counters
### Steps
1. Restart satellite with wiped persistent storage.
2. Reset satellite.
3. Run deployment sequence.

### Expected Results
1. All persistent variables shall be set to the default values.
2. Reset counter shall increment correctly.
3. Deployment counter shall increment correctly.

### Actual Results
*  
*  
*  

### Verdict
*  

---

### PERSIST2 - Communication Configuration
### Steps
1. Restart satellite with wiped persistent storage.
2. Set FLASH_TRANSMISSIONS_ENABLED to false.
3. Wait until satellite attempts automated transmissions.
4. Send some frames that require a response (e.g. CMD_PING).
5. Set FLASH_TRANSMISSIONS_ENABLED to true and repeat steps 3 - 4.
6. Change satellite callsign.
7. Change the length of LoRa receive window.
8. Change the length of FSK receive window.

### Expected Results
1. All persistent variables shall be set to the default values.
2. All transmissions from the satellite shall be disabled.
3. Automated transmissions shall not be sent.
4. Satellite shall not respond to the frames.
5. Satellite shall resume sending automated transmissions and respond to frames.
6. Callsign shall be updated successfully.
7. LoRa receive window length shall be updated correctly.
8. FSK receive window length shall be updated correctly.

### Actual Results
*  
*  
*  

### Verdict
*  

---

### PERSIST3 - RTC
### Steps
1. Restart satellite with wiped persistent storage.
2. Set RTC configuration using debug console.
3. Keep the satellite running for several hours.

### Expected Results
1. All persistent variables shall be set to the default values.
2. RTC configuration shall be set correctly.
3. RTC shall not drift, the timestamp in persistent storage shall be always correctly updated.

### Actual Results
*  
*  
*  

### Verdict
*  

---

### PERSIST4 - Communication Logging
### Steps
1. Restart satellite with wiped persistent storage.
2. Send some valid LoRa frames.
3. Repeat step 2 using invalid LoRa frames (incorrect callsign, malformed frame structure, missing CRC etc.).
4. Repeat steps 2 - 3 using FSK frames.

### Expected Results
1. All persistent variables shall be set to the default values.
2. Frames shall be correctly received and processed. Valid LoRa frame counter shall increment correctly.
3. Frames shall not be processed. Invalid LoRa frame counter shall increment correctly.
4. FSK frame counters shall increment correctly.

### Actual Results
*  
*  
*  

### Verdict
*  

---

### PERSIST5 - Limits
### Steps
1. Restart satellite with wiped persistent storage.
2. Set deployment voltage limit to non-default value.
3. Set heater battery voltage limit to non-default value.
4. Set CW beep battery voltage limit to non-default value.
5. Set low power voltage limit to non-default value.
6. Set battery heater temperature limit to non-default value.
7. Set MPPT temperature limit to non-default value.

### Expected Results
1. All persistent variables shall be set to the default values.
2. Deployment voltage limit shall be updated correctly.
3. Heater battery voltage limit shall be updated correctly.
4. CW beep battery voltage limit shall be updated correctly.
5. Low power voltage limit shall be updated correctly.
6. Battery temperature heater limit shall be updated correctly.
7. MPPT temperature limit shall be updated correctly.

### Actual Results
*  
*  
*  

### Verdict
*  

---

### PERSIST6 - Stats
### Steps
1. Restart satellite with wiped persistent storage.
2. Check all stats using CMD_GET_FULL_SYSTEM_INFO.
3. Keep satellite running for several hours. Change some measured variables (e.g. temperature sensors or light exposure) and check the stats.

### Expected Results
1. All persistent variables shall be set to the default values.
2. All stats shall have the default values.
3. Minimum, average and maximum of all variables shall update correctly.

### Actual Results
*  
*  
*  

### Verdict
*  

### Notes
* The following stats are kept: temperatures (Y panel, top, bottom, battery, secondary battery), voltages & currents (panels XA, XB, ZA, ZB, Y and MPPT), light exposure (Y panel and top), gyroscope/accelerometer/magnetometer (all axes).

---

### PERSIST7 - NMEA Logging
### Steps
1. Restart satellite with wiped persistent storage.
2. Send the command CMD_LOG_GPS.
3. Download the NMEA log.

### Expected Results
1. All persistent variables shall be set to the default values.
2. NMEA sentences shall be logged.
3. NMEA log shall be successfully downloaded.

### Actual Results
*  
*  
*  

### Verdict
*  

---

## Power Control

---

### PWCTRLT1 - Variable Sleep Interval Length
### Steps
1. Run satellite using battery with voltage level more than 4.05 V.
2. Repeat step 1 using battery with voltage level in range 4.0 - 4.05 V.
3. Repeat step 1 using battery with voltage level in range 3.9 - 4.0 V.
3. Repeat step 1 using battery with voltage level in range 3.8 - 3.9 V.
3. Repeat step 1 using battery with voltage level in range 3.7 - 3.8 V.
3. Repeat step 1 using battery with voltage level lower than 3.7 V.

### Expected Results
1. Sleep interval length shall be set to 20 seconds.
2. Sleep interval length shall be set to 35 seconds.
2. Sleep interval length shall be set to 100 seconds.
2. Sleep interval length shall be set to 160 seconds.
2. Sleep interval length shall be set to 180 seconds.
2. Sleep interval length shall be set to 240 seconds.

### Actual Results
*  
*  
*  

### Verdict
*  

---

### PWCTRLT2 - Low Power Mode
### Steps
1. Restart satellite with wiped persistent storage and enabled RESET_SYSTEM_INFO macro.
2. Use battery with voltage level above 3.8 V (the default low power mode voltage level).
3. Run the satellite for at least 10 minutes.
4. Use battery with voltage level below 3.8 V.
5. Repeat step 3.
6. Disable low power mode using CMD_SET_LOW_POWER_ENABLE and repeat steps 4 - 5.

### Expected Results
1. All persistent variables shall be set to the default values.
2. Low power mode will not be enabled.
3. Satellite shall run normally and transmit all automated frames. Low power mode shall not be active.
4. Low power mode will be enabled.
5. Satellite shall only send basic system info via GFSK and nothing via LoRa. Low power mode shall be active.
6. Low power mode shall not be active despite low battery voltage.

### Actual Results
*  
*  
*  

### Verdict
*  

---

### PWCTRLT3 - MPPT Control
### Steps
1. Restart satellite with wiped persistent storage and enabled RESET_SYSTEM_INFO macro.
2. Enable MPPT keep alive using CMD_SET_MPPT_MODE.
3. Lower temperature of primary battery sensor below 0 deg. C. and check MPPT state.
4. Repeat step 3 for the secondary battery temperature sensor.
5. Disable MPPT keep alive using CMD_SET_MPPT_MODE.
6. Repeat steps 3 - 4.
7. Disable temperature switch using CMD_SET_MPPT_MODE and repeat steps 3 - 4.

### Expected Results
1. All persistent variables shall be set to the default values.
2. MPPT keep alive shall be enabled.
3. Battery charging shall remain enabled despite low temperature.
4. Battery charging shall still be enabled.
5. MPPT keep alive shall be disabled.
6. Battery charging shall only be enabled if both temperature sensors are above 0 deg. C.
7. Battery temperatures shall have no effect on charging.

### Actual Results
*  
*  
*  

### Verdict
*  

---

### PWCTRLT4 - Battery Heater
### Steps
1. Restart satellite with wiped persistent storage and enabled RESET_SYSTEM_INFO macro.
2. Use battery with voltage level above 3.8 V (the default battery heater voltage limit).
3. Lower temperature of primary battery temperature sensor below 0 deg. C. and check heater state.
4. Repeat step 3 only with the secondary battery temperature sensor.
5. Lower both temperature sensors below 0 deg. C. and check heater state.
6. Use battery with voltage level below 3.8 V and repeat steps 3 - 5.

### Expected Results
1. All persistent variables shall be set to the default values.
2. Battery heater will be enabled.
3. Heater shall not be enabled (both temperature sensors must be below 0 deg. C.).
4. Heater shall remain disabled.
5. Heater shall be enabled.
6. Heater shall remain disabled at all times.

### Actual Results
*  
*  
*  

### Verdict
*  

---

## Sensors

---

### SENST1 - Inertial Measurement Unit
### Steps
1. Restart satellite with wiped persistent storage and enabled RESET_SYSTEM_INFO macro.
2. Check IMU readings during integration debug period.
3. Check IMU readings at several points after initialization (e.g. using CMD_GET_FULL_SYSTEM_INFO).

### Expected Results
1. IMU shall initialize correctly. Debug console shall display integration debug information.
2. IMU shall report correct values. Moving the satellite shall result in expected changes in reported values.
3. IMU shall always report correct values.

### Actual Results
*  
*  
*  

### Verdict
*  

---

### SENST2 - Temperature Sensors
### Steps
1. Restart satellite with wiped persistent storage and enabled RESET_SYSTEM_INFO macro.
2. Check all temperature readings during integration debug period.
3. Check all temperature readings at several points after initialization (e.g. using CMD_GET_FULL_SYSTEM_INFO).

### Expected Results
1. All temperature sensors shall initialize correctly. Debug console shall display integration debug information.
2. All temperature sensors shall report correct values. Changing environment temperature shall result in expected change in reported values.
3. All temperature sensors shall always report correct values.

### Actual Results
*  
*  
*  

### Verdict
*  

---

### SENST3 - Current Sensors
### Steps
1. Restart satellite with wiped persistent storage and enabled RESET_SYSTEM_INFO macro.
2. Check all current and voltage readings during integration debug period.
3. Check all current and voltage readings at several points after initialization (e.g. using CMD_GET_FULL_SYSTEM_INFO).

### Expected Results
1. All current sensors shall initialize correctly. Debug console shall display integration debug information.
2. All current sensors shall report correct values. Changing environment lighting shall result in expected change in reported values.
3. All current sensors shall always report correct values.

### Actual Results
*  
*  
*  

### Verdict
*  

---

### SENST4 - Light Sensors
### Steps
1. Restart satellite with wiped persistent storage and enabled RESET_SYSTEM_INFO macro.
2. Check all light readings during integration debug period.
3. Check all light readings at several points after initialization (e.g. using CMD_GET_FULL_SYSTEM_INFO).

### Expected Results
1. All light sensors shall initialize correctly. Debug console shall display integration debug information.
2. All light sensors shall report correct values. Changing environment lighting shall result in expected change in reported values.
3. All light sensors shall always report correct values.

### Actual Results
*  
*  
*  

### Verdict
*  

---

## Camera

### CAMT1 - Camera Power
### Steps
1. Restart satellite.
2. Leave the satellite running for at least 20 minutes. Do not send CMD_CAMERA_CAPTURE during this time.
3. Send CMD_CAMERA_CAPTURE frame.

### Expected Results
1. Both camera buses (I2C and SPI) shall initialize correctly. Camera shall only be powered on during the initialization.
2. At no point shall the camera power on (this shall be verified using external equipment, e.g. an oscilloscope).
3. Frame shall be received and processed correctly. Camera shall only power on prior to taking a picture and shall power off immediately afterwards.

### Actual Results
*  
*  
*  

### Verdict
*  

---

### CAMT2 - Camera Configuration
### Steps
1. Send CMD_CAMERA_CAPTURE with size parameter set to OV2640_160x120.
2. Repeat step 1 for every supported picture size.
3. Send CMD_CAMERA_CAPTURE with other parameters (light mode, saturation, contrast etc.) set to non-default values.

### Expected Results
1. Satellite shall correctly process the frame. Camera shall take a picture of the correct size. The picture length shall correspond to the configured picture size.
2. Camera shall always take picture of the correct size.
3. Camera shall always take picture with the proper configuration. No configuration shall cause unexpected behavior (e.g. picture size out of bounds or zero).

### Actual Results
*  
*  
*  

### Verdict
*  

---

## Communication

### COMMST1 - Ping-Pong Exchange
### Steps
1. Send valid CMD_PING frame from ground station using LoRa modem.
2. Repeat step 1 using FSK modem.
3. Add valid optional data to CMD_PING frame and send it using LoRa modem.
4. Repeat step 3 using CMD_PING frame with invalid optional data field (optDataLen field does not match number of bytes in optData field).

### Expected Results
1. Satellite shall respond with RESP_PONG frame using LoRa modem.
2. Satellite shall respond with RESP_PONG frame using FSK modem.
3. Satellite shall ignore optional data and respond with valid RESP_PONG frame.
4. Satellite shall ignore optional data and respond with valid RESP_PONG frame. Mismatch between optDataLen and actual length of optData field shall be reported in debug console.

### Actual Results
*  
*  
*  

### Verdict
*  

---

### COMMST2 - Relaying Messages
### Steps
1. Send valid CMD_RETRANSMIT frame from ground station using LoRa modem.
2. Repeat step 1 using FSK modem.
3. Send CMD_ RETRANSMIT frame with invalid optional data field (optDataLen field does not match number of bytes in optData field).


### Expected Results
1. Satellite shall respond with RESP_REPEATED_MESSAGE frame using LoRa modem.
2. Satellite shall respond with RESP_REPEATED_MESSAGE frame using FSK modem.
3. Satellite shall not respond with any frame. Mismatch between optDataLen and actual length of optData field shall be reported in debug console.

### Actual Results
*  
*  
*  

### Verdict
*  

---

### COMMST3 - Relaying Messages with Custom Configuration
### Steps
1. Send valid CMD_RETRANSMIT_CUSTOM frame from ground station using LoRa modem. Set modulation parameters to any non-default settings.
2. Wait for any transmission from the satellite.
3. Repeat steps 1 – 2 for settings with lowest possible data rate (bandwidth 7.8 kHz, SF12).
4. Repeat steps 1 – 2 for multiple different settings.
5. Send invalid CMD_RETRANSMIT_CUSTOM frame with one of the modulation parameters outside accepted range.
6. Repeat step 5 for all modulation parameters.

### Expected Results
1. Satellite shall respond with RESP_REPEATED_MESSAGE_CUSTOM frame using LoRa modem with the correct settings.
2. Further transmissions from the satellite shall use the default modem configuration.
3. Satellite shall always respond using correct settings. Satellite shall not reset when responding with very low data rates.
4. Satellite shall always respond using correct settings.
5. Satellite shall ignore this frame. Incorrect modulation parameter value shall be reported in debug console.
6. Satellite shall always ignore the invalid frame.

### Actual Results
*  
*  
*  

### Verdict
*  

---

### COMMST4 - Basic System Info Request
### Steps
1. Send valid CMD_TRANSMIT_SYSTEM_INFO frame from ground station using LoRa modem.
2. Repeat step 1 using FSK modem.
3. On the satellite, change as many variables contained in basic system info frame as possible and repeat step 1.

### Expected Results
1. Satellite shall respond with RESP_ SYSTEM_INFO frame using LoRa modem. All received values shall reflect the current satellite state.
2. Satellite shall respond with RESP_ SYSTEM_INFO frame using FSK modem.
3. All received values shall reflect the current satellite state.

### Actual Results
*  
*  
*  

### Verdict
*  

---

### COMMST5 - Packet Info Request
### Steps
1. Send valid CMD_GET_PACKET_INFO frame from ground station using LoRa modem.
2. Repeat step 1 using FSK modem.

### Expected Results
1. Satellite shall respond with RESP_PACKET_INFO frame using LoRa modem. The response shall contain correct values for SNR, RSSI and packet counters.
2. Satellite shall respond with RESP_PACKET_INFO frame using FSK modem.

### Actual Results
*  
*  
*  

### Verdict
*  

---

### COMMST6 - Statistics Request
### Steps
1. Send valid CMD_GET_STATISTICS frame from ground station using LoRa modem.
2. Repeat step 1 using FSK modem with flags for all statistics.

### Expected Results
1. Satellite shall reject the frame due to LoRa modem being active. No response shall be sent.
2. Satellite shall respond with RESP_STATISTICS frame using FSK modem. The response shall contain the expected set of statistics.

### Actual Results
*  
*  
*  

### Verdict
*  

---

### COMMST7 - Full System Info Request
### Steps
1. Send valid CMD_GET_FULL_SYSTEM_INFO frame from ground station using LoRa modem.
2. Repeat step 1 using FSK modem with flags for all statistics.

### Expected Results
1. Satellite shall reject the frame due to LoRa modem being active. No response shall be sent.
2. Satellite shall respond with RESP_FULL_SYSTEM_INFO frame using FSK modem. All received values shall reflect the current satellite state.

### Actual Results
*  
*  
*  

### Verdict
*  

---

### COMMST8 - Store & Forward
### Steps
1. Ensure store & forward storage is empty (e.g. by sending CMD_WIPE_EEPROM with the appropriate flag).
2. Add a message to the store & forward storage using some 32-bit ID.
3. Request the store & forward message using the ID from step 2.
4. Request store & forward message for a non-existent message ID.
5. Add another message using a new ID.
6. Request the store & forward message using the ID from previous step.
7. Add a different message using ID from step 5.
8. Repeat step 6.

### Expected Results
1. Store & forward storage shall not contain any messages.
2. New store & forward entry shall be created for the provided ID. Satellite shall respond with the 16-bit ID of the assigned slot.
3. Satellite shall respond with the correct forwarded message.
4. Satellite shall respond with forwarded message set to 0xFFFF (ID not found).
5. New store & forward entry shall be created for the provided ID. Satellite shall respond with the 16-bit ID of the assigned slot. Assigned slot ID shall be different than the one in step 2.
6. Satellite shall respond with the correct forwarded message.
7. Stored message for that ID shall be overwritten. Satellite shall respond with the 16-bit ID of the assigned slot. Assigned slot ID shall be the same as in step 5.
8. Satellite shall respond with the new forwarded message.

### Actual Results
*  
*  
*  

### Verdict
*  

---

### COMMST9 - Encryption and Password Protection
### Steps
1. Send any valid private command without optional data (e.g. CMD_DEPLOY).
2. Repeat step 1 using incorrect password.
3. Repeat step 1 using incorrect encryption key.
4. Repeat steps 1 – 3 using private command with optional data (e.g. CMD_SET_MPPT_MODE).

### Expected Results
1. Satellite shall perform the correct action (based on frame type).
2. Satellite shall successfully decode frame and detect incorrect password. Password mismatch shall be reported in the debug console.
3. Satellite shall fail in decoding the frame. Password mismatch shall be reported in the debug console.
4. Satellite shall correctly check password and report any mismatch.

### Actual Results
*  
*  
*  

### Verdict
*  

---

### COMMST10 - Restart Command
### Steps
1. Send valid CMD_RESTART frame from ground station using LoRa modem.
2. Repeat step 1 using FSK modem.

### Expected Results
1. Satellite shall restart.
2. Satellite shall always restart correctly.

### Actual Results
*  
*  
*  

### Verdict
*  

---

### COMMST11 - Persistent Storage Wipe
### Steps
1. Change all possible persistent storage variables (system properties, store and forward, images, NMEA log, stats).
2. Send CMD_WIPE_EEPROM with all flags set.

### Expected Results
1. All sections of persistent storage shall contain non-default values.
2. Satellite shall wipe all sections of persistent storage. Satellite shall not reset during time-intensive operations (e.g. wiping images).

### Actual Results
*  
*  
*  

### Verdict
*  

---

### COMMST12 - Transmission Control Configuration
### Steps
1. Send any frame that will make satellite respond (e.g. CMD_PING).
2. Send valid CMD_SET_TRANSMIT_ENABLE frame with first optional data byte set to 0 from ground station using LoRa modem.
3. Repeat step 1 using both modems.
4. Wait for at least 20 minutes.
5. Send valid CMD_SET_TRANSMIT_ENABLE frame with first optional data byte set to 1.
6. Repeat step 1 using both modems.
7. Wait for at least 20 minutes.
8. Repeat Steps 2 - 7 for the second optional data byte, which controls automated statistics transmissions.

### Expected Results
1. Satellite shall respond with the correct response frame.
2. Satellite shall no longer send any transmissions.
3. Satellite shall not send any response frames.
4. Satellite shall not send any automated frames.
5. Satellite shall resume sending transmissions.
6. Satellite shall always respond with the correct response frame.
7. Satellite shall send automated system info frames using LoRa and FSK.
8. Automated statistics transmissions shall be enabled by default, disabled after executing step 2 and enabled again in step 7.

### Actual Results
*  
*  
*  

### Verdict
*  

---

### COMMST13 - Callsign Configuration
### Steps
1. Wipe persistent system configuration using CMD_WIPE_EEPROM frame.
2. Send CMD_PING with callsign “FOSSASAT-2”.
3. Send valid CMD_SET_CALLSIGN frame with callsign set to some other string (e.g. “NEWCALLSIGN-4”).
4. Send CMD_PING with callsign “FOSSASAT-2”.
5. Send CMD_PING with the new callsign.
6. Send CMD_SET_CALLSIGN frame with new callsign longer than 32 bytes.

### Expected Results
1. Satellite callsign shall be set to default value (“FOSSASAT-2”).
2. Satellite shall respond using the default callsign.
3. Satellite shall set its callsign to the new value.
4. Satellite shall not respond to frame with incorrect callsign. Callsign mismatch shall be reported in debug console.
5. Satellite shall respond using the new callsign.
6. Satellite shall not update its callsign. Callsign length error shall be reported in debug console.

### Actual Results
*  
*  
*  

### Verdict
*  

---

### COMMST14 - Spreading Factor Configuration
### Steps
1. Wipe persistent system configuration using CMD_WIPE_EEPROM frame.
2. Send any frame that will make satellite respond (e.g. CMD_PING).
3. Send valid CMD_SET_SF_MODE frame with optional data byte set to 1.
4. Repeat step 2 with ground station using standard spreading factor.
5. Repeat step 2 with ground station using alternative spreading factor.
6. Send valid CMD_SET_SF_MODE frame with optional data byte set to 0.
7. Repeat steps 4 – 5.

### Expected Results
1. Spreading factor shall be set to standard value.
2. Satellite shall respond using standard spreading factor.
3. Satellite shall use alternative spreading factor.
4. Satellite shall not be able to receive transmissions with standard spreading factor.
5. Satellite shall correctly respond using alternative spreading factor.
6. Spreading factor shall be set back to standard mode.
7. Satellite shall only respond to transmissions with standard spreading factor.

### Actual Results
*  
*  
*  

### Verdict
*  

---

### COMMST15 - Receive Window Length Configuration
### Steps
1. Restart satellite with wiped persistent storage and enabled RESET_SYSTEM_INFO macro.
2. Set FSK window receive length to some non-default, non-zero value.
3. Leave satellite running and check the FSK receive window length.
4. Repeat steps 2 - 3 for LoRa receive window.
5. Set FSK receive window length to 0.
6. Set LoRa receive window length to 0.
7. Attempt to set FSK receive window length to 0 again.

### Expected Results
1. All persistent variables shall be set to the default values.
2. Receive window length for FSK modem shall be updated correctly.
3. Satellite shall use the new FSK receive window length.
4. Receive window for LoRa shall be correctly set and used.
5. Satellite shall allow zero FSK receive window length.
6. Satellite shall set LoRa window length to 0 and restore default FSK receive window length.
7. Satellite shall not allow FSK receive window to be set to 0 and always restore default FSK receive window length.

### Actual Results
*  
*  
*  

### Verdict
*  

---

### COMMST16 - Picture Download
### Steps
1. Take a picture using any picture parameter configuration.
2. Request length of the picture taken in previous step.
3. Request appropriate amount of picture bytes from slot used in step 1.
4. Using hex editor, save the received data as .jpeg file.
5. Repeat steps 1 - 3 for all possible picture sizes and some changed picture parameters (brightness, saturation etc.), always using new picture slot to save the picture.

### Expected Results
1. Picture shall be taken and saved to persistent storage.
2. Satellite shall respond with the picture length.
3. The request picture shall be transmitted as JPEG-encoded byte stream.
4. The received file shall be a valid JPEG picture.
5. The correct picture length and data bytes shall be transmitted for any possible picture size and parameters.

### Actual Results
*  
*  
*  

### Verdict
*  

### Notes
* Separate sketch shall provided for picture downlink.

---

### COMMST17 - RTC Configuration
### Steps
1. Restart satellite with wiped persistent storage and enabled RESET_SYSTEM_INFO macro.
2. Using CMD_SET_RTC, set RTC to some date and time.
3. Keep satellite running for at least 20 minutes.

### Expected Results
1. All persistent variables shall be set to the default values.
2. RTC configuration and persistent timestamp shall be updated.
3. RTC timestamp shall always correctly update. The time elapsed shall be correctly recored.

### Actual Results
*  
*  
*  

### Verdict
*  

---

### COMMST18 - Manual Magnetorquer Operation
### Steps

### Expected Results

### Actual Results
*  
*  
*  

### Verdict
*  

---

### COMMST19 - Packet Routing
### Steps
1. Send CMD_ROUTE frame targeting satellite with the callsign "FOSSASAT-1B" and payload CMD_PING.
2. Repeat step 1 using payload frame with optional data (e.g. CMD_RETRANSMIT).

### Expected Results
1. Satellite shall retransmit the payload frame (CMD_PING) with the target callsign ("FOSSASAT-1B"). The target satellite shall respond to this frame.
2. Satellite shall correctly retransmit the routed frame. The target satellite shall respond to the payload frame.

### Actual Results
*  
*  
*  

### Verdict
*  

### Notes
* This test requires a FOSSASAT-1B setup to be withing range.

---

### COMMST20 - Morse Code Beacon
### Steps
1. Restart satellite with wiped persistent storage and enabled RESET_SYSTEM_INFO macro.
2. Keep satellite running with battery above 3.8 V (default Morse beacon limit) for at least 10 minutes.
3. Repeat step 2 using battery below 3.8 V (or with adjusted Morse beacon limit).

### Expected Results
1. All persistent variables shall be set to the default values.
2. Satellite shall send automated Morse code beacon consisting of callsign and battery voltage, encoded as letter (A = 3.2 V, B = 3.25 V etc.).
3. Satellite shall only send 3 500 ms long "beeps" at the carrier frequency, instead of the Morse code beacon.

### Actual Results
*  
*  
*  

### Verdict
*  

---

### COMMST21 - Response Delay
### Steps
1. Send any frame that will make the satellite respond (e.g. CMD_PING) using LoRa modem.
2. Repeat step 1 for at least 3 other frames.
3. Repeat steps 1 - 3 for FSK modem.

### Expected Results
1. Satellite shall respond with the appropriate frame type. There shall be at least 1 second delay between end of command transmission and start of response transmission.
2. Satellite shall always wait one second.
3. Satellite shall wait one second regardless of currently active modem.

### Actual Results
*  
*  
*  

### Verdict
*  

---

### COMMST22 - Frame acknowledgement
### Steps
1. Send valid CMD_PING frame from ground station using LoRa modem.
2. Repeat step 1 using FSK modem.
3. Repeat steps 1 - 2 for every possible frame type.

### Expected Results
1. Satellite shall acknowledge reception of the frame by transmitting RESP_ACKNOWLEDGE frame with appropriate function ID.
2. Satellite shall acknowledge frame reception using FSK modem.
3. Satellite shall always acknowledge frame reception using correct modem.

### Actual Results
*  
*  
*  

### Verdict
*  

---

### COMMST23 - Write to external flash
### Steps
1. Read one of the flash pages using CMD_GET_FLASH_CONTENTS (e.g. the system info page).
2. Change the contents of the flash page using CMD_SET_FLASH_CONTENTS.

### Expected Results
1. Flash shall contain the expected data.
2. Flash page shall be updated on the correct address. The rest of the page shall remain unchanged.

### Actual Results
*  
*  
*  

### Verdict
*  

---

## ADCS

---
