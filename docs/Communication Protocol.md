# FOSSASAT-2 Communication Protocol
The purpose of this document is to provide overview of the communication system implemented for FOSSASAT-2. The following is merely a list of all implemented communication commands and responses - for further information on the communication protocol itself, please refer to [FOSSASAT-1 Communication Guide](https://github.com/FOSSASystems/FOSSASAT-1/blob/master/FOSSA%20Documents/FOSSASAT-1%20Comms%20Guide.pdf).

---
# Public Commands
### CMD_PING
- Optional data length: 0
- Optional data: none
- Response: [RESP_PONG](#RESP_PONG)
- Description: Simple ping/pong exchange.

### CMD_RETRANSMIT
- Optional data length: 0 - 32
- Optional data:
  - 0 - N: message to be repeated
- Response: [RESP_REPEATED_MESSAGE](#RESP_REPEATED_MESSAGE)
- Description: Satellite will retransmit the optional data provided in this command using the current radio configuration.

### CMD_RETRANSMIT_CUSTOM
- Optional data length: 8 - 39
- Optional data:
  - 0: bandwidth (0x00 for 7.8 kHz, 0x07 for 125 kHz)
  - 1: spreading factor (0x00 for SF5, 0x07 for SF12)
  - 2: coding rate (0x05 for 4/5, 0x08 for 4/8)
  - 3 - 4: preamble length in symbols, LSB first
  - 5: CRC enabled (0x01) or disabled (0x00)
  - 6: output power in dBm (signed 8-bit integer, -17 to 22)
  - 7 - N: message to be repeated
- Response: [RESP_REPEATED_MESSAGE_CUSTOM](#RESP_REPEATED_MESSAGE_CUSTOM)
- Description: Satellite will retransmit the optional data provided in this command using the provided radio configuration.

### CMD_TRANSMIT_SYSTEM_INFO
- Optional data length: 0
- Optional data: none
- Response: [RESP_SYSTEM_INFO](#RESP_SYSTEM_INFO)
- Description: Request basic system information.

### CMD_GET_PACKET_INFO
- Optional data length: 0
- Optional data: none
- Response: [RESP_PACKET_INFO](#RESP_PACKET_INFO)
- Description: Request packet/communication information and statistics.

### CMD_GET_STATISTICS
- Optional data length: 1
- Optional data:
  - 0: flags of stats to include. All flags may be active at once.
    - 0x01: temperatures
    - 0x02: currents
    - 0x04: voltages
    - 0x08: light sensors
- Response: [RESP_STATISTICS](#RESP_STATISTICS)
- Description: Request satellite statistics according to the set flags. Only available in FSK mode.

### CMD_GET_FULL_SYSTEM_INFO
- Optional data length: 0
- Optional data: none
- Response: [RESP_FULL_SYSTEM_INFO](#RESP_FULL_SYSTEM_INFO)
- Description: Request basic system information. Only available in FSK mode.

### CMD_STORE_AND_FORWARD_ADD

### CMD_STORE_AND_FORWARD_REQUEST

---
# Private Commands
The following commands are encrypted using AES-128 and must be correctly decrypted and verified in order to be executed.

### CMD_DEPLOY
- Optional data length: 0
- Optional data: none
- Response: [RESP_DEPLOYMENT_STATE](#RESP_DEPLOYMENT_STATE)
- Description: Attempt to deploy antenna.

### CMD_RESTART
- Optional data length: 0
- Optional data: none
- Response: none
- Description: Restart the satellite.

### CMD_WIPE_EEPROM
- Optional data length: 1
- Optional data:
  - 0: flags of persistent storage sections to include. All flags may be active at once.
    - 0x01: system info and configuration (all settings in persistent storage will be reset to the default values)
    - 0x02: statistics
    - 0x04: store and forward frames
    - 0x08: NMEA log
    - 0x10: image storage (execution of this command will take several minutes)
- Response: none
- Description: Wipes persistent storages.

### CMD_SET_TRANSMIT_ENABLE
- Optional data length: 1
- Optional data:
  - 0: transmit enable (0x01) or disable (0x00)
- Response: none
- Description: Can be used to completely disable all transmissions from satellite.

### CMD_SET_CALLSIGN
- Optional data length: 0 - 32
- Optional data:
  - 0 - N: new callsign to be set, must be a null-terminated string of printable ASCII characters.
- Response: none
- Description: Changes satellite callsign.

### CMD_SET_SF_MODE
- Optional data length: 1
- Optional data:
  - 0: standard (0x00) or alternative (0x01) spreading factor
- Response: none
- Description: Can be used to switch LoRa spreading factor between two pre-configured values.

### CMD_SET_MPPT_MODE
- Optional data length: 2
- Optional data:
  - 0: MPPT temperature switch enabled (0x01) or disabled (0x00)
  - 1: MPPT keep alive enabled (0x01) or disabled (0x00)
- Response: none
- Description: Changes MPPT temperature switch (disables charging in low temperatures) or MPPT keep alive (forces charging at all times).

### CMD_SET_LOW_POWER_ENABLE
- Optional data length: 1
- Optional data:
  - 0: low power mode enabled (0x01) or disabled (0x00)
- Response: none
- Description: Changes low power mode enabled flag - when disabled, satellite cannot enter low power mode.

### CMD_SET_RECEIVE_WINDOWS
- Optional data length: 2
- Optional data:
  - 0: FSK receive window length in seconds
  - 1: LoRa receive window length in seconds
- Response: none
- Description: Changes the length of FSK and LoRa receive windows. Either one can be set to 0 to skip that receive window.

### CMD_CAMERA_CAPTURE
- Optional data length: 4
- Optional data:
 - 0: picture slot to be used, 0 - 127
 - 1: lower 4 bits light mode, upper 4 bits picture size
 - 2: lower 4 bits brightness, upper 4 bits saturation
 - 3: lower 4 bits special filter, upper 4 bits contrast
- Response: [RESP_CAMERA_STATE](#RESP_CAMERA_STATE)
- Description: Request camera to take a picture with provided settings and save it in given picture slot in flash storage.

### CMD_SET_POWER_LIMITS
- Optional data length: 17
- Optional data:
  - 0 - 1: deployment voltage limit in mV, signed 16-bit integer, LSB first
  - 2 - 3: heater voltage limit in mV, signed 16-bit integer, LSB first
  - 4 - 5: CW beep voltage limit in mV, signed 16-bit integer, LSB first
  - 6 - 7: low power voltage limit in mV, signed 16-bit integer, LSB first
  - 8 - 11: heater temperature limit in deg. C, float, LSB first
  - 12 - 15: MPPT switch temperature limit in deg. C, float, LSB first
  - 16: heater duty cycle, 0 - 255
- Response: none
- Description: Changes voltage and temperature limits, and heater duty cycle.

### CMD_SET_RTC
- Optional data length: 7
- Optional data:
  - 0: year, offset from 2000
  - 1: month
  - 2: day
  - 3: day of week, 1 for Monday
  - 4: hours
  - 5: minutes
  - 6: seconds
- Response: none
- Description: Changes current RTC time.

### CMD_RECORD_IMU
- Optional data length: 4
- Optional data:
  - 0: number of samples (10 is maximum)
  - 1 - 2: sampling period in ms, unsigned 16-bit integer, LSB first
  - 3: flags which specify the device to use, only one may be active at a time:
    - 0x01: gyroscope
    - 0x02: accelerometer
    - 0x04: magnetometer
- Response: [RESP_RECORDED_IMU](#RESP_RECORDED_IMU)
- Description: Records selected IMU device. Logging will be stopped if battery voltage drops below low power mode level.

### CMD_RUN_ADCS
- Optional data length: 7
- Optional data:
  - 0: X axis H-bridge magnitude, signed 8-bit integer, -63 to 63
  - 1: Y axis H-bridge magnitude, signed 8-bit integer, -63 to 63
  - 2: Z axis H-bridge magnitude, signed 8-bit integer, -63 to 63
  - 3 - 6: maneuver duration in ms, unsigned 32-bit integer, LSB first
- Response: [RESP_ADCS_RESULT](#RESP_ADCS_RESULT)
- Description: Performs ADCS maneuver. May be terminated prematurely if one or more H-bridges return some error, or if battery voltage drops below low power mode level.

### CMD_LOG_GPS
- Optional data length: 4
- Optional data:
  - 0 - 3: GPS logging duration in ms, unsigned 32-bit integer, LSB first
- Response: [RESP_GPS_LOG_LENGTH](#RESP_GPS_LOG_LENGTH)
- Description: Records GPS output. Logging will be stopped if battery voltage drops below low power mode level.

### CMD_GET_GPS_LOG
- Optional data length: 4
- Optional data:
  - 0 - 3: offset from GPS log start, unsigned 32-bit integer, LSB first
- Response: [RESP_GPS_LOG](#RESP_GPS_LOG)
- Description: Request downlink of logged GPS data. Only available in FSK mode.

### CMD_GET_FLASH_CONTENTS
- Optional data length: 5
- Optional data:
  - 0 - 3: flash address to read, unsigned 32-bit integer, LSB first
  - 4: number of bytes to read, 0 - 128.
- Response: [RESP_FLASH_CONTENTS](#RESP_FLASH_CONTENTS)
- Description: Request downlink of data saved in external flash.

### CMD_GET_PICTURE_LENGTH
- Optional data length: 1
- Optional data:
  - 0:picture slot for which the length should be read
- Response: [RESP_CAMERA_PICTURE_LENGTH](#RESP_CAMERA_PICTURE_LENGTH)
- Description: Reads length of picture in provided slot.

### CMD_GET_PICTURE_BURST
- Optional data length: 3
- Optional data:
  - 0: picture slot to read
  - 1 - 2: picture packet ID at which should the reading start, unsigned 16-bit integer, LSB first
- Response: [RESP_CAMERA_PICTURE](#RESP_CAMERA_PICTURE)
- Description: Requests burst downlink of picture from provided slot.

---
# Responses

### RESP_PONG
- Optional data length: 0
- Optional data: none

### RESP_REPEATED_MESSAGE
- Optional data length: 0 - 32
- Optional data:
  - 0 - N: repeated message

### RESP_REPEATED_MESSAGE_CUSTOM
- Optional data length: 0 - 32
- Optional data:
  - 0 - N: repeated message

### RESP_SYSTEM_INFO
- Optional data length: 19
- Optional data:
  - 0: MPPT output voltage * 20 mV, unsigned 8-bit integer
  - 1 - 2: MPPT output current * 10 uA, signed 16-bit integer
  - 3 - 6: onboard time as Unix timestamp, unsigned 32-bit integer
  - 7: power configuration
    - bit 0: transmissions enabled
    - bit 1: low power mode enabled
    - bits 2 - 4: currently active low power mode
    - bit 5: MPPT temperature switch enabled
    - bit 6: MPPT keep alive enabled
  - 8 - 9: reset counter, unsigned 16-bit integer
  - 10: solar panel XA voltage * 20 mV, unsigned 8-bit integer
  - 11: solar panel XB voltage * 20 mV, unsigned 8-bit integer
  - 12: solar panel ZA voltage * 20 mV, unsigned 8-bit integer
  - 13: solar panel ZB voltage * 20 mV, unsigned 8-bit integer
  - 14: solar panel Y voltage * 20 mV, unsigned 8-bit integer
  - 15 - 16: battery temperature * 0.01 deg. C, signed 16-bit integer
  - 17 - 18: OBC board temperature * 0.01 deg. C, signed 16-bit integer

### RESP_PACKET_INFO
- Optional data length: 10
- Optional data:
  - 0: SNR of the last received packet * 4 dB, signed 8-bit integer
  - 1: RSSI of the last received packet * -2 dBm, unsigned 8-bit integer
  - 2 - 3: number of received valid LoRa frames, unsigned 16-bit integer
  - 4 - 5: number of received invalid LoRa frames, unsigned 16-bit integer
  - 6 - 7: number of received valid FSK frames, unsigned 16-bit integer
  - 8 - 9: number of received invalid FSK frames, unsigned 16-bit integer

### RESP_STATISTICS
- Optional data length: 1 - 109
- Optional data:
  - 0: flags of statistics included in the response
  - 30 bytes for temperatures * 0.01 deg. C, signed 16-bit integers
  - 36 bytes for currents * 10 uA, signed 16-bit integers
  - 18 bytes for voltages * 20 mV, unsigned 8-bit integers
  - 24 bytes for light sensors, floats

### RESP_FULL_SYSTEM_INFO
- Optional data length: 46
- Optional data:
  - 0: MPPT output voltage * 20 mV, unsigned 8-bit integer
  - 1 - 2: MPPT output current * 10 uA, signed 16-bit integer
  - 3 - 6: onboard time as Unix timestamp, unsigned 32-bit integer
  - 7: power configuration
    - bit 0: transmissions enabled
    - bit 1: low power mode enabled
    - bits 2 - 4: currently active low power mode
    - bit 5: MPPT temperature switch enabled
    - bit 6: MPPT keep alive enabled
  - 8 - 9: reset counter, unsigned 16-bit integer
  - 10: solar panel XA voltage * 20 mV, unsigned 8-bit integer
  - 11 - 12: solar panel XA current * 10 uA, signed 16-bit integer
  - 13: solar panel XB voltage * 20 mV, unsigned 8-bit integer
  - 14 - 15: solar panel XB current * 10 uA, signed 16-bit integer
  - 16: solar panel ZA voltage * 20 mV, unsigned 8-bit integer
  - 17 - 18: solar panel ZA current * 10 uA, signed 16-bit integer
  - 19: solar panel ZB voltage * 20 mV, unsigned 8-bit integer
  - 20 - 21: solar panel ZB current * 10 uA, signed 16-bit integer
  - 22: solar panel Y voltage * 20 mV, unsigned 8-bit integer
  - 23 - 24: solar panel Y current * 10 uA, signed 16-bit integer
  - 25 - 26: Y panel temperature * 0.01 deg. C, signed 16-bit integer
  - 27 - 28: OBC board temperature * 0.01 deg. C, signed 16-bit integer
  - 29 - 30: bottom board temperature * 0.01 deg. C, signed 16-bit integer
  - 31 - 32: battery temperature * 0.01 deg. C, signed 16-bit integer
  - 33 - 34: second battery temperature * 0.01 deg. C, signed 16-bit integer
  - 35 - 38: Y panel light sensor * 1 lux, float
  - 39 - 42: top board light sensor * 1 lux, float
  - 43: last X axis H-bridge fault, unsigned 8-bit integer
  - 44: last Y axis H-bridge fault, unsigned 8-bit integer
  - 45: last Z axis H-bridge fault, unsigned 8-bit integer

### RESP_FORWARDED_MESSAGE

### RESP_DEPLOYMENT_STATE
- Optional data length: 1
- Optional data:
  - 0: deployment counter, unsigned 8-bit integer

### RESP_CAMERA_STATE
- Optional data length: 4
- Optional data:
  - 0 - 3: captured image length or camera error state, unsigned 32-bit integer

### RESP_RECORDED_IMU
- Optional data length: 12 * number of samples
- Optional data:
  - first 4 bytes: X axis measurements, float
  - second 4 bytes: Y axis measurements, float
  - third 4 bytes: Z axis measurements, float

### RESP_ADCS_RESULT
- Optional data length: 7
- Optional data:
  - 0: X axis H-bridge fault
  - 1: Y axis H-bridge fault
  - 2: Z axis H-bridge fault
  - 3 - 6: elapsed duration of maneuver in ms, unsigned 32-bit integer

### RESP_GPS_LOG
- Optional data length: 4 - 128
- Optional data:
  - 0 - 3: GPS log entry timestamps as offset since measurement start, unsigned 32-bit integer
  - 4 - N: GPS log entry

### RESP_GPS_LOG_LENGTH
- Optional data length: 4
- Optional data:
  - 0 - 3: GPS log length

### RESP_FLASH_CONTENTS
- Optional data length: 0 - 128
- Optional data:
  - 0 - N: flash data

### RESP_CAMERA_PICTURE
- Optional data length: 6 - 130
- Optional data:
  - 0 - 1: picture packet ID, unsigned 16-bit integer
  - 2 - N: picture data (4 null bytes sent if the requested slot has no image)

### RESP_CAMERA_PICTURE_LENGTH
- Optional data length: 4
- Optional data:
  - 0 - 3: length of image in requested slot
