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
- Optional data length: 4 - 32
- Optional data:
  - 0 - 3: sender ID, unsigned 32-bit integer, LSB first
  - 4 - N: message to be repeated
- Response: [RESP_REPEATED_MESSAGE](#RESP_REPEATED_MESSAGE)
- Description: Satellite will retransmit the optional data provided in this command using the current radio configuration.

### CMD_RETRANSMIT_CUSTOM
- Optional data length: 11 - 43
- Optional data:
  - 0: bandwidth (0x00 for 7.8 kHz, 0x07 for 125 kHz)
  - 1: spreading factor (0x00 for SF5, 0x07 for SF12)
  - 2: coding rate (0x05 for 4/5, 0x08 for 4/8)
  - 3 - 4: preamble length in symbols, LSB first
  - 5: CRC enabled (0x01) or disabled (0x00)
  - 6: output power in dBm (signed 8-bit integer, -17 to 22)
  - 7 - 10: sender ID, unsigned 32-bit integer, LSB first
  - 11 - N: message to be repeated
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
    - 0x10: IMU
- Response: [RESP_STATISTICS](#RESP_STATISTICS)
- Description: Request satellite statistics according to the set flags. Only available in FSK mode.

### CMD_GET_FULL_SYSTEM_INFO
- Optional data length: 0
- Optional data: none
- Response: [RESP_FULL_SYSTEM_INFO](#RESP_FULL_SYSTEM_INFO)
- Description: Request full system information. Only available in FSK mode.

### CMD_STORE_AND_FORWARD_ADD
- Optional data length: 4 - 32
- Optional data:
  - 0 - 3: ID of the message, unsigned 32-bit integer, LSB first
  - 4 - N: message to be stored
- Response: [RESP_STORE_AND_FORWARD_ASSIGNED_SLOT](#RESP_STORE_AND_FORWARD_ASSIGNED_SLOT)
- Description: Adds message to store and forward storage.

### CMD_STORE_AND_FORWARD_REQUEST
- Optional data length: 4
- Optional data:
  - 0 - 3: ID of the message to be sent, unsigned 32-bit integer, LSB first
- Response: [RESP_FORWARDED_MESSAGE](#RESP_FORWARDED_MESSAGE)
- Description:  Satellite will retransmit the message stored previously with supplied ID.

### CMD_REQUEST_PUBLIC_PICTURE
- Optional data length: 3
- Optional data:
  - 0: picture slot to read
  - 1 - 2: picture packet ID at which should the reading start, unsigned 16-bit integer, LSB first
- Response: [RESP_PUBLIC_PICTURE](#RESP_PUBLIC_PICTURE)
- Description: Requests burst downlink of picture from provided slot. Only public slot numbers (80 - 100) can be requested.

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
    - 0x20: ADCS parameters (all settings in persistent storage will be reset to the default values)
    - 0x40: ADCS ephemerids
- Response: none
- Description: Wipes persistent storages.

### CMD_SET_TRANSMIT_ENABLE
- Optional data length: 3
- Optional data:
  - 0: transmit enable (0x01) or disable (0x00)
  - 1: automated statistics transmission enable (0x01) or disable (0x00)
  - 2: FSK mandated for large packets enable (0x01) or disable (0x00)
- Response: none
- Description: Can be used to completely disable all transmissions from satellite. Additionally controls whether satellite transmits statistics automatically (enabled by default) and whether FSK mode is mandated for large packets (enabled by default).

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
- Optional data length: 5
- Optional data:
  - 0 - 1: number of samples, unsigned 16-bit integer, LSB first
  - 2 - 3: sampling period in ms, unsigned 16-bit integer, LSB first
  - 4: flags which specify the device to use, any combination:
    - 0x01: gyroscope
    - 0x02: accelerometer
    - 0x04: magnetometer
- Response: [RESP_RECORDED_IMU](#RESP_RECORDED_IMU)
- Description: Records selected IMU device(s). Logging will be stopped if battery voltage drops below low power mode level. Only available in FSK mode, each sample is sent in separate response

### CMD_RUN_MANUAL_ACS
- Optional data length: 23
- Optional data:
  - 0: X axis H-bridge high magnitude, signed 8-bit integer, -63 to 63
  - 1: X axis H-bridge low magnitude, signed 8-bit integer, -63 to 63
  - 2: Y axis H-bridge high magnitude, signed 8-bit integer, -63 to 63
  - 3: Y axis H-bridge low magnitude, signed 8-bit integer, -63 to 63
  - 4: Z axis H-bridge high magnitude, signed 8-bit integer, -63 to 63
  - 5: Z axis H-bridge low magnitude, signed 8-bit integer, -63 to 63
  - 6 - 9: X axis pulse length in ms, unsigned 32-bit integer, LSB first
  - 10 - 13: Y axis pulse length in ms, unsigned 32-bit integer, LSB first
  - 14 - 17: Z axis pulse length in ms, unsigned 32-bit integer, LSB first
  - 18 - 21 maneuver duration in ms, unsigned 32-bit integer, LSB first
  - 22: flags which specify whether to ignore H-bridge fault:
    - 0x01: X axis
    - 0x02: Y axis
    - 0x04: Z axis
- Response: [RESP_MANUAL_ACS_RESULT](#RESP_MANUAL_ACS_RESULT)
- Description: Performs ADCS maneuver. May be terminated prematurely if one or more H-bridges return some error, or if battery voltage drops below low power mode level.

### CMD_LOG_GPS
- Optional data length: 8
- Optional data:
  - 0 - 3: GPS logging duration in seconds, unsigned 32-bit integer, LSB first
  - 4 - 7: GPS logging start offset in seconds, unsigned 32-bit integer, LSB first
- Response: [RESP_GPS_LOG_STATE](#RESP_GPS_LOG_STATE)
- Description: Records GPS output. Logging will be stopped if battery voltage drops below low power mode level.

### CMD_GET_GPS_LOG
- Optional data length: 5
- Optional data:
  - 0: whether to download GPS log from start (0x00 - oldest entries sent first) or start (0x01 - newest entries sent first)
  - 1 - 2: offset from GPS log start (as a number of NMEA sequences), unsigned 16-bit integer, LSB first
  - 3 - 4: number of NMEA sentences do downlink, or 0 to downlink the entire log, unsigned 16-bit integer, LSB first
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
- Optional data length: 4
- Optional data:
  - 0: picture slot to read
  - 1 - 2: picture packet ID at which should the reading start, unsigned 16-bit integer, LSB first
  - 3: whether to send the full picture (0x00) or only the scan data (0x01)
- Response: [RESP_CAMERA_PICTURE](#RESP_CAMERA_PICTURE)
- Description: Requests burst downlink of picture from provided slot.

### CMD_ROUTE
- Optional data length: 0 - N
- Optional data:
  - 0 - N: FCP frame to route
- Response: none
- Description: Requests retransmission of the frame in optional data to the recipient satellite. Example for CMD_RETRANSMIT with route ground -> FOSSASAT-2 -> FOSSASAT-1B: `FOSSASAT-2<CMD_ROUTE><0x19>FOSSASAT-1B<CMD_RETRANSMIT><0x0C>Hello World!`

### CMD_SET_FLASH_CONTENTS
- Optional data length: 5 - 132
- Optional data:
  - 0 - 3: flash address to write, unsigned 32-bit integer, LSB first
  - 4 - N: data to be written to flash
- Response: none
- Description: Writes data to arbitrary address in external flash. This command should only be used in emergency situations, as it can be used to override some safety checks.

### CMD_SET_TLE
- Optional data length: 138
- Optional data:
  - 0 - 137: TLE to be set, excluding the title line.
- Response: none
- Description: Writes new TLE into non-volatile storage. The format is the same as standard ASCII TLE, 69 characters per line, without line ending characters, the title line is not included.

### CMD_GET_GPS_LOG_STATE
- Optional data length: 0
- Response: [RESP_GPS_LOG_STATE](#RESP_GPS_LOG_STATE)
- Description: Requests state of GPS log (length, last entry address, last fix address).

### CMD_RUN_GPS_COMMAND
- Optional data length: 0 - N
- Optional data:
  - 0 - N: SkyTraq binary protocol message to be sent (payload only, without sequence start/stop bytes, payload length or checksum)
- Response: [RESP_GPS_COMMAND_RESPONSE](#RESP_GPS_COMMAND_RESPONSE)
- Description: Requests state of GPS log (length, last entry address, last fix address).

### CMD_SET_SLEEP_INTERVALS
- Optional data length: N x 4 (maximum N = 8)
- Optional data:
  - 0 - 1: voltage level of the first sleep interval in mV, signed 16-bit integer, LSB first
  - 2 - 3: first sleep interval length in seconds, unsigned 16-bit integer, LSB first
  - 4 - 7: second sleep interval voltage level and length
  - 8 - 11: third sleep interval voltage level and length etc.
- Response: none
- Description: Sets new sleep interval voltage levels and sleep durations. Maximum of 8 intervals can be set, MUST be ordered by voltage threshold (highest to lowest).

### CMD_ABORT
- Optional data length: 0
- Response: none
- Description: Aborts the currently running operation (e.g. GPS logging or ADCS control).

### CMD_MANEUVER
- Optional data length: 5
- Optional data:
  - 0: control flags
    - bit 0: override X axis H-bridge fault check
    - bit 1: override Y axis H-bridge fault check
    - bit 2: override Z axis H-bridge fault check
    - bit 3: ignored (see [CMD_DETUMBLE](#CMD_DETUMBLE))
    - bit 4: override Euler angle tolerance check
    - bit 5: override angular velocity tolerance check
  - 1 - 4: maneuver length in ms, unsigned 32-bit integer, LSB first
- Response: none
- Description: Performs maneuver with closed-loop ADCS control.

### CMD_SET_ADCS_PARAMETERS
- Optional data length: 63
- Optional data:
  - 0 - 3: maximum pulse intensity, float
  - 4 - 7: maximum pulse length, float
  - 8 - 11: detumbling angular velocity change tolerance, float
  - 12 - 15: minimum inertial moment, float
  - 16 - 19: pulse amplitude, float
  - 20 - 23: calculation tolerance, float
  - 24 - 27: active control Euler angle change tolerance, float
  - 28 - 31: active control angular velocity change tolerance, float
  - 32 - 35: eclipse threshold, float
  - 36 - 39: rotation matrix weight ratio, float
  - 40 - 43: rotation verification trigger level, float
  - 44 - 47: Kalman filter disturbance covariance, float
  - 48 - 51: Kalman filter noise covariance, float
  - 52 - 55: time step between ADCS updates in ms, unsigned 32-bit integer, LSB first
  - 56 - 59: H-bridge timer update period in ms, unsigned 32-bit integer, LSB first
  - 60: H-bridge value for "high" output, signed 8-bit integer, -63 to 63
  - 61: H-bridge value for "low" output, signed 8-bit integer, -63 to 63
  - 62: number of controllers
- Response: none
- Description: Sets configurable ADCS parameters. Floating point numbers should are sent first due to internal automated conversion.

### CMD_ERASE_FLASH
- Optional data length: 4
- Optional data:
  - 0 - 3: address of sector to erase, unsigned 32-bit integer, LSB first
- Response: none
- Description: Erases one external flash sector.

### CMD_SET_ADCS_CONTROLLER
- Optional data length: 77
- Optional data:
  - 0: controller ID
  - 1 - 76: ADCS controller matrix, 3x6 array, float
- Response: none
- Description: Sets one of the ADCS controllers. The number of active controllers must be updated using CMD_SET_ADCS_PARAMETERS! Controller array values are sent in the order [0][0] -> [0][1] -> [0][2] ... [2][3] -> [2][4] -> [2][5], each value is sent LSB first.

### CMD_SET_ADCS_EPHEMERIDES
- Optional data length:
- Optional data:
  - 0 - 1: ephemerides 128-byte chunk ID (0 - 4095), unsigned 16-bit integer, LSB first
  - 2 - N: ephemerides data: 3x solar ephemeris (float, X - Y - Z), 3x magnetic ephemeris (float, X - Y - Z) and 1x controller ID (unsigned 8-bit integer) per each row.
- Response: none
- Description: Sets ephemerides data. One frame can contain up to 5 ephemerides rows, which will be written to the address of provided 128-byte chunk.

### CMD_DETUMBLE
- Optional data length: 5
- Optional data:
  - 0: control flags
    - bit 0: override X axis H-bridge fault check
    - bit 1: override Y axis H-bridge fault check
    - bit 2: override Z axis H-bridge fault check
    - bit 3: override detumbling angular velocity tolerance check
    - bit 4: ignored (see [CMD_MANEUVER](#CMD_MANEUVER))
    - bit 5: ignored (see [CMD_MANEUVER](#CMD_MANEUVER))
  - 1 - 4: detumbling length in ms (ignored if detumble-only flag is set), unsigned 32-bit integer, LSB first
- Response: none
- Description: Performs detumbling with closed-loop ADCS control.

### CMD_SET_IMU_OFFSET
- Optional data length: 36
- Optional data:
  - 0 - 3: X axis gyroscope offset in rad/s, float, LSB first
  - 4 - 7: Y axis gyroscope offset in rad/s, float, LSB first
  - 8 - 11: Z axis gyroscope offset in rad/s, float, LSB first
  - 12 - 15: X axis accelerometer offset in m/s^2, float, LSB first
  - 16 - 19: Y axis accelerometer offset in m/s^2, float, LSB first
  - 20 - 23: Z axis accelerometer offset in m/s^2, float, LSB first
  - 24 - 27: X axis magnetometer offset in T, float, LSB first
  - 28 - 31: Y axis magnetometer offset in T, float, LSB first
  - 32 - 35: Z axis magnetometer offset in T, float, LSB first
- Response: none
- Description: Sets static IMU offset.

### CMD_SET_IMU_CALIBRATION
- Optional data length: 48
- Optional data:
  - 0 - 35: transformation matrix, 9 floats
  - 36 - 47: bias vector, 3 floats
- Response: none
- Description: Sets IMU calibration data (matrix sent in the same order as [CMD_SET_ADCS_CONTROLLER](#CMD_SET_ADCS_CONTROLLER)).

---
# Responses

### RESP_ACKNOWLEDGE
- Optional data length: 2
- Optional data:
  - 0: Function ID of the acknowledged frame, or 0xFF if the decoder was unable to read function ID
  - 1: Resulting of frame parsing operations:
    - 0x00: success, command is being executed
    - 0x01: callsign mismatch
    - 0x02: Rx failed
    - 0x03: unable to read function ID
    - 0x04: decryption failed
    - 0x05: optional data length extraction failed
    - 0x06: unknown function ID
- Description: This response frame is sent after receiving any command frame. It serves as acknowledgement of reception, and contains the result of parsing, decoding and decrypting steps taken during processing of the command frame. This result IS NOT the result of command execution, since RESP_ACKNOWLEDGE is sent prior to executing the frame function.

### RESP_PONG
- Optional data length: 0
- Optional data: none

### RESP_REPEATED_MESSAGE
- Optional data length: 4 - 32
- Optional data:
  - 0 - 3: sender ID, unsigned 32-bit integer
  - 4 - N: repeated message

### RESP_REPEATED_MESSAGE_CUSTOM
- Optional data length: 4 - 32
- Optional data:
  - 0 - 3: sender ID, unsigned 32-bit integer
  - 4 - N: repeated message

### RESP_SYSTEM_INFO
- Optional data length: 23
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
  - 19 - 22: external flash system info page CRC error counter, unsigned 32-bit integer

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
- Optional data length: 1 - 217
- Optional data:
  - 0: flags of statistics included in the response
  - 30 bytes for temperatures * 0.01 deg. C, signed 16-bit integers
  - 36 bytes for currents * 10 uA, signed 16-bit integers
  - 18 bytes for voltages * 20 mV, unsigned 8-bit integers
  - 24 bytes for light sensors, floats
  - 108 bytes for IMU, floats

### RESP_FULL_SYSTEM_INFO
- Optional data length: 56
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
    - bit 7: science mode active
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
  - 35 - 36: MCU temperature * 0.01 deg. C, signed 16-bit integer
  - 37 - 40: Y panel light sensor * 1 lux, float
  - 41 - 44: top board light sensor * 1 lux, float
  - 45: last X axis H-bridge fault, unsigned 8-bit integer
  - 46: last Y axis H-bridge fault, unsigned 8-bit integer
  - 47: last Z axis H-bridge fault, unsigned 8-bit integer
  - 48 - 51: external flash system info page CRC error counter, unsigned 32-bit integer
  - 52: FSK window receive length in seconds
  - 53: LoRa window receive length in seconds
  - 54: sensor states:
    - bit 0: light sensor top
    - bit 1: light sensor Y
    - bit 2: current sensor MPPT
    - bit 3: current sensor Y
    - bit 4: current sensor ZB
    - bit 5: current sensor ZA
    - bit 6: current sensor XB
    - bit 7: current sensor XA
  - 55: last ADCS result

### RESP_STORE_AND_FORWARD_ASSIGNED_SLOT
- Optional data length: 2
- Optional data:
  - 0 - 1: Slot assigned to this message, unsigned 16-bit integer

### RESP_FORWARDED_MESSAGE
- Optional data length: 0 - 27
- Optional data:
  - 0 - N: Requested message

### RESP_PUBLIC_PICTURE
- Optional data length: 6 - 162
- Optional data:
  - 0 - 1: picture packet ID, unsigned 16-bit integer
  - 2 - N: picture data (4 null bytes sent if the requested slot has no image), last 32 bytes are RS(255, 223) FEC code.

### RESP_DEPLOYMENT_STATE
- Optional data length: 1
- Optional data:
  - 0: deployment counter, unsigned 8-bit integer

### RESP_CAMERA_STATE
- Optional data length: 4
- Optional data:
  - 0 - 3: captured image length or camera error state, unsigned 32-bit integer

### RESP_RECORDED_IMU
- Optional data length: 13 - 37
- Optional data:
  - 0: flags which specify the device to use, any combination may be active:
    - 0x01: gyroscope
    - 0x02: accelerometer
    - 0x04: magnetometer
  - 1 - 12: gyroscope sample (X/Y/Z, only present if gyroscope flag is set), float
  - 13 - 24: accelerometer sample (X/Y/Z, only present if accelerometer flag is set), float
  - 25 - 36: magnetometer sample (X/Y/Z, only present if magnetometer flag is set), float

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

### RESP_GPS_LOG_STATE
- Optional data length: 12
- Optional data:
  - 0 - 3: GPS log length
  - 4 - 7: address of the last NMEA entry
  - 8 - 11: address of the last NMEA fix

### RESP_FLASH_CONTENTS
- Optional data length: 0 - 128
- Optional data:
  - 0 - N: flash data

### RESP_CAMERA_PICTURE
- Optional data length: 6 - 162
- Optional data:
  - 0 - 1: picture packet ID, unsigned 16-bit integer
  - 2 - N: picture data (4 null bytes sent if the requested slot has no image), last 32 bytes are RS(255, 223) FEC code.

### RESP_CAMERA_PICTURE_LENGTH
- Optional data length: 4
- Optional data:
  - 0 - 3: length of image in requested slot

### RESP_GPS_COMMAND_RESPONSE
- Optional data length: 0 - N
- Optional data:
  - 0 - N: response to GPS command
