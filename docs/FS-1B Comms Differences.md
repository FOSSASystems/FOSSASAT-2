# FOSSASAT-1B communication protocol differences
The following is a list of all frames that have different optional data for FOSSASAT-1B and FOSSASAT-2. FOSSASAT-1B does not implement any private frames above CMD_RECORD_SOLAR_CELLS.

### RESP_SYSTEM_INFO
- Optional data length: 19
- Optional data:
  - 0: battery voltage * 20 mV, unsigned 8-bit integer
  - 1 - 2: battery charging current * 10 uA, signed 16-bit integer
  - 3: battery charging voltage * 20 mV, unsigned 8-bit integer
  - 4 - 7: time since last reset in seconds, unsigned 32-bit integer
  - 8: power configuration
    - bit 0: low power mode currently active
    - bit 1: low power mode enabled
    - bit 2: MPPT temperature switch enabled
    - bit 3: MPPT keep alive enabled
    - bit 5: transmissions enabled
  - 9 - 10: reset counter, unsigned 16-bit integer
  - 11: solar panel A voltage * 20 mV, unsigned 8-bit integer
  - 12: solar panel B voltage * 20 mV, unsigned 8-bit integer
  - 13: solar panel C voltage * 20 mV, unsigned 8-bit integer
  - 14 - 15: battery temperature * 0.01 deg. C, signed 16-bit integer
  - 16 - 17: OBC board temperature * 0.01 deg. C, signed 16-bit integer
  - 18: MCU temperature in deg. C, signed 8-bit integer

### CMD_GET_STATISTICS
- Optional data length: 1
- Optional data:
  - 0: flags of stats to include. All flags may be active at once.
    - 0x01: charging voltage
    - 0x02: charging current
    - 0x04: battery voltage
    - 0x08: solar cell A voltage
    - 0x10: solar cell B voltage
    - 0x20: solar cell C voltage
    - 0x40: battery temperature
    - 0x80: board temperature
- Response: [RESP_STATISTICS](#RESP_STATISTICS)
- Description: Request satellite statistics according to the set flags.

### RESP_STATISTICS
- Optional data length: 1 - 34
- Optional data:
  - 0: flags of statistics included in the response
  - 3 bytes for battery charging voltage * 20 mV, unsigned 8-bit integer
  - 6 bytes for battery charging current * 10 uA, signed 16-bit integer
  - 3 bytes for battery voltage * 20 mV, unsigned 8-bit integer
  - 3 bytes for solar cell A voltage * 20 mV, unsigned 8-bit integer
  - 3 bytes for solar cell B voltage * 20 mV, unsigned 8-bit integer
  - 3 bytes for solar cell C voltage * 20 mV, unsigned 8-bit integer
  - 6 bytes for battery temperature * 0.01 deg. C, signed 16-bit integer
  - 6 bytes for OBC board temperature * 0.01 deg. C, signed 16-bit integer

### CMD_WIPE_EEPROM
- Optional data length: 0
- Optional data: none
- Response: none
- Description: Wipes persistent storages.

### CMD_SET_TRANSMIT_ENABLE
- Optional data length: 1
- Optional data:
  - 0: transmit enable (0x01) or disable (0x00)
- Response: none
- Description: Can be used to completely disable all transmissions from satellite.

### CMD_RECORD_SOLAR_CELLS
- Optional data length: 3
- Optional data:
  - 0: number of samples to record, maximum of 40
  - 1 - 2: Sampling period in ms, unsigned 16-bit integer
- Response: none
- Description: Starts recording voltages on all solar cells.

### RESP_RECORDED_SOLAR_CELLS
- Optional data length: 0 - 120 (depending on number of samples)
- Optional data:
  - N + 0: solar cell A voltage sample * 20 mV, unsigned 8-bit integer
  - N + 1: solar cell B voltage sample * 20 mV, unsigned 8-bit integer
  - N + 2: solar cell C voltage sample * 20 mV, unsigned 8-bit integer
