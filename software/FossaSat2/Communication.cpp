#include "Communication.h"

void Communication_Receive_Interrupt() {
  // check interrups are enabled
  if (!interruptsEnabled) {
    return;
  }

  // set flag
  dataReceived = true;
}

int16_t Communication_Set_SpreadingFactor(uint8_t sfMode) {
  uint8_t sfs[] = {LORA_SPREADING_FACTOR, LORA_SPREADING_FACTOR_ALT};

  // check currently active modem
  if(currentModem == MODEM_FSK) {
    return(ERR_WRONG_MODEM);
  }

  // update standard/alternative spreading factor
  int16_t state = radio.setSpreadingFactor(sfs[sfMode]);

  // only save current spreading factor mode if the change was successful
  if(state == ERR_NONE) {
    spreadingFactorMode = sfMode;
  }

  return(state);
}

int16_t Communication_Set_LoRa_Configuration(float bw, uint8_t sf, uint8_t cr, uint16_t preambleLen, bool crc, int8_t power) {
  // set LoRa radio config
  int16_t state = radio.begin(CARRIER_FREQUENCY, bw, sf, cr, SYNC_WORD, power, LORA_CURRENT_LIMIT, preambleLen, TCXO_VOLTAGE);
  if (state != ERR_NONE) {
    return (state);
  }

  // set CRC
  state = radio.setCRC(crc);
  return (state);
}

int16_t Communication_Set_Modem(uint8_t modem) {
  int16_t state = ERR_NONE;
  FOSSASAT_DEBUG_PRINT(F("Set modem "));
  FOSSASAT_DEBUG_WRITE(modem);
  FOSSASAT_DEBUG_PRINTLN();

  // initialize requested modem
  switch (modem) {
    case MODEM_LORA:
        state = radio.begin(CARRIER_FREQUENCY,
                            LORA_BANDWIDTH,
                            LORA_SPREADING_FACTOR,
                            LORA_CODING_RATE,
                            SYNC_WORD,
                            LORA_OUTPUT_POWER,
                            LORA_CURRENT_LIMIT,
                            LORA_PREAMBLE_LENGTH,
                            TCXO_VOLTAGE);
        radio.setCRC(true);
      break;
    case MODEM_FSK: {
        state = radio.beginFSK(CARRIER_FREQUENCY,
                               FSK_BIT_RATE,
                               FSK_FREQUENCY_DEVIATION,
                               FSK_RX_BANDWIDTH,
                               FSK_OUTPUT_POWER,
                               FSK_CURRENT_LIMIT,
                               FSK_PREAMBLE_LENGTH,
                               FSK_DATA_SHAPING,
                               TCXO_VOLTAGE);
        uint8_t syncWordFSK[2] = {SYNC_WORD, SYNC_WORD};
        radio.setSyncWord(syncWordFSK, 2);
        radio.setCRC(2);
      } break;
    default:
      FOSSASAT_DEBUG_PRINT(F("Unkown modem "));
      FOSSASAT_DEBUG_PRINTLN(modem);
      return(ERR_UNKNOWN);
  }

  // handle possible error codes
  FOSSASAT_DEBUG_PRINT(F("Radio init "));
  FOSSASAT_DEBUG_PRINTLN(state);
  FOSSASAT_DEBUG_DELAY(10);
  if (state != ERR_NONE) {
    // radio chip failed, restart
    PowerControl_Watchdog_Restart();
  }

  // set spreading factor
  Communication_Set_SpreadingFactor(spreadingFactorMode);

  // save current modem
  currentModem = modem;
  return(state);
}

void Communication_Send_Morse_Beacon(float battVoltage) {
  // initialize Morse client
  morse.begin(CARRIER_FREQUENCY, MORSE_SPEED);

  // read callsign
  uint8_t callsignLen = PersistentStorage_Get<uint8_t>(FLASH_CALLSIGN_LEN);
  char callsign[MAX_STRING_LENGTH];
  PersistentStorage_Get_Callsign(callsign, callsignLen);

  // send start signals
  for(uint8_t i = 0; i < MORSE_PREAMBLE_LENGTH; i++) {
    morse.startSignal();
    FOSSASAT_DEBUG_PRINT('*');
    FOSSASAT_DEBUG_DELAY(10);
    PowerControl_Watchdog_Heartbeat();
  }

  // send callsign
  for(uint8_t i = 0; i < callsignLen; i++) {
    morse.print(callsign[i]);
    FOSSASAT_DEBUG_PRINT(callsign[i]);
    FOSSASAT_DEBUG_DELAY(10);
    PowerControl_Watchdog_Heartbeat();
  }

  // space
  morse.print(' ');
  FOSSASAT_DEBUG_PRINT(' ');
  FOSSASAT_DEBUG_DELAY(10);
  PowerControl_Watchdog_Heartbeat();

  // send battery voltage code
  char code = 'A' + (uint8_t)((battVoltage - MORSE_BATTERY_MIN) / MORSE_BATTERY_STEP);
  morse.println(code);
  FOSSASAT_DEBUG_PRINTLN(code);
  FOSSASAT_DEBUG_DELAY(10);
  PowerControl_Watchdog_Heartbeat();
}

void Communication_CW_Beep(uint32_t len) {
  FOSSASAT_DEBUG_PRINTLN(F("beep"));
  FOSSASAT_DEBUG_DELAY(10);
  radio.transmitDirect();
  PowerControl_Wait(len, LOW_POWER_NONE);
  radio.standby();
}

void Communication_Send_Basic_System_Info() {
  // build response frame
  static const uint8_t optDataLen = 7*sizeof(uint8_t) + 3*sizeof(int16_t) + sizeof(uint16_t) + sizeof(uint32_t);
  uint8_t optData[optDataLen];
  uint8_t* optDataPtr = optData;

  FOSSASAT_DEBUG_PRINTLN(F("--- System info: ---"));

  uint8_t mpptOutputVoltage = currSensorMPPT.readBusVoltage() * (VOLTAGE_UNIT / VOLTAGE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, mpptOutputVoltage, "batteryVoltage", VOLTAGE_MULTIPLIER, "mV");

  int16_t mpptOutputCurrent = currSensorMPPT.readCurrent() * (CURRENT_UNIT / CURRENT_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, mpptOutputCurrent, "mpptOutputCurrent", CURRENT_MULTIPLIER, "uA");

  uint32_t onboardTime = rtc.getEpoch();
  Communication_Frame_Add(&optDataPtr, onboardTime, "onboardTime", 1, "");

  // power config: FLASH_TRANSMISSIONS_ENABLED (0), FLASH_LOW_POWER_MODE_ENABLED (1), FLASH_LOW_POWER_MODE (2 - 4), FLASH_MPPT_TEMP_SWITCH_ENABLED (5), FLASH_MPPT_KEEP_ALIVE_ENABLED (6)
  uint8_t powerConfig = ((PersistentStorage_Get<uint8_t>(FLASH_TRANSMISSIONS_ENABLED)           & 0b00000001) |
                        ((PersistentStorage_Get<uint8_t>(FLASH_LOW_POWER_MODE_ENABLED)    << 1) & 0b00000010) |
                        ((PersistentStorage_Get<uint8_t>(FLASH_LOW_POWER_MODE)            << 2) & 0b00011100) |
                        ((PersistentStorage_Get<uint8_t>(FLASH_MPPT_TEMP_SWITCH_ENABLED)  << 5) & 0b00100000) |
                        ((PersistentStorage_Get<uint8_t>(FLASH_MPPT_KEEP_ALIVE_ENABLED)   << 6) & 0b01000000));
  Communication_Frame_Add(&optDataPtr, powerConfig, "powerConfig", 1, "");

  uint16_t resetCounter = PersistentStorage_Get<uint16_t>(FLASH_RESTART_COUNTER);
  Communication_Frame_Add(&optDataPtr, resetCounter, "resetCounter", 1, "");

  uint8_t voltageXA = currSensorXA.readBusVoltage() * (VOLTAGE_UNIT / VOLTAGE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, voltageXA, "voltageXA", VOLTAGE_MULTIPLIER, "mV");

  uint8_t voltageXB = currSensorXB.readBusVoltage() * (VOLTAGE_UNIT / VOLTAGE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, voltageXB, "voltageXB", VOLTAGE_MULTIPLIER, "mV");

  uint8_t voltageZA = currSensorZA.readBusVoltage() * (VOLTAGE_UNIT / VOLTAGE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, voltageZA, "voltageZA", VOLTAGE_MULTIPLIER, "mV");

  uint8_t voltageZB = currSensorZB.readBusVoltage() * (VOLTAGE_UNIT / VOLTAGE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, voltageZB, "voltageZB", VOLTAGE_MULTIPLIER, "mV");

  uint8_t voltageY = currSensorY.readBusVoltage() * (VOLTAGE_UNIT / VOLTAGE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, voltageY, "voltageY", VOLTAGE_MULTIPLIER, "mV");

  int16_t batteryTemperature = Sensors_Read_Temperature(tempSensorBattery) * (TEMPERATURE_UNIT / TEMPERATURE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, batteryTemperature, "batteryTemperature", TEMPERATURE_MULTIPLIER, "mdeg C");

  int16_t boardTemperature = Sensors_Read_Temperature(tempSensorTop) * (TEMPERATURE_UNIT / TEMPERATURE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, boardTemperature, "boardTemperature", TEMPERATURE_MULTIPLIER, "mdeg C");

  FOSSASAT_DEBUG_PRINTLN(F("--------------------"));

  // send response
  Communication_Send_Response(RESP_SYSTEM_INFO, optData, optDataLen);
}

void Communication_Send_Full_System_Info() {
  // build response frame
  static const uint8_t optDataLen = 10*sizeof(uint8_t) + 11*sizeof(int16_t) + sizeof(uint16_t) + sizeof(uint32_t) + 2*sizeof(float);
  uint8_t optData[optDataLen];
  uint8_t* optDataPtr = optData;

  FOSSASAT_DEBUG_PRINTLN(F("--- System info: ---"));

  uint8_t mpptOutputVoltage = currSensorMPPT.readBusVoltage() * (VOLTAGE_UNIT / VOLTAGE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, mpptOutputVoltage, "batteryVoltage", VOLTAGE_MULTIPLIER, "mV");

  int16_t mpptOutputCurrent = currSensorMPPT.readCurrent() * (CURRENT_UNIT / CURRENT_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, mpptOutputCurrent, "mpptOutputCurrent", CURRENT_MULTIPLIER, "uA");

  uint32_t onboardTime = rtc.getEpoch();
  Communication_Frame_Add(&optDataPtr, onboardTime, "onboardTime", 1, "");

  // power config: FLASH_TRANSMISSIONS_ENABLED (0), FLASH_LOW_POWER_MODE_ENABLED (1), FLASH_LOW_POWER_MODE (2 - 4), FLASH_MPPT_TEMP_SWITCH_ENABLED (5), FLASH_MPPT_KEEP_ALIVE_ENABLED (6)
  uint8_t powerConfig = ((PersistentStorage_Get<uint8_t>(FLASH_TRANSMISSIONS_ENABLED)           & 0b00000001) |
                        ((PersistentStorage_Get<uint8_t>(FLASH_LOW_POWER_MODE_ENABLED)    << 1) & 0b00000010) |
                        ((PersistentStorage_Get<uint8_t>(FLASH_LOW_POWER_MODE)            << 2) & 0b00011100) |
                        ((PersistentStorage_Get<uint8_t>(FLASH_MPPT_TEMP_SWITCH_ENABLED)  << 5) & 0b00100000) |
                        ((PersistentStorage_Get<uint8_t>(FLASH_MPPT_KEEP_ALIVE_ENABLED)   << 6) & 0b01000000));
  Communication_Frame_Add(&optDataPtr, powerConfig, "powerConfig", 1, "");

  uint16_t resetCounter = PersistentStorage_Get<uint16_t>(FLASH_RESTART_COUNTER);
  Communication_Frame_Add(&optDataPtr, resetCounter, "resetCounter", 1, "");

  uint8_t voltageXA = currSensorXA.readBusVoltage() * (VOLTAGE_UNIT / VOLTAGE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, voltageXA, "voltageXA", VOLTAGE_MULTIPLIER, "mV");

  int16_t currentXA = currSensorXA.readCurrent() * (CURRENT_UNIT / CURRENT_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, currentXA, "currentXA", CURRENT_MULTIPLIER, "uA");

  uint8_t voltageXB = currSensorXB.readBusVoltage() * (VOLTAGE_UNIT / VOLTAGE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, voltageXB, "voltageXB", VOLTAGE_MULTIPLIER, "mV");

  int16_t currentXB = currSensorXB.readCurrent() * (CURRENT_UNIT / CURRENT_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, currentXB, "currentXB", CURRENT_MULTIPLIER, "uA");

  uint8_t voltageZA = currSensorZA.readBusVoltage() * (VOLTAGE_UNIT / VOLTAGE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, voltageZA, "voltageZA", VOLTAGE_MULTIPLIER, "mV");

  int16_t currentZA = currSensorZA.readCurrent() * (CURRENT_UNIT / CURRENT_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, currentZA, "currentZA", CURRENT_MULTIPLIER, "uA");

  uint8_t voltageZB = currSensorZB.readBusVoltage() * (VOLTAGE_UNIT / VOLTAGE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, voltageZB, "voltageZB", VOLTAGE_MULTIPLIER, "mV");

  int16_t currentZB = currSensorZB.readCurrent() * (CURRENT_UNIT / CURRENT_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, currentZB, "currentZB", CURRENT_MULTIPLIER, "uA");

  uint8_t voltageY = currSensorY.readBusVoltage() * (VOLTAGE_UNIT / VOLTAGE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, voltageY, "voltageY", VOLTAGE_MULTIPLIER, "mV");

  int16_t currentY = currSensorY.readCurrent() * (CURRENT_UNIT / CURRENT_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, currentY, "currentY", CURRENT_MULTIPLIER, "uA");

  int16_t tempPanelY = Sensors_Read_Temperature(tempSensorPanelY) * (TEMPERATURE_UNIT / TEMPERATURE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, tempPanelY, "tempPanelY", TEMPERATURE_MULTIPLIER, "mdeg C");

  int16_t boardTemperature = Sensors_Read_Temperature(tempSensorTop) * (TEMPERATURE_UNIT / TEMPERATURE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, boardTemperature, "boardTemperature", TEMPERATURE_MULTIPLIER, "mdeg C");

  int16_t tempBottom = Sensors_Read_Temperature(tempSensorBottom) * (TEMPERATURE_UNIT / TEMPERATURE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, tempBottom, "tempBottom", TEMPERATURE_MULTIPLIER, "mdeg C");

  int16_t batteryTemperature = Sensors_Read_Temperature(tempSensorBattery) * (TEMPERATURE_UNIT / TEMPERATURE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, batteryTemperature, "batteryTemperature", TEMPERATURE_MULTIPLIER, "mdeg C");

  int16_t secBatteryTemperature = Sensors_Read_Temperature(tempSensorSecBattery) * (TEMPERATURE_UNIT / TEMPERATURE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, secBatteryTemperature, "secBatteryTemperature", TEMPERATURE_MULTIPLIER, "mdeg C");

  float lightPanelY = lightSensorPanelY.readLux();
  Communication_Frame_Add(&optDataPtr, lightPanelY, "lightPanelY", 1, "lux");

  float lightTop = lightSensorTop.readLux();
  Communication_Frame_Add(&optDataPtr, lightTop, "lightTop", 1, "lux");

  uint8_t bridgeXfault = bridgeX.getFault();
  Communication_Frame_Add(&optDataPtr, bridgeXfault, "bridgeXfault", 1, "");

  uint8_t bridgeYfault = bridgeY.getFault();
  Communication_Frame_Add(&optDataPtr, bridgeYfault, "bridgeYfault", 1, "");

  uint8_t bridgeZfault = bridgeZ.getFault();
  Communication_Frame_Add(&optDataPtr, bridgeZfault, "bridgeZfault", 1, "");

  FOSSASAT_DEBUG_PRINTLN(F("--------------------"));

  // send response
  Communication_Send_Response(RESP_FULL_SYSTEM_INFO, optData, optDataLen);
}

template <typename T>
void Communication_Frame_Add(uint8_t** buffPtr, T val, const char* name, uint32_t mult, const char* unit) {
  memcpy(*buffPtr, &val, sizeof(val));
  (*buffPtr) += sizeof(val);
  FOSSASAT_DEBUG_PRINT(name);
  FOSSASAT_DEBUG_PRINT(F(" = "));
  FOSSASAT_DEBUG_PRINT(val);
  FOSSASAT_DEBUG_PRINT('*');
  FOSSASAT_DEBUG_PRINT(mult);
  FOSSASAT_DEBUG_PRINT(' ');
  FOSSASAT_DEBUG_PRINTLN(unit);
}

template void Communication_Frame_Add<int8_t>(uint8_t**, int8_t, const char*, uint32_t, const char*);
template void Communication_Frame_Add<uint8_t>(uint8_t**, uint8_t, const char*, uint32_t, const char*);
template void Communication_Frame_Add<int16_t>(uint8_t**, int16_t, const char*, uint32_t, const char*);
template void Communication_Frame_Add<uint16_t>(uint8_t**, uint16_t, const char*, uint32_t, const char*);
template void Communication_Frame_Add<uint32_t>(uint8_t**, uint32_t, const char*, uint32_t, const char*);

void Communication_Process_Packet() {
  // disable interrupts
  interruptsEnabled = false;

  // read data
  size_t len = radio.getPacketLength();
  FOSSASAT_DEBUG_PRINT(F("Packet length: "));
  FOSSASAT_DEBUG_PRINTLN(len);
  if(len == 0) {
    dataReceived = false;
    interruptsEnabled = true;
    return;
  }

  uint8_t frame[MAX_RADIO_BUFFER_LENGTH];
  int16_t state = radio.readData(frame, len);

  // check reception state
  if(state == ERR_NONE) {
    FOSSASAT_DEBUG_PRINT(F("Got frame "));
    FOSSASAT_DEBUG_PRINTLN(len);
    FOSSASAT_DEBUG_PRINT_BUFF(frame, len);

    // check callsign
    uint8_t callsignLen = PersistentStorage_Get<uint8_t>(FLASH_CALLSIGN_LEN);
    char callsign[MAX_STRING_LENGTH];
    PersistentStorage_Get_Callsign(callsign, callsignLen);
    if(memcmp(frame, (uint8_t*)callsign, callsignLen - 1) == 0) {
      // check passed
      Comunication_Parse_Frame(frame, len);
    } else {
      FOSSASAT_DEBUG_PRINTLN(F("Callsign mismatch!"));
      PersistentStorage_Increment_Frame_Counter(false);
    }

  } else {
    FOSSASAT_DEBUG_PRINT(F("Reception failed, code "));
    FOSSASAT_DEBUG_PRINT(state);
    PersistentStorage_Increment_Frame_Counter(false);
  }

  // reset flag
  dataReceived = false;

  // enable interrupts
  interruptsEnabled = true;
}

void Comunication_Parse_Frame(uint8_t* frame, uint8_t len) {
  // get callsign from EEPROM
  uint8_t callsignLen = PersistentStorage_Get<uint8_t>(FLASH_CALLSIGN_LEN);
  char callsign[MAX_STRING_LENGTH];
  PersistentStorage_Get_Callsign(callsign, callsignLen);

  // get functionID
  int16_t functionId = FCP_Get_FunctionID(callsign, frame, len);
  if (functionId < 0) {
    FOSSASAT_DEBUG_PRINT(F("Unable to get function ID 0x"));
    FOSSASAT_DEBUG_PRINTLN(functionId, HEX);
    return;
  }
  FOSSASAT_DEBUG_PRINT(F("Function ID = 0x"));
  FOSSASAT_DEBUG_PRINTLN(functionId, HEX);

  // check encryption
  int16_t optDataLen = 0;
  uint8_t optData[MAX_OPT_DATA_LENGTH];
  if((functionId >= PRIVATE_OFFSET) && (functionId <= (PRIVATE_OFFSET + NUM_PRIVATE_COMMANDS))) {
    // frame contains encrypted data, decrypt
    FOSSASAT_DEBUG_PRINTLN(F("Decrypting"));

    // get optional data length
    optDataLen = FCP_Get_OptData_Length(callsign, frame, len, encryptionKey, password);
    if(optDataLen < 0) {
      FOSSASAT_DEBUG_PRINT(F("Decrypt failed "));
      FOSSASAT_DEBUG_PRINTLN(optDataLen);

      // decryption failed, increment invalid frame counter and return
      PersistentStorage_Increment_Frame_Counter(false);
      return;
    }

    // get optional data
    if(optDataLen > 0) {
      FCP_Get_OptData(callsign, frame, len, optData, encryptionKey, password);
    }

  } else if(functionId < PRIVATE_OFFSET) {
    // no decryption necessary

    // get optional data length
    optDataLen = FCP_Get_OptData_Length(callsign, frame, len);
    if(optDataLen < 0) {
      // optional data extraction failed,
      FOSSASAT_DEBUG_PRINT(F("Failed to get optDataLen "));
      FOSSASAT_DEBUG_PRINTLN(optDataLen);

      // increment invalid frame counter
      PersistentStorage_Increment_Frame_Counter(false);
      return;
    }

    // get optional data
    if(optDataLen > 0) {
      FCP_Get_OptData(callsign, frame, len, optData);
    }
  } else {
    // unknown function ID
    FOSSASAT_DEBUG_PRINT(F("Unknown function ID, 0x"));
    FOSSASAT_DEBUG_PRINTLN(functionId, HEX);
    PersistentStorage_Increment_Frame_Counter(false);
    return;
  }

  // check optional data presence
  if(optDataLen > 0) {
    // execute with optional data
    FOSSASAT_DEBUG_PRINT(F("optDataLen = "));
    FOSSASAT_DEBUG_PRINTLN(optDataLen);
    FOSSASAT_DEBUG_PRINT_BUFF(optData, (uint8_t)optDataLen);
    Communication_Execute_Function(functionId, optData, optDataLen);

  } else {
    // execute without optional data
    Communication_Execute_Function(functionId);
  }

}

void Communication_Execute_Function(uint8_t functionId, uint8_t* optData, size_t optDataLen) {
  // increment valid frame counter
  PersistentStorage_Increment_Frame_Counter(true);

  // delay before responding
  FOSSASAT_DEBUG_DELAY(100);
  PowerControl_Wait(RESPONSE_DELAY, true);

  // execute function based on ID
  switch (functionId) {

    // public function IDs

    case CMD_PING: {
      // send pong
      Communication_Send_Response(RESP_PONG);
    } break;

    case CMD_RETRANSMIT: {
        // check message length
        if (optDataLen <= MAX_STRING_LENGTH) {
          // respond with the requested data
          Communication_Send_Response(RESP_REPEATED_MESSAGE, optData, optDataLen);
        }
      } break;

    case CMD_RETRANSMIT_CUSTOM: {
        // check message length
        if ((optDataLen >= 8) && (optDataLen <= MAX_STRING_LENGTH + 7)) {
          // check bandwidth value (loaded from array - rest of settings are checked by library)
          if (optData[0] > 7) {
            FOSSASAT_DEBUG_PRINT(F("Invalid BW "));
            FOSSASAT_DEBUG_PRINTLN(optData[0]);
            break;
          }

          // attempt to change the settings
          float bws[] = {7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0};
          uint16_t preambleLength = 0;
          memcpy(&preambleLength, optData + 3, sizeof(uint16_t));

          // change modem configuration
          int16_t state = Communication_Set_LoRa_Configuration(bws[optData[0]], optData[1], optData[2], preambleLength, optData[5], optData[6]);

          // check if the change was successful
          if (state != ERR_NONE) {
            FOSSASAT_DEBUG_PRINT(F("Custom config failed, code "));
            FOSSASAT_DEBUG_PRINTLN(state);
          } else {
            // configuration changed successfully, transmit response
            Communication_Send_Response(RESP_REPEATED_MESSAGE_CUSTOM, optData + 7, optDataLen - 7, true);
          }
        }
      } break;

    case CMD_TRANSMIT_SYSTEM_INFO: {
      // send system info via LoRa
      Communication_Send_Basic_System_Info();
    } break;

    case CMD_GET_PACKET_INFO: {
        // get last packet info and send it
        static const uint8_t respOptDataLen = 2*sizeof(uint8_t) + 4*sizeof(uint16_t);
        uint8_t respOptData[respOptDataLen];
        uint8_t* respOptDataPtr = respOptData;

        // SNR
        int8_t snr = (int8_t)(radio.getSNR() * 4.0);
        Communication_Frame_Add(&respOptDataPtr, snr, "SNR", 4, "dB");

        // RSSI
        uint8_t rssi = (uint8_t)(radio.getRSSI() * -2.0);
        Communication_Frame_Add(&respOptDataPtr, rssi, "RSSI", 2, "dBm");

        uint16_t loraValid = PersistentStorage_Get<uint16_t>(FLASH_LORA_VALID_COUNTER);
        Communication_Frame_Add(&respOptDataPtr, loraValid, "LoRa valid", 1, "");

        uint16_t loraInvalid = PersistentStorage_Get<uint16_t>(FLASH_LORA_INVALID_COUNTER);
        Communication_Frame_Add(&respOptDataPtr, loraInvalid, "LoRa invalid", 1, "");

        uint16_t fskValid = PersistentStorage_Get<uint16_t>(FLASH_FSK_VALID_COUNTER);
        Communication_Frame_Add(&respOptDataPtr, fskValid, "FSK valid", 1, "");

        uint16_t fskInvalid = PersistentStorage_Get<uint16_t>(FLASH_FSK_INVALID_COUNTER);
        Communication_Frame_Add(&respOptDataPtr, fskInvalid, "FSK invalid", 1, "");

        Communication_Send_Response(RESP_PACKET_INFO, respOptData, respOptDataLen);
      } break;

    case CMD_GET_STATISTICS: {
      if(Communication_Check_OptDataLen(1, optDataLen)) {
        // check FSK is active
        if(currentModem != MODEM_FSK) {
          FOSSASAT_DEBUG_PRINTLN(F("FSK is required to get stats"));
          return;
        }

        // response will have maximum of 109 bytes if all stats are included
        uint8_t respOptData[109];
        uint8_t respOptDataLen = 1;
        uint8_t* respOptDataPtr = respOptData;

        // copy stat flags
        uint8_t flags = optData[0];
        memcpy(respOptDataPtr, &flags, sizeof(uint8_t));
        respOptDataPtr += sizeof(uint8_t);

        if(flags & 0b00000001) {
          // temperatures
          PersistentStorage_Read(FLASH_STATS_TEMP_PANEL_Y, respOptDataPtr, 15*sizeof(int16_t));
          respOptDataPtr += 15*sizeof(int16_t);
          respOptDataLen += 15*sizeof(int16_t);
        }

        if(flags & 0b00000010) {
          // currents
          PersistentStorage_Read(FLASH_STATS_CURR_XA, respOptDataPtr, 18*sizeof(int16_t));
          respOptDataPtr += 18*sizeof(int16_t);
          respOptDataLen += 18*sizeof(int16_t);
        }

        if(flags & 0b00000100) {
          // voltages
          PersistentStorage_Read(FLASH_STATS_VOLT_XA, respOptDataPtr, 18*sizeof(uint8_t));
          respOptDataPtr += 18*sizeof(uint8_t);
          respOptDataLen += 18*sizeof(uint8_t);
        }

        if(flags & 0b00001000) {
          // lights
          PersistentStorage_Read(FLASH_STATS_LIGHT_PANEL_Y, respOptDataPtr, 6*sizeof(float));
          respOptDataPtr += 6*sizeof(float);
          respOptDataLen += 6*sizeof(float);
        }

        Communication_Send_Response(RESP_STATISTICS, respOptData, respOptDataLen);
      }
    } break;

    case CMD_GET_FULL_SYSTEM_INFO: {
      // check FSK is active
      if(currentModem != MODEM_FSK) {
        FOSSASAT_DEBUG_PRINTLN(F("FSK is required to get full system info"));
        return;
      }

      // send complete system info via GFSK
      Communication_Send_Full_System_Info();
    } break;

    case CMD_STORE_AND_FORWARD_ADD: {
      if (optDataLen <= MAX_STRING_LENGTH - 1) {
        // get the user-provided message ID
        uint32_t messageID = 0;
        memcpy(&messageID, optData, sizeof(uint32_t));

        // search storage to see if that ID is already in use
        uint16_t storageLen = PersistentStorage_Get<uint16_t>(FLASH_STORE_AND_FORWARD_LENGTH);
        uint16_t slotNum = 0;
        for(; slotNum < storageLen + 1; slotNum++) {
          uint8_t buff[4];
          PersistentStorage_Read(FLASH_STORE_AND_FORWARD_START + slotNum * MAX_STRING_LENGTH, buff, 4);
          uint32_t id = 0;
          memcpy(&id, buff, sizeof(uint32_t));
          if(id == messageID) {
            break;
          }
          PowerControl_Watchdog_Heartbeat();
        }

        // create message entry from ID, length and message
        uint8_t messageBuff[MAX_STRING_LENGTH];
        uint8_t messageLen = optDataLen - sizeof(uint32_t);
        memcpy(messageBuff, &messageID, sizeof(uint32_t));
        messageBuff[sizeof(uint32_t)] = messageLen;
        memcpy(messageBuff + sizeof(uint32_t) + sizeof(uint8_t), optData + sizeof(uint32_t), messageLen);

        // add message to store and forward
        PersistentStorage_Set_Message(slotNum, messageBuff, optDataLen + 1);

        // update storage length if needed
        if(slotNum > storageLen) {
          PersistentStorage_Set<uint16_t>(FLASH_STORE_AND_FORWARD_LENGTH, slotNum);
        }

        // send response
        uint8_t respOptData[2];
        memcpy(respOptData, &slotNum, sizeof(uint16_t));
        Communication_Send_Response(RESP_STORE_AND_FORWARD_ASSIGNED_SLOT, respOptData, 2);
      }
    } break;

    case CMD_STORE_AND_FORWARD_REQUEST: {
      if(Communication_Check_OptDataLen(4, optDataLen)) {

        // get the user-provided message ID
        uint32_t messageID = 0;
        memcpy(&messageID, optData, sizeof(uint32_t));

        // search storage to see if that ID exists
        uint16_t storageLen = PersistentStorage_Get<uint16_t>(FLASH_STORE_AND_FORWARD_LENGTH);
        uint16_t slotNum = 0;
        bool idFound = false;
        for(; slotNum < storageLen + 1; slotNum++) {
          uint8_t buff[4];
          PersistentStorage_Read(FLASH_STORE_AND_FORWARD_START + slotNum * MAX_STRING_LENGTH, buff, 4);
          uint32_t id = 0;
          memcpy(&id, buff, sizeof(uint32_t));
          FOSSASAT_DEBUG_PRINTLN(id, HEX);
          if(id == messageID) {
            idFound = true;
            break;
          }
          PowerControl_Watchdog_Heartbeat();
        }

        // check if the ID was found
        if(idFound) {
          // fetch message from storage
          uint8_t messageBuff[MAX_STRING_LENGTH];
          uint8_t messageLen = PersistentStorage_Get_Message(slotNum, messageBuff);
          Communication_Send_Response(RESP_FORWARDED_MESSAGE, messageBuff, messageLen);
          return;
        }

        // requested message does not exist
        uint8_t respOptData[] = {0xFF, 0xFF};
        Communication_Send_Response(RESP_FORWARDED_MESSAGE, respOptData, 2);
      }
    } break;

    // private frames below this line

    case CMD_DEPLOY: {
        // run deployment sequence
        PowerControl_Deploy();

        // get deployment counter value and send it
        uint8_t attemptNumber = PersistentStorage_Get<uint8_t>(FLASH_DEPLOYMENT_COUNTER);
        Communication_Send_Response(RESP_DEPLOYMENT_STATE, &attemptNumber, 1);
      } break;

    case CMD_RESTART: {
      // restart
      PowerControl_Watchdog_Restart();
    } break;

    case CMD_WIPE_EEPROM: {
        // check optional data length
        if(Communication_Check_OptDataLen(1, optDataLen)) {
          // optional data present, check the value
          if(optData[0] & 0b00000001) {
            // wipe system info
            FOSSASAT_DEBUG_PRINTLN(F("Resetting system info"));
            PersistentStorage_Reset_System_Info();
            PowerControl_Watchdog_Heartbeat();
          }

          if(optData[0] & 0b00000010) {
            // wipe stats
            FOSSASAT_DEBUG_PRINTLN(F("Wiping stats"));
            PersistentStorage_SectorErase(FLASH_STATS);
            PowerControl_Watchdog_Heartbeat();
          }

          if(optData[0] & 0b00000100) {
            // wipe store & forward
            FOSSASAT_DEBUG_PRINTLN(F("Wiping store & forward"));
            PersistentStorage_64kBlockErase(FLASH_STORE_AND_FORWARD_START);
            PowerControl_Watchdog_Heartbeat();
            // reset store & forward length
            PersistentStorage_Set<uint32_t>(FLASH_STORE_AND_FORWARD_LENGTH, 0);
          }

          if(optData[0] & 0b00001000) {
            // wipe NMEA
            FOSSASAT_DEBUG_PRINTLN(F("Wiping NMEA storage"));
            PersistentStorage_64kBlockErase(FLASH_NMEA_LOG_START);
            PowerControl_Watchdog_Heartbeat();
            // reset NMEA log length
            PersistentStorage_Set<uint32_t>(FLASH_NMEA_LOG_LENGTH, 0);
          }

          if(optData[0] & 0b00010000) {
            // wipe image lengths
            FOSSASAT_DEBUG_PRINTLN(F("Wiping image lengths"));
            PersistentStorage_SectorErase(FLASH_IMAGE_LENGTHS_1);
            PersistentStorage_SectorErase(FLASH_IMAGE_LENGTHS_2);
            PowerControl_Watchdog_Heartbeat();

            // wipe all 64k image blocks
            FOSSASAT_DEBUG_PRINTLN(F("Wiping images (will take about 3 minutes)"));
            for(uint32_t addr = FLASH_IMAGES_START; addr < FLASH_CHIP_SIZE; addr += FLASH_64K_BLOCK_SIZE) {
              PersistentStorage_64kBlockErase(addr);
              PowerControl_Watchdog_Heartbeat();
            }
            FOSSASAT_DEBUG_PRINTLN(F("Image wipe done"));
          }
        }
      } break;

    case CMD_SET_TRANSMIT_ENABLE: {
        // check optional data length
        if(Communication_Check_OptDataLen(1, optDataLen)) {
          PersistentStorage_Set(FLASH_TRANSMISSIONS_ENABLED, optData[0]);
        }
      } break;

    case CMD_SET_CALLSIGN: {
        // check optional data is less than limit
        if(optDataLen <= MAX_STRING_LENGTH) {
          // get callsign from frame
          char newCallsign[MAX_STRING_LENGTH];
          memcpy(newCallsign, optData, optDataLen);
          newCallsign[optDataLen] = '\0';

          // update callsign
          PersistentStorage_Set_Callsign(newCallsign);
          FOSSASAT_DEBUG_PRINT(F("newCallsign = "));
          FOSSASAT_DEBUG_PRINTLN(newCallsign);
        }
      } break;

    case CMD_SET_SF_MODE: {
        // check optional data is exactly 1 byte
        if(Communication_Check_OptDataLen(1, optDataLen)) {
          // update spreading factor mode
          spreadingFactorMode = optData[0];
          FOSSASAT_DEBUG_PRINT(F("spreadingFactorMode="));
          FOSSASAT_DEBUG_PRINTLN(spreadingFactorMode);
          Communication_Set_SpreadingFactor(spreadingFactorMode);
        }
      } break;

    case CMD_SET_LOW_POWER_ENABLE: {
        // check optional data is exactly 1 byte
        if(Communication_Check_OptDataLen(1, optDataLen)) {
          // update spreading factor mode
          uint8_t lowPowerEnable = optData[0];
          FOSSASAT_DEBUG_PRINT(F("lowPowerEnable="));
          FOSSASAT_DEBUG_PRINTLN(lowPowerEnable);
          PersistentStorage_Set<uint8_t>(FLASH_LOW_POWER_MODE_ENABLED, lowPowerEnable);
        }
      } break;

    case CMD_SET_MPPT_MODE: {
      // check optional data is exactly 2 bytes
      if(Communication_Check_OptDataLen(2, optDataLen)) {
        FOSSASAT_DEBUG_PRINT(F("mpptTempSwitchEnabled="));
        FOSSASAT_DEBUG_PRINTLN(optData[0]);
        PersistentStorage_Set<uint8_t>(FLASH_MPPT_TEMP_SWITCH_ENABLED, optData[0]);
        FOSSASAT_DEBUG_PRINT(F("mpptKeepAliveEnabled="));
        FOSSASAT_DEBUG_PRINTLN(optData[1]);
        PersistentStorage_Set<uint8_t>(FLASH_MPPT_KEEP_ALIVE_ENABLED, optData[1]);
      }
    } break;

    case CMD_SET_RECEIVE_WINDOWS: {
      // check optional data is exactly 2 bytes
      if(Communication_Check_OptDataLen(2, optDataLen)) {
        // set LoRa receive length
        uint8_t loraRxLen = optData[1];
        FOSSASAT_DEBUG_PRINT(F("loraRxLen="));
        FOSSASAT_DEBUG_PRINTLN(loraRxLen);
        PersistentStorage_Set(FLASH_LORA_RECEIVE_LEN, loraRxLen);

        // set FSK receive length
        uint8_t fskRxLen = optData[0];
        FOSSASAT_DEBUG_PRINT(F("fskRxLen="));
        FOSSASAT_DEBUG_PRINTLN(fskRxLen);
        PersistentStorage_Set(FLASH_FSK_RECEIVE_LEN, fskRxLen);

        // check if there will be still some receive window open
        if((PersistentStorage_Get<uint8_t>(FLASH_LORA_RECEIVE_LEN) == 0) && (PersistentStorage_Get<uint8_t>(FLASH_FSK_RECEIVE_LEN) == 0)) {
          FOSSASAT_DEBUG_PRINT(F("Request to set both lengths to 0, restoring FSK default."));
          PersistentStorage_Set(FLASH_FSK_RECEIVE_LEN, FSK_RECEIVE_WINDOW_LENGTH);
        }
      }
    } break;

    case CMD_CAMERA_CAPTURE: {
      // check optional data is exactly 4 bytes
      if(Communication_Check_OptDataLen(4, optDataLen)) {
        // get parameters
        uint8_t pictureSize = (uint8_t)((optData[1] & 0xF0) >> 4);
        uint8_t lightMode = (uint8_t)(optData[1] & 0x0F);
        uint8_t saturation = (uint8_t)((optData[2] & 0xF0) >> 4);
        uint8_t brightness = (uint8_t)(optData[2] & 0x0F);
        uint8_t contrast = (uint8_t)((optData[3] & 0xF0) >> 4);
        uint8_t special = (uint8_t)(optData[3] & 0x0F);

        // power up camera
        digitalWrite(CAMERA_POWER_FET, HIGH);

        // initialize
        uint32_t cameraState = (uint32_t)Camera_Init(pictureSize, lightMode, saturation, brightness, contrast, special);
        if(cameraState != 0) {
          // initialization failed, send the error
          digitalWrite(CAMERA_POWER_FET, LOW);
          FOSSASAT_DEBUG_PRINT(F("Camera init failed, code "));
          FOSSASAT_DEBUG_PRINTLN(cameraState);
          uint8_t respOptData[4];
          memcpy(respOptData, &cameraState, 4);
          Communication_Send_Response(RESP_CAMERA_STATE, respOptData, 4);
          return;
        }

        // take a picture
        uint32_t imgLen = Camera_Capture(optData[0]);
        digitalWrite(CAMERA_POWER_FET, LOW);
        FOSSASAT_DEBUG_PRINT_FLASH(FLASH_SYSTEM_INFO_START, 0x50)

        // send response
        uint8_t respOptData[4];
        memcpy(respOptData, &imgLen, 4);
        Communication_Send_Response(RESP_CAMERA_STATE, respOptData, 4);
      }
    } break;

    case CMD_SET_POWER_LIMITS: {
      // check optional data is exactly 17 bytes
      if(Communication_Check_OptDataLen(17, optDataLen)) {
        // print values for debugging only
        #ifdef FOSSASAT_DEBUG
        int16_t voltageLimit = 0;
        float temperatureLimit = 0;

        memcpy(&voltageLimit, optData, sizeof(int16_t));
        FOSSASAT_DEBUG_PRINT(F("deploymentVoltageLimit = "));
        FOSSASAT_DEBUG_PRINTLN(voltageLimit);

        memcpy(&voltageLimit, optData + sizeof(int16_t), sizeof(int16_t));
        FOSSASAT_DEBUG_PRINT(F("heaterBatteryLimit = "));
        FOSSASAT_DEBUG_PRINTLN(voltageLimit);

        memcpy(&voltageLimit, optData + 2*sizeof(int16_t), sizeof(int16_t));
        FOSSASAT_DEBUG_PRINT(F("cwBeepLimit = "));
        FOSSASAT_DEBUG_PRINTLN(voltageLimit);

        memcpy(&voltageLimit, optData + 3*sizeof(int16_t), sizeof(int16_t));
        FOSSASAT_DEBUG_PRINT(F("lowPowerLimit = "));
        FOSSASAT_DEBUG_PRINTLN(voltageLimit);

        memcpy(&temperatureLimit, optData + 4*sizeof(int16_t), sizeof(float));
        FOSSASAT_DEBUG_PRINT(F("heaterTempLimit = "));
        FOSSASAT_DEBUG_PRINTLN(temperatureLimit);

        memcpy(&temperatureLimit, optData + 4*sizeof(int16_t) + sizeof(float), sizeof(float));
        FOSSASAT_DEBUG_PRINT(F("mpptTempLimit = "));
        FOSSASAT_DEBUG_PRINTLN(temperatureLimit);

        FOSSASAT_DEBUG_PRINT(F("heaterDutyCycle = "));
        FOSSASAT_DEBUG_PRINTLN(optData[16]);
        #endif

        // write all at once
        PersistentStorage_Set_Buffer(FLASH_DEPLOYMENT_BATTERY_VOLTAGE_LIMIT, optData, optDataLen);
      }
    } break;

    case CMD_SET_RTC: {
      // check optional data
      if(Communication_Check_OptDataLen(7, optDataLen)) {
        FOSSASAT_DEBUG_PRINT(F("year = "));
        FOSSASAT_DEBUG_PRINTLN(optData[0]);
        FOSSASAT_DEBUG_PRINT(F("month = "));
        FOSSASAT_DEBUG_PRINTLN(optData[1]);
        FOSSASAT_DEBUG_PRINT(F("day = "));
        FOSSASAT_DEBUG_PRINTLN(optData[2]);
        FOSSASAT_DEBUG_PRINT(F("weekDay = "));
        FOSSASAT_DEBUG_PRINTLN(optData[3]);
        FOSSASAT_DEBUG_PRINT(F("hours = "));
        FOSSASAT_DEBUG_PRINTLN(optData[4]);
        FOSSASAT_DEBUG_PRINT(F("minutes = "));
        FOSSASAT_DEBUG_PRINTLN(optData[5]);
        FOSSASAT_DEBUG_PRINT(F("seconds = "));
        FOSSASAT_DEBUG_PRINTLN(optData[6]);
        rtc.setDate(optData[3], optData[2], optData[1], optData[0]);
        rtc.setTime(optData[4], optData[5], optData[6]);
        PersistentStorage_Set<uint32_t>(FLASH_RTC_EPOCH, rtc.getEpoch());
      }
    } break;

    case CMD_RECORD_IMU: {
      // check optional data
      if(Communication_Check_OptDataLen(4, optDataLen)) {
        uint8_t numSamples = optData[0];
        FOSSASAT_DEBUG_PRINT(F("numSamples="));
        FOSSASAT_DEBUG_PRINTLN(numSamples);

        // check number of samples is less than limit
        if(numSamples > 10) {
          FOSSASAT_DEBUG_PRINT(F("too much!"));
          break;
        }

        // check flags
        char device;
        if(optData[3] & 0b00000001) {
          // gyroscope
          device = 'G';
        } else if(optData[3] & 0b00000010) {
          // accelerometer
          device = 'A';
        } else if(optData[3] & 0b00000100) {
          // magnetometer
          device = 'M';
        } else {
          FOSSASAT_DEBUG_PRINTLN(F("Unknown device!"));
          break;
        }

        // get sample period
        uint16_t period = 0;
        memcpy(&period, optData + 1, 2);
        FOSSASAT_DEBUG_PRINT(F("period="));
        FOSSASAT_DEBUG_PRINTLN(period);

        uint8_t respOptDataLen = 3*sizeof(float) * numSamples;
        uint8_t respOptData[MAX_OPT_DATA_LENGTH];

        for(uint16_t i = 0; i < respOptDataLen; i += 3*sizeof(float)) {
          // check if the battery is good enough to continue
          uint32_t start = millis();
          #ifdef ENABLE_TRANSMISSION_CONTROL
          if(PersistentStorage_Get<uint8_t>(FLASH_LOW_POWER_MODE) != LOW_POWER_NONE) {
             // battery check failed, stop measurement and send what we have
             respOptDataLen = i;
             break;
          }
          #endif

          // read the requested values
          float valX = 0;
          float valY = 0;
          float valZ = 0;
          switch(device) {
            case 'G': {
              valX = imu.calcGyro(imu.gx);
              valY = imu.calcGyro(imu.gy);
              valZ = imu.calcGyro(imu.gz);
            } break;
            case 'A': {
              valX = imu.calcAccel(imu.ax);
              valY = imu.calcAccel(imu.ay);
              valZ = imu.calcAccel(imu.az);
            } break;
            case 'M': {
              valX = imu.calcMag(imu.mx);
              valY = imu.calcMag(imu.my);
              valZ = imu.calcMag(imu.mz);
            } break;
          }

          FOSSASAT_DEBUG_PRINT(valX);
          FOSSASAT_DEBUG_PRINT('\t');
          FOSSASAT_DEBUG_PRINT(valY);
          FOSSASAT_DEBUG_PRINT('\t');
          FOSSASAT_DEBUG_PRINTLN(valZ);

          memcpy(respOptData + i, &valX, sizeof(float));
          memcpy(respOptData + i + sizeof(float), &valY, sizeof(float));
          memcpy(respOptData + i + 2*sizeof(float), &valZ, sizeof(float));

          // wait for for the next measurement
          while(millis() - start < period) {
            // update IMU
            Sensors_Update_IMU();

            // pet watchdog
            PowerControl_Watchdog_Heartbeat();
          }
        }

        Communication_Send_Response(RESP_RECORDED_IMU, respOptData, respOptDataLen);
      }
    } break;

    case CMD_RUN_ADCS: {
      if(Communication_Check_OptDataLen(7, optDataLen)) {
        int8_t x = optData[0];
        FOSSASAT_DEBUG_PRINT(F("x = "));
        FOSSASAT_DEBUG_PRINTLN(x);

        int8_t y = optData[1];
        FOSSASAT_DEBUG_PRINT(F("y = "));
        FOSSASAT_DEBUG_PRINTLN(y);

        int8_t z = optData[2];
        FOSSASAT_DEBUG_PRINT(F("z = "));
        FOSSASAT_DEBUG_PRINTLN(z);

        uint32_t duration = 0;
        memcpy(&duration, optData + 3, sizeof(uint32_t));
        FOSSASAT_DEBUG_PRINT(F("duration = "));
        FOSSASAT_DEBUG_PRINTLN(duration);

        uint8_t respOptData[7];

        // clear faults
        bridgeX.getFault();
        bridgeY.getFault();
        bridgeZ.getFault();

        // set H-bridge outputs
        uint32_t start = millis();
        uint32_t elapsed = 0;
        bridgeX.drive(x);
        bridgeY.drive(y);
        bridgeZ.drive(z);
        while(millis() - start < duration) {
          // check battery
          #ifdef ENABLE_TRANSMISSION_CONTROL
          if(PersistentStorage_Get<uint8_t>(FLASH_LOW_POWER_MODE) != LOW_POWER_NONE) {
             // battery check failed, stop ADCS
             respOptData[0] = UVLO;
             respOptData[1] = UVLO;
             respOptData[2] = UVLO;
             break;
          }
          #endif

          // check faults
          respOptData[0] = bridgeX.getFault();
          if((x != 0) && (respOptData[0] != 0)) {
            break;
          }
          respOptData[1] = bridgeY.getFault();
          if((y != 0) && (respOptData[1] != 0)) {
            break;
          }
          respOptData[2] = bridgeZ.getFault();
          if((z != 0) && (respOptData[2] != 0)) {
            break;
          }

          // pet watchdog
          PowerControl_Watchdog_Heartbeat();
        }

        // stop everything
        bridgeX.stop();
        bridgeY.stop();
        bridgeZ.stop();

        // send response
        elapsed = millis() - start;
        memcpy(respOptData + 3, &elapsed, sizeof(uint32_t));
        Communication_Send_Response(RESP_ADCS_RESULT, respOptData, 7);
      }
    } break;

    case CMD_GET_PICTURE_BURST: {
      if(Communication_Check_OptDataLen(3, optDataLen)) {
        // check FSK is active
        if(currentModem != MODEM_FSK) {
          FOSSASAT_DEBUG_PRINTLN(F("FSK is required to transfer picture"));
          return;
        }

        // get the basic info
        FOSSASAT_DEBUG_PRINT(F("Reading slot: "));
        uint8_t slot = optData[0];
        FOSSASAT_DEBUG_PRINTLN(slot);
        FOSSASAT_DEBUG_PRINT(F("Starting at ID: "));
        uint16_t i = 0;
        memcpy(&i, optData + 1, sizeof(uint16_t));
        FOSSASAT_DEBUG_PRINTLN(i);
        FOSSASAT_DEBUG_PRINT(F("Starting at address: 0x"));
        uint32_t imgAddress = FLASH_IMAGES_START + slot*FLASH_IMAGE_SLOT_SIZE;
        FOSSASAT_DEBUG_PRINTLN(imgAddress, HEX);
        FOSSASAT_DEBUG_PRINT(F("Image length (bytes): "));
        uint32_t imgLen = PersistentStorage_Get_Image_Len(slot);
        FOSSASAT_DEBUG_PRINTLN(imgLen, HEX);
        if(imgLen == 0xFFFFFFFF) {
          FOSSASAT_DEBUG_PRINTLN(F("No image in that slot."));
          uint8_t respOptData[] = {0, 0, 0, 0, 0, 0};
          Communication_Send_Response(RESP_CAMERA_PICTURE, respOptData, 6);
          return;
        }

        static const uint8_t respOptDataLen = 2 + MAX_IMAGE_PACKET_LENGTH;
        uint8_t respOptData[respOptDataLen];
        for(; i < imgLen / MAX_IMAGE_PACKET_LENGTH; i++) {
          // write packet ID
          memcpy(respOptData, &i, sizeof(uint16_t));

          // send full packet
          PersistentStorage_Read(imgAddress + i*MAX_IMAGE_PACKET_LENGTH, respOptData + 2, MAX_IMAGE_PACKET_LENGTH);
          Communication_Send_Response(RESP_CAMERA_PICTURE, respOptData, respOptDataLen);
          PowerControl_Watchdog_Heartbeat();

          // check battery
          #ifdef ENABLE_TRANSMISSION_CONTROL
          if(PersistentStorage_Get<uint8_t>(FLASH_LOW_POWER_MODE) != LOW_POWER_NONE) {
            // battery check failed, stop sending data
            FOSSASAT_DEBUG_PRINTLN(F("Battery too low, stopped."));
            return;
          }
          #endif
        }

        // send the final packet (might not be full)
        uint16_t remLen = imgLen - i*MAX_IMAGE_PACKET_LENGTH;
        memcpy(respOptData, &i, sizeof(uint16_t));
        PersistentStorage_Read(imgAddress + i*MAX_IMAGE_PACKET_LENGTH, respOptData + 2, remLen);
        Communication_Send_Response(RESP_CAMERA_PICTURE, respOptData, 2 + remLen);

      }
    } break;

    case CMD_GET_FLASH_CONTENTS: {
      if(Communication_Check_OptDataLen(5, optDataLen)) {
        // get the basic info
        uint32_t addr = 0;
        memcpy(&addr, optData, sizeof(uint32_t));
        FOSSASAT_DEBUG_PRINT(F("Reading adress: 0x"));
        FOSSASAT_DEBUG_PRINTLN(addr, HEX);
        uint8_t len = optData[4];
        FOSSASAT_DEBUG_PRINT(F("Length: "));
        FOSSASAT_DEBUG_PRINTLN(len);

        if(len > MAX_OPT_DATA_LENGTH) {
          FOSSASAT_DEBUG_PRINTLN(F("Too long!"));
          return;
        }

        // read picture packet
        uint8_t respOptData[MAX_OPT_DATA_LENGTH];
        PersistentStorage_Read(addr, respOptData, len);

        // send response
        Communication_Send_Response(RESP_FLASH_CONTENTS, respOptData, len);
      }
    } break;

    case CMD_GET_PICTURE_LENGTH: {
      if(Communication_Check_OptDataLen(1, optDataLen)) {
        FOSSASAT_DEBUG_PRINT(F("Reading slot: "));
        uint8_t slot = optData[0];
        FOSSASAT_DEBUG_PRINTLN(slot);

        FOSSASAT_DEBUG_PRINT(F("Image length (bytes): "));
        uint32_t imgLen = PersistentStorage_Get_Image_Len(slot);
        FOSSASAT_DEBUG_PRINTLN(imgLen, HEX);

        static const uint8_t respOptDataLen = sizeof(uint32_t);
        uint8_t respOptData[respOptDataLen];
        memcpy(respOptData, &imgLen, sizeof(uint32_t));
        Communication_Send_Response(RESP_CAMERA_PICTURE_LENGTH, respOptData, respOptDataLen);
      }
    } break;

    case CMD_LOG_GPS: {
      if(Communication_Check_OptDataLen(4, optDataLen)) {
        // get parameters
        uint32_t duration = 0;
        memcpy(&duration, optData, sizeof(uint32_t));
        FOSSASAT_DEBUG_PRINT(F("GPS logging duration: "));
        FOSSASAT_DEBUG_PRINTLN(duration);

        // check battery
        #ifdef ENABLE_TRANSMISSION_CONTROL
        if(PersistentStorage_Get<uint8_t>(FLASH_LOW_POWER_MODE) != LOW_POWER_NONE) {
          // battery check failed
          FOSSASAT_DEBUG_PRINTLN(F("Battery too low."));
          return;
        }
        #endif

        // wipe NMEA log
        PersistentStorage_64kBlockErase(FLASH_NMEA_LOG_START);
        PowerControl_Watchdog_Heartbeat();

        // power up GPS
        digitalWrite(GPS_POWER_FET, HIGH);

        // run for the requested duration
        uint32_t start = millis();
        uint8_t buff[MAX_IMAGE_PACKET_LENGTH];
        uint16_t buffPos = sizeof(uint32_t);
        uint32_t flashPos = FLASH_NMEA_LOG_START;
        while(millis() - start < duration) {
          // read GPS data to buffer
          while(GpsSerial.available() > 0) {
            char c = GpsSerial.read();

            // check if we got line ending
            if(c != '\n') {
              // add to buffer
              buff[buffPos] = c;
              buffPos++;
            } else {
              // add timestamp
              uint32_t stamp = millis() - start;
              memcpy(buff, &stamp, sizeof(uint32_t));
              FOSSASAT_DEBUG_PRINTLN(stamp, HEX);

              // add null terminator instead of CR
              buff[buffPos - 1] = '\0';
              FOSSASAT_DEBUG_PRINTLN((char*)buff + 4);

              //  write buffer to flash
              PersistentStorage_Write(flashPos, buff, buffPos, false);
              flashPos += MAX_IMAGE_PACKET_LENGTH;
              buffPos = sizeof(uint32_t);
            }
          }

          // check battery
          PowerControl_Watchdog_Heartbeat();
          #ifdef ENABLE_TRANSMISSION_CONTROL
          if(PersistentStorage_Get<uint8_t>(FLASH_LOW_POWER_MODE) != LOW_POWER_NONE) {
            FOSSASAT_DEBUG_PRINTLN(F("Battery too low."));
            break;
          }
          #endif
        }

        // turn GPS off
        digitalWrite(GPS_POWER_FET, LOW);

        // save the number of logged bytes and send it
        uint32_t logged = flashPos - FLASH_NMEA_LOG_START;
        FOSSASAT_DEBUG_PRINT(F("Logged total of (bytes): "));
        FOSSASAT_DEBUG_PRINTLN(logged);
        PersistentStorage_Set<uint32_t>(FLASH_NMEA_LOG_LENGTH, logged);

        static const uint8_t respOptDataLen = sizeof(uint32_t);
        uint8_t respOptData[respOptDataLen];
        memcpy(respOptData, &logged, sizeof(uint32_t));
        Communication_Send_Response(RESP_GPS_LOG_LENGTH, respOptData, respOptDataLen);
      }
    } break;

    case CMD_GET_GPS_LOG: {
      if(Communication_Check_OptDataLen(4, optDataLen)) {
        // check FSK is active
        if(currentModem != MODEM_FSK) {
          FOSSASAT_DEBUG_PRINTLN(F("FSK is required to transfer GPS log"));
          return;
        }

        // get parameters
        uint32_t offset = 0;
        memcpy(&offset, optData, sizeof(uint32_t));
        FOSSASAT_DEBUG_PRINT(F("GPS log download offset: "));
        FOSSASAT_DEBUG_PRINTLN(offset);
        uint32_t addr = FLASH_NMEA_LOG_START + offset;
        FOSSASAT_DEBUG_PRINT(F("Starting from address: 0x"));
        FOSSASAT_DEBUG_PRINTLN(addr, HEX);

        // read log length from flash
        uint32_t logged = PersistentStorage_Get<uint32_t>(FLASH_NMEA_LOG_LENGTH);
        FOSSASAT_DEBUG_PRINT(F("GPS log length: "));
        FOSSASAT_DEBUG_PRINTLN(logged);
        if(logged == 0) {
          FOSSASAT_DEBUG_PRINT(F("No GPS data logged"));
          uint8_t respOptData[4] = {0, 0, 0, 0};
          Communication_Send_Response(RESP_GPS_LOG, respOptData, 4);
          return;
        }

        // read data from flash
        uint8_t respOptData[MAX_IMAGE_PACKET_LENGTH];
        for(; addr < FLASH_NMEA_LOG_START + logged; addr += MAX_IMAGE_PACKET_LENGTH) {
          // read data into buffer
          PersistentStorage_Read(addr, respOptData, MAX_IMAGE_PACKET_LENGTH);

          // get the number of bytes in log entry
          uint8_t respOptDataLen = 4 + strlen((char*)respOptData + 4);

          // send response
          Communication_Send_Response(RESP_GPS_LOG, respOptData, respOptDataLen);

          // check battery
          PowerControl_Watchdog_Heartbeat();
          #ifdef ENABLE_TRANSMISSION_CONTROL
          if(PersistentStorage_Get<uint8_t>(FLASH_LOW_POWER_MODE) != LOW_POWER_NONE) {
            FOSSASAT_DEBUG_PRINTLN(F("Battery too low."));
            return;
          }
          #endif
        }
      }
    } break;

    default:
      FOSSASAT_DEBUG_PRINT(F("Unknown function ID!"));
      return;
  }
}

int16_t Communication_Send_Response(uint8_t respId, uint8_t* optData, size_t optDataLen, bool overrideModem) {
  // get callsign from EEPROM
  uint8_t callsignLen = PersistentStorage_Get<uint8_t>(FLASH_CALLSIGN_LEN);
  char callsign[MAX_STRING_LENGTH];
  PersistentStorage_Get_Callsign(callsign, callsignLen);

  // build response frame
  uint8_t len = FCP_Get_Frame_Length(callsign, optDataLen);
  uint8_t frame[MAX_RADIO_BUFFER_LENGTH];
  int16_t state = FCP_Encode(frame, callsign, respId, optDataLen, optData);
  FOSSASAT_DEBUG_PRINT(F("Encoding state: "));
  FOSSASAT_DEBUG_PRINTLN(state);

  // send response
  return (Communication_Transmit(frame, len, overrideModem));
}

bool Communication_Check_OptDataLen(uint8_t expected, uint8_t actual) {
  if(expected != actual) {
    // received length of optional data does not match expected
    FOSSASAT_DEBUG_PRINT(F("optDataLen mismatch, exp. "));
    FOSSASAT_DEBUG_PRINT(expected);
    FOSSASAT_DEBUG_PRINT(F(" got "));
    FOSSASAT_DEBUG_PRINTLN(actual);
    return(false);
  }

  return(true);
}

int16_t Communication_Transmit(uint8_t* data, uint8_t len, bool overrideModem) {
  // check transmit enable flag
#ifdef ENABLE_TRANSMISSION_CONTROL
  uint8_t txEnabled = PersistentStorage_Get<uint8_t>(FLASH_TRANSMISSIONS_ENABLED);
  if (txEnabled == 0x00) {
    FOSSASAT_DEBUG_PRINTLN(F("Tx off by cmd"));
    return (ERR_TX_TIMEOUT);
  }
#endif

  // print frame for debugging
  FOSSASAT_DEBUG_PRINT(F("Sending frame "));
  FOSSASAT_DEBUG_PRINTLN(len);
  FOSSASAT_DEBUG_PRINT_BUFF(data, len);

  // check if modem should be switched - required for transmissions with custom settings
  uint8_t modem = currentModem;
  FOSSASAT_DEBUG_PRINT(F("Using modem "));
  if(overrideModem) {
    FOSSASAT_DEBUG_WRITE(MODEM_LORA);
    FOSSASAT_DEBUG_PRINTLN(F(" (overridden)"));
    Communication_Set_Modem(MODEM_LORA);
  } else {
    FOSSASAT_DEBUG_WRITE(modem);
    FOSSASAT_DEBUG_PRINTLN();
  }

  // get timeout
  uint32_t timeout = 0;
  if(currentModem == MODEM_FSK) {
    timeout = (float)radio.getTimeOnAir(len) * 5.0;
  } else {
    timeout = (float)radio.getTimeOnAir(len) * 1.5;
  }
  FOSSASAT_DEBUG_PRINT(F("Timeout in: "));
  FOSSASAT_DEBUG_PRINTLN(timeout);
  FOSSASAT_DEBUG_DELAY(10);

  // start transmitting
  int16_t state = radio.startTransmit(data, len);
  if (state != ERR_NONE) {
    FOSSASAT_DEBUG_PRINT(F("Tx failed "));
    FOSSASAT_DEBUG_PRINTLN(state);
    return (state);
  }

  // wait for transmission finish
  uint32_t start = micros();
  uint32_t lastBeat = 0;
  while (!digitalRead(RADIO_DIO1)) {
    // pet watchdog every second
    if (micros() - lastBeat > (uint32_t)WATCHDOG_LOOP_HEARTBEAT_PERIOD * (uint32_t)1000) {
      PowerControl_Watchdog_Heartbeat();
      lastBeat = micros();
    }

    // check timeout
    if (micros() - start > timeout) {
      // timed out while transmitting
      radio.standby();
      Communication_Set_Modem(modem);
      FOSSASAT_DEBUG_PRINT(F("Tx timeout"));
      return (ERR_TX_TIMEOUT);
    }
  }

  FOSSASAT_DEBUG_PRINT(F("Tx done in: "));
  FOSSASAT_DEBUG_PRINTLN(micros() - start);
  FOSSASAT_DEBUG_DELAY(10);

  // transmission done, set mode standby
  state = radio.standby();

  // restore modem
  if(overrideModem) {
    Communication_Set_Modem(modem);
  }

  if(modem == MODEM_FSK) {
    #ifdef ENABLE_TRANSMISSION_CONTROL
    radio.reset();
    #endif
    Communication_Set_Modem(MODEM_FSK);
  }

  return (state);
}
