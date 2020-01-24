#include "Communication.h"

void Communication_Receive_Interrupt() {
  // check interrups are enabled
  if (!interruptsEnabled) {
    return;
  }

  // set flag
  dataReceived = true;
}

void Communication_Change_Modem(HardwareTimer* tmr) {
  (void)tmr;

  // check interrups are enabled
  if (!interruptsEnabled) {
    return;
  }

  // set flag
  switchModem = true;
}

void Communication_Set_Modem(uint8_t modem) {
  int16_t state = ERR_NONE;
  FOSSASAT_DEBUG_PRINT(F("Set modem "));
  FOSSASAT_DEBUG_PRINTLN(modem);

  // initialize requested modem
  switch (modem) {
    case MODEM_LORA:
      // config function for Lora is already available due to custom retransmission
      state = Communication_Set_LoRa_Configuration(LORA_BANDWIDTH,
              LORA_SPREADING_FACTOR,
              LORA_CODING_RATE,
              LORA_PREAMBLE_LENGTH,
              true,
              LORA_OUTPUT_POWER);
      break;
    case MODEM_FSK: {
        state = radio.beginFSK(CARRIER_FREQUENCY,
                               FSK_BIT_RATE,
                               FSK_FREQUENCY_DEVIATION,
                               FSK_RX_BANDWIDTH,
                               FSK_OUTPUT_POWER,
                               FSK_CURRENT_LIMIT,
                               FSK_PREAMBLE_LENGTH,
                               FSK_DATA_SHAPING);
        const uint8_t syncWordFSK[] = FSK_SYNC_WORD;
        radio.setSyncWord((uint8_t*)syncWordFSK, sizeof(syncWordFSK) / sizeof(uint8_t));
        radio.setCRC(2);
      } break;
    default:
      FOSSASAT_DEBUG_PRINT(F("Unkown modem "));
      FOSSASAT_DEBUG_PRINTLN(modem);
      return;
  }

  // handle possible error codes
  FOSSASAT_DEBUG_PRINT(F("Radio init done, code "));
  FOSSASAT_DEBUG_PRINTLN(state);
  if ((state == ERR_CHIP_NOT_FOUND) || (state == ERR_SPI_CMD_FAILED) || (state == ERR_SPI_CMD_INVALID)) {
    // radio chip was not found, restart
#ifdef ENABLE_RADIO_ERROR_RESET
    PowerControl_Watchdog_Restart();
#endif
  }

  // set TCXO
  radio.setTCXO(TCXO_VOLTAGE);
}

int16_t Communication_Set_LoRa_Configuration(float bw, uint8_t sf, uint8_t cr, uint16_t preambleLen, bool crc, int8_t power) {
  // set LoRa radio config
  int16_t state = radio.begin(CARRIER_FREQUENCY, bw, sf, cr, LORA_SYNC_WORD, power, LORA_CURRENT_LIMIT, preambleLen);
  if (state != ERR_NONE) {
    return (state);
  }

  // set CRC
  state = radio.setCRC(crc);
  return (state);
}

template <class T>
void Communication_System_Info_Add(uint8_t** buffPtr, T val, const char* name, uint32_t mult, const char* unit) {
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

template void Communication_System_Info_Add<uint8_t>(uint8_t**, uint8_t, const char*, uint32_t, const char*);
template void Communication_System_Info_Add<int16_t>(uint8_t**, int16_t, const char*, uint32_t, const char*);
template void Communication_System_Info_Add<uint16_t>(uint8_t**, uint16_t, const char*, uint32_t, const char*);

void Communication_Send_System_Info() {
  // build response frame
  static const uint8_t optDataLen = sizeof(uint8_t) + 8*sizeof(int16_t) + sizeof(uint16_t);
  uint8_t optData[optDataLen];
  uint8_t* optDataPtr = optData;

  FOSSASAT_DEBUG_PRINTLN(F("--- System info: ---"));

  // TODO add operational mode status

  uint8_t mpptOutputVoltage = currSensorMPPT.readBusVoltage() * (VOLTAGE_UNIT / VOLTAGE_MULTIPLIER);
  Communication_System_Info_Add(&optDataPtr, mpptOutputVoltage, "mpptOutputVoltage", VOLTAGE_MULTIPLIER, "mV");

  int16_t mpptOutputCurrent = currSensorMPPT.readCurrent() * (CURRENT_UNIT / CURRENT_MULTIPLIER);
  Communication_System_Info_Add(&optDataPtr, mpptOutputCurrent, "mpptOutputCurrent", CURRENT_MULTIPLIER, "uA");

  int16_t batteryTemperature = Sensors_Read_Temperature(tempSensorBattery) * (TEMPERATURE_UNIT / TEMPERATURE_MULTIPLIER);
  Communication_System_Info_Add(&optDataPtr, batteryTemperature, "batteryTemperature", TEMPERATURE_MULTIPLIER, "mdeg C");

  int16_t secBatteryTemperature = Sensors_Read_Temperature(tempSensorSecBattery) * (TEMPERATURE_UNIT / TEMPERATURE_MULTIPLIER);
  Communication_System_Info_Add(&optDataPtr, secBatteryTemperature, "secBatteryTemperature", TEMPERATURE_MULTIPLIER, "mdeg C");

  int16_t currentXA = currSensorXA.readCurrent() * (CURRENT_UNIT / CURRENT_MULTIPLIER);
  Communication_System_Info_Add(&optDataPtr, currentXA, "currentXA", CURRENT_MULTIPLIER, "uA");

  int16_t currentXB = currSensorXB.readCurrent() * (CURRENT_UNIT / CURRENT_MULTIPLIER);
  Communication_System_Info_Add(&optDataPtr, currentXB, "currentXB", CURRENT_MULTIPLIER, "uA");

  int16_t currentZA = currSensorZA.readCurrent() * (CURRENT_UNIT / CURRENT_MULTIPLIER);
  Communication_System_Info_Add(&optDataPtr, currentZA, "currentZA", CURRENT_MULTIPLIER, "uA");

  int16_t currentZB = currSensorZB.readCurrent() * (CURRENT_UNIT / CURRENT_MULTIPLIER);
  Communication_System_Info_Add(&optDataPtr, currentZB, "currentZB", CURRENT_MULTIPLIER, "uA");

  int16_t currentY = currSensorY.readCurrent() * (CURRENT_UNIT / CURRENT_MULTIPLIER);
  Communication_System_Info_Add(&optDataPtr, currentY, "currentY", CURRENT_MULTIPLIER, "uA");

  uint16_t resetCounter = 0xFFFF;
  PersistentStorage_Get(FLASH_RESTART_COUNTER_ADDR, resetCounter);
  Communication_System_Info_Add(&optDataPtr, resetCounter, "resetCounter", 1, "");

  FOSSASAT_DEBUG_PRINTLN(F("--------------------"));

  // send response
  Communication_Send_Response(RESP_SYSTEM_INFO, optData, optDataLen);
}

void Communication_Send_Morse_Beacon() {
  // check transmit enable flag
#ifdef ENABLE_TRANSMISSION_CONTROL
  uint8_t txEnabled = 0xFF;
  PersistentStorage_Get(FLASH_TRANSMISSIONS_ENABLED, txEnabled);
  if (txEnabled == 0x00) {
    FOSSASAT_DEBUG_PRINTLN(F("Tx off by cmd"));
    return;
  }
#endif

  // set modem to FSK
  uint8_t modem = currentModem;
  if (modem != MODEM_FSK) {
    Communication_Set_Modem(MODEM_FSK);
  }

  // read callsign
  uint8_t callsignLen = 0xFF;
  PersistentStorage_Get(FLASH_CALLSIGN_LEN_ADDR, callsignLen);
  char callsign[MAX_STRING_LENGTH];
  PersistentStorage_Get_Callsign(callsign, callsignLen);

  // TODO measure battery voltage
  float batt = 0;//currSensorMPPT.readBusVoltage();
  char battStr[6];
  dtostrf(batt, 5, 3, battStr);

  // build Morse beacon frame
  char morseFrame[MAX_OPT_DATA_LENGTH];
  sprintf(morseFrame, "%s %s", callsign, battStr);
  FOSSASAT_DEBUG_PRINT(F("Morse beacon data: "));
  FOSSASAT_DEBUG_PRINTLN(morseFrame);

  // send Morse data
  int16_t state = morse.begin(CARRIER_FREQUENCY, MORSE_SPEED);
  FOSSASAT_DEBUG_PRINT(F("Morse init done, code "));
  FOSSASAT_DEBUG_PRINTLN(state);
  if (state == ERR_NONE) {
    FOSSASAT_DEBUG_PRINTLN(F("Sending data"));
    FOSSASAT_DEBUG_STOPWATCH_START();

    // send preamble
    for (uint8_t i = 0; i < MORSE_PREAMBLE_LENGTH; i++) {
      morse.startSignal();
      FOSSASAT_DEBUG_PRINT('x');
      PowerControl_Watchdog_Heartbeat();
      PowerControl_Wait(100, LOW_POWER_NONE);
    }

    // print frame single symbol at a time to reset watchdog
    for (uint8_t i = 0; i < strlen(morseFrame); i++) {
      morse.print(morseFrame[i]);
      FOSSASAT_DEBUG_PRINT(morseFrame[i]);
      PowerControl_Watchdog_Heartbeat();
    }
    morse.println();
    FOSSASAT_DEBUG_PRINTLN('+');
    PowerControl_Watchdog_Heartbeat();

    FOSSASAT_DEBUG_PRINTLN(F("Done!"));
    FOSSASAT_DEBUG_STOPWATCH_STOP();
  }

  // set modem back to previous configuration
  if (modem != MODEM_FSK) {
    Communication_Set_Modem(modem);
  }
}

void Comunication_Parse_Frame(uint8_t* frame, uint8_t len) {
  // get callsign from EEPROM
  uint8_t callsignLen = 0xFF;
  PersistentStorage_Get(FLASH_CALLSIGN_LEN_ADDR, callsignLen);
  char callsign[MAX_STRING_LENGTH];
  PersistentStorage_Get_Callsign(callsign, callsignLen);

  // check callsign
  if (memcmp(frame, (uint8_t*)callsign, callsignLen - 1) != 0) {
    // check failed
    FOSSASAT_DEBUG_PRINT(F("Callsign mismatch, expected "));
    FOSSASAT_DEBUG_PRINTLN(callsign);
    return;
  }

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
  if (functionId >= PRIVATE_OFFSET) {
    // private frame, decrypt

  } else {
    // no decryption necessary

    // get optional data length
    optDataLen = FCP_Get_OptData_Length(callsign, frame, len);
    if (optDataLen < 0) {
      // optional data extraction failed,
      FOSSASAT_DEBUG_PRINT(F("Failed to get optDataLen, code "));
      FOSSASAT_DEBUG_PRINTLN(optDataLen);
      return;
    }

    // get optional data
    if (optDataLen > 0) {
      FCP_Get_OptData(callsign, frame, len, optData);
    }
  }

  // check optional data presence
  if (optDataLen > 0) {
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
  // execute function based on ID
  switch (functionId) {

    // public function IDs

    case CMD_PING:
      // send pong
      Communication_Send_Response(RESP_PONG);
      break;

    case CMD_RETRANSMIT: {
        // check message length
        if (optDataLen <= MAX_OPT_DATA_LENGTH) {
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
            Communication_Send_Response(RESP_REPEATED_MESSAGE_CUSTOM, optData + 7, optDataLen - 7, false, true);
          }
        }
      } break;

    case CMD_TRANSMIT_SYSTEM_INFO:
      // send system info via LoRa
      Communication_Send_System_Info();
      break;

    case CMD_GET_PACKET_INFO: {
        // get last packet info and send it
        uint8_t respOptData[] = {(uint8_t)(radio.getSNR() * 4.0), (uint8_t)(radio.getRSSI() * -2.0)};
        Communication_Send_Response(RESP_PACKET_INFO, respOptData, 2);
      } break;

    // TODO new public frames

    case CMD_DEPLOY: {
        // run deployment sequence
        PowerControl_Deploy();

        // get deployment counter value and send it
        uint8_t attemptNumber = 0xFF;
        PersistentStorage_Get(FLASH_DEPLOYMENT_COUNTER_ADDR, attemptNumber);
        Communication_Send_Response(RESP_DEPLOYMENT_STATE, &attemptNumber, 1, true);
      } break;

    case CMD_RESTART:
      // restart
      PowerControl_Watchdog_Restart();
      break;

    case CMD_WIPE_EEPROM: {
        // check optional data length
        if(Communication_Check_OptDataLen(1, optDataLen)) {
          // optional data present, check the value
          if(optData[0] & 0b00000001) {
            // wipe system info
            PersistentStorage_Reset_System_Info();
          }
  
          if(optData[0] & 0b00000010) {
            // wipe images
            
          }
        }
      } break;

    case CMD_SET_TRANSMIT_ENABLE: {
        // check optional data length
        if(Communication_Check_OptDataLen(1, optDataLen)) {
          uint8_t txEnabled = optData[0];
          PersistentStorage_Set(FLASH_TRANSMISSIONS_ENABLED, txEnabled);
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

    // TODO new private frames

    default:
      FOSSASAT_DEBUG_PRINT(F("Unknown function ID!"));
      return;
  }
}

int16_t Communication_Send_Response(uint8_t respId, uint8_t* optData, size_t optDataLen, bool encrypt, bool overrideModem) {
  // get callsign from EEPROM
  uint8_t callsignLen = 0xFF;
  PersistentStorage_Get(FLASH_CALLSIGN_LEN_ADDR, callsignLen);
  char callsign[MAX_STRING_LENGTH];
  PersistentStorage_Get_Callsign(callsign, callsignLen);

  // build response frame
  uint8_t len = 0;
  uint8_t frame[MAX_RADIO_BUFFER_LENGTH];
  if(encrypt) {
    len = FCP_Get_Frame_Length(callsign, optDataLen, password);
    FCP_Encode(frame, callsign, respId, optDataLen, optData, encryptionKey, password);
  } else {
    len = FCP_Get_Frame_Length(callsign, optDataLen);
    FCP_Encode(frame, callsign, respId, optDataLen, optData);
  }

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
  uint8_t txEnabled = 0xFF;
  PersistentStorage_Get(FLASH_TRANSMISSIONS_ENABLED, txEnabled);
  if (txEnabled == 0x00) {
    FOSSASAT_DEBUG_PRINTLN(F("Tx off by cmd"));
    return (ERR_TX_TIMEOUT);
  }
#endif

  // disable receive interrupt
  detachInterrupt(digitalPinToInterrupt(RADIO_DIO1));

  // print frame for debugging
  FOSSASAT_DEBUG_PRINT(F("Sending LoRa frame, len = "));
  FOSSASAT_DEBUG_PRINTLN(len);
  FOSSASAT_DEBUG_PRINT_BUFF(data, len);

  // send frame by non-ISM LoRa
  uint8_t modem = currentModem;

  // check if modem should be switched - required for transmissions with custom settings
  if (!overrideModem) {
    Communication_Set_Modem(MODEM_LORA);
  }

  // get timeout
  uint32_t timeout = (float)radio.getTimeOnAir(len) * 1.5;

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

  // transmission done, set mode standby
  state = radio.standby();
  Communication_Set_Modem(modem);

  // set receive ISR
  radio.setDio1Action(Communication_Receive_Interrupt);
  radio.startReceive();

  return (state);
}
