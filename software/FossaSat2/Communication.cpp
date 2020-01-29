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
  PowerControl_Wait(len, true);
  radio.standby();
}

void Communication_Send_Basic_System_Info() {
  // build response frame
  static const uint8_t optDataLen = 6*sizeof(uint8_t) + 3*sizeof(int16_t) + sizeof(uint16_t);
  uint8_t optData[optDataLen];
  uint8_t* optDataPtr = optData;

  FOSSASAT_DEBUG_PRINTLN(F("--- System info: ---"));

  uint8_t mpptOutputVoltage = currSensorMPPT.readBusVoltage() * (VOLTAGE_UNIT / VOLTAGE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, mpptOutputVoltage, "batteryVoltage", VOLTAGE_MULTIPLIER, "mV");

  int16_t mpptOutputCurrent = currSensorMPPT.readCurrent() * (CURRENT_UNIT / CURRENT_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, mpptOutputCurrent, "mpptOutputCurrent", CURRENT_MULTIPLIER, "uA");

  // TODO uptime counter (4B)

  // TODO power config

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
  if((functionId >= PRIVATE_OFFSET) && (functionId < (PRIVATE_OFFSET + NUM_PRIVATE_COMMANDS))) {
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
            Communication_Send_Response(RESP_REPEATED_MESSAGE_CUSTOM, optData + 7, optDataLen - 7, true);
          }
        }
      } break;

    case CMD_TRANSMIT_SYSTEM_INFO:
      // send system info via LoRa
      Communication_Send_Basic_System_Info();
      break;

    case CMD_GET_PACKET_INFO: {
        // get last packet info and send it
        static const uint8_t respOptDataLen = 2*sizeof(uint8_t) + 4*sizeof(uint16_t);
        uint8_t respOptData[respOptDataLen];
        uint8_t* respOptDataPtr = respOptData;

        // SNR
        uint8_t snr = (uint8_t)(radio.getSNR() * 4.0);
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

    case CMD_GET_FULL_SYSTEM_INFO: {
      
    } break;

    // TODO new public frames

    case CMD_DEPLOY: {
        // run deployment sequence
        PowerControl_Deploy();

        // get deployment counter value and send it
        uint8_t attemptNumber = PersistentStorage_Get<uint8_t>(FLASH_DEPLOYMENT_COUNTER);
        Communication_Send_Response(RESP_DEPLOYMENT_STATE, &attemptNumber, 1);
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
            // TODO wipe images
            
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
      // TODO implement MPPT keep alive
    } break;

    case CMD_SET_RECEIVE_WINDOWS: {
      // check optional data is exactly 2 bytes
      if(Communication_Check_OptDataLen(2, optDataLen)) {
        // set FSK receive length
        uint8_t windowLen = optData[0];
        FOSSASAT_DEBUG_PRINT(F("fskRxLen="));
        FOSSASAT_DEBUG_PRINTLN(windowLen);
        PersistentStorage_Set(FLASH_FSK_RECEIVE_LEN, windowLen);

        // set LoRa receive length
        windowLen = optData[1];
        FOSSASAT_DEBUG_PRINT(F("loraRxLen="));
        FOSSASAT_DEBUG_PRINTLN(windowLen);
        PersistentStorage_Set(FLASH_LORA_RECEIVE_LEN, windowLen);
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

    // TODO new private frames

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
  FCP_Encode(frame, callsign, respId, optDataLen, optData);

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

  // disable receive interrupt
  radio.clearDio1Action();

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
