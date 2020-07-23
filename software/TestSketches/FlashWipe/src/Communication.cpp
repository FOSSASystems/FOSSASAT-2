#include "Communication.h"

// cppcheck-suppress unusedFunction
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
  int16_t state = radio.begin(LORA_FREQUENCY, bw, sf, cr, SYNC_WORD, power, LORA_CURRENT_LIMIT, preambleLen, TCXO_VOLTAGE);
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
        state = radio.begin(LORA_FREQUENCY,
                            LORA_BANDWIDTH,
                            LORA_SPREADING_FACTOR,
                            LORA_CODING_RATE,
                            SYNC_WORD,
                            LORA_OUTPUT_POWER,
                            LORA_PREAMBLE_LENGTH,
                            TCXO_VOLTAGE);
        radio.setCRC(true);
        radio.setCurrentLimit(LORA_CURRENT_LIMIT);
      break;
    case MODEM_FSK: {
        state = radio.beginFSK(FSK_FREQUENCY,
                               FSK_BIT_RATE,
                               FSK_FREQUENCY_DEVIATION,
                               FSK_RX_BANDWIDTH,
                               FSK_OUTPUT_POWER,
                               FSK_PREAMBLE_LENGTH,
                               TCXO_VOLTAGE);
        uint8_t syncWordFSK[2] = {SYNC_WORD, SYNC_WORD};
        radio.setSyncWord(syncWordFSK, 2);
        radio.setCRC(2);
        radio.setDataShaping(FSK_DATA_SHAPING);
        radio.setCurrentLimit(FSK_CURRENT_LIMIT);
      } break;
    default:
      FOSSASAT_DEBUG_PRINT(F("Unkown modem "));
      FOSSASAT_DEBUG_PRINTLN(modem);
      return(ERR_UNKNOWN);
  }

  radio.setWhitening(true, WHITENING_INITIAL);

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
  morse.begin(FSK_FREQUENCY, MORSE_SPEED);

  // read callsign
  uint8_t callsignLen = PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_CALLSIGN_LEN);
  char callsign[MAX_STRING_LENGTH];
  PersistentStorage_Get_Callsign(callsign, callsignLen);

  // send start signals
  for(int8_t i = 0; i < MORSE_PREAMBLE_LENGTH; i++) {
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
  static const uint8_t optDataLen = 7*sizeof(uint8_t) + 3*sizeof(int16_t) + sizeof(uint16_t) + 2*sizeof(uint32_t);
  uint8_t optData[optDataLen];
  uint8_t* optDataPtr = optData;

  FOSSASAT_DEBUG_PRINTLN(F("--- System info: ---"));

  uint8_t mpptOutputVoltage = Sensors_Current_ReadVoltage(currSensorMPPT) * (VOLTAGE_UNIT / VOLTAGE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, mpptOutputVoltage, "batteryVoltage", VOLTAGE_MULTIPLIER, "mV");

  int16_t mpptOutputCurrent = Sensors_Current_Read(currSensorMPPT) * ((CURRENT_UNIT / 1000) / CURRENT_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, mpptOutputCurrent, "mpptOutputCurrent", CURRENT_MULTIPLIER, "uA");

  uint32_t onboardTime = rtc.getEpoch();
  Communication_Frame_Add(&optDataPtr, onboardTime, "onboardTime", 1, "");

  // power config: FLASH_TRANSMISSIONS_ENABLED (0), FLASH_LOW_POWER_MODE_ENABLED (1), FLASH_LOW_POWER_MODE (2 - 4), FLASH_MPPT_TEMP_SWITCH_ENABLED (5), FLASH_MPPT_KEEP_ALIVE_ENABLED (6)
  uint8_t powerConfig = ((PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_TRANSMISSIONS_ENABLED)           & 0b00000001) |
                        ((PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_LOW_POWER_MODE_ENABLED)    << 1) & 0b00000010) |
                        ((PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_LOW_POWER_MODE)            << 2) & 0b00011100) |
                        ((PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_MPPT_TEMP_SWITCH_ENABLED)  << 5) & 0b00100000) |
                        ((PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_MPPT_KEEP_ALIVE_ENABLED)   << 6) & 0b01000000));
  Communication_Frame_Add(&optDataPtr, powerConfig, "powerConfig", 1, "");

  uint16_t resetCounter = PersistentStorage_SystemInfo_Get<uint16_t>(FLASH_RESTART_COUNTER);
  Communication_Frame_Add(&optDataPtr, resetCounter, "resetCounter", 1, "");

  uint8_t voltageXA = Sensors_Current_ReadVoltage(currSensorXA) * (VOLTAGE_UNIT / VOLTAGE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, voltageXA, "voltageXA", VOLTAGE_MULTIPLIER, "mV");

  uint8_t voltageXB = Sensors_Current_ReadVoltage(currSensorXB) * (VOLTAGE_UNIT / VOLTAGE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, voltageXB, "voltageXB", VOLTAGE_MULTIPLIER, "mV");

  uint8_t voltageZA = Sensors_Current_ReadVoltage(currSensorZA) * (VOLTAGE_UNIT / VOLTAGE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, voltageZA, "voltageZA", VOLTAGE_MULTIPLIER, "mV");

  uint8_t voltageZB = Sensors_Current_ReadVoltage(currSensorZB) * (VOLTAGE_UNIT / VOLTAGE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, voltageZB, "voltageZB", VOLTAGE_MULTIPLIER, "mV");

  uint8_t voltageY = Sensors_Current_ReadVoltage(currSensorY) * (VOLTAGE_UNIT / VOLTAGE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, voltageY, "voltageY", VOLTAGE_MULTIPLIER, "mV");

  int16_t batteryTemperature = Sensors_Temperature_Read(tempSensorBattery) * (TEMPERATURE_UNIT / TEMPERATURE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, batteryTemperature, "batteryTemperature", TEMPERATURE_MULTIPLIER, "mdeg C");

  int16_t boardTemperature = Sensors_Temperature_Read(tempSensorTop) * (TEMPERATURE_UNIT / TEMPERATURE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, boardTemperature, "boardTemperature", TEMPERATURE_MULTIPLIER, "mdeg C");

  uint32_t errCounter = PersistentStorage_SystemInfo_Get<uint32_t>(FLASH_MEMORY_ERROR_COUNTER);
  Communication_Frame_Add(&optDataPtr, errCounter, "errCounter", 1, "");

  FOSSASAT_DEBUG_PRINTLN(F("--------------------"));

  // send response
  Communication_Send_Response(RESP_SYSTEM_INFO, optData, optDataLen);
}

void Communication_Send_Full_System_Info() {
  // build response frame
  static const uint8_t optDataLen = 14*sizeof(uint8_t) + 12*sizeof(int16_t) + sizeof(uint16_t) + 2*sizeof(uint32_t) + 2*sizeof(float);
  uint8_t optData[optDataLen];
  uint8_t* optDataPtr = optData;

  FOSSASAT_DEBUG_PRINTLN(F("--- System info: ---"));

  uint8_t mpptOutputVoltage = Sensors_Current_ReadVoltage(currSensorMPPT) * (VOLTAGE_UNIT / VOLTAGE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, mpptOutputVoltage, "batteryVoltage", VOLTAGE_MULTIPLIER, "mV");

  int16_t mpptOutputCurrent = Sensors_Current_Read(currSensorMPPT) * ((CURRENT_UNIT / 1000) / CURRENT_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, mpptOutputCurrent, "mpptOutputCurrent", CURRENT_MULTIPLIER, "uA");

  uint32_t onboardTime = rtc.getEpoch();
  Communication_Frame_Add(&optDataPtr, onboardTime, "onboardTime", 1, "");

  // power config: FLASH_TRANSMISSIONS_ENABLED (0), FLASH_LOW_POWER_MODE_ENABLED (1), FLASH_LOW_POWER_MODE (2 - 4), FLASH_MPPT_TEMP_SWITCH_ENABLED (5), FLASH_MPPT_KEEP_ALIVE_ENABLED (6)
  uint8_t powerConfig = ((PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_TRANSMISSIONS_ENABLED)           & 0b00000001) |
                        ((PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_LOW_POWER_MODE_ENABLED)    << 1) & 0b00000010) |
                        ((PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_LOW_POWER_MODE)            << 2) & 0b00011100) |
                        ((PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_MPPT_TEMP_SWITCH_ENABLED)  << 5) & 0b00100000) |
                        ((PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_MPPT_KEEP_ALIVE_ENABLED)   << 6) & 0b01000000) |
                        (((uint8_t)scienceModeActive << 7) & 0b10000000));
  Communication_Frame_Add(&optDataPtr, powerConfig, "powerConfig", 1, "");

  uint16_t resetCounter = PersistentStorage_SystemInfo_Get<uint16_t>(FLASH_RESTART_COUNTER);
  Communication_Frame_Add(&optDataPtr, resetCounter, "resetCounter", 1, "");

  uint8_t voltageXA = Sensors_Current_ReadVoltage(currSensorXA) * (VOLTAGE_UNIT / VOLTAGE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, voltageXA, "voltageXA", VOLTAGE_MULTIPLIER, "mV");

  int16_t currentXA = Sensors_Current_Read(currSensorXA) * ((CURRENT_UNIT / 1000) / CURRENT_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, currentXA, "currentXA", CURRENT_MULTIPLIER, "uA");

  uint8_t voltageXB = Sensors_Current_ReadVoltage(currSensorXB) * (VOLTAGE_UNIT / VOLTAGE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, voltageXB, "voltageXB", VOLTAGE_MULTIPLIER, "mV");

  int16_t currentXB = Sensors_Current_Read(currSensorXB) * ((CURRENT_UNIT / 1000) / CURRENT_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, currentXB, "currentXB", CURRENT_MULTIPLIER, "uA");

  uint8_t voltageZA = Sensors_Current_ReadVoltage(currSensorZA) * (VOLTAGE_UNIT / VOLTAGE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, voltageZA, "voltageZA", VOLTAGE_MULTIPLIER, "mV");

  int16_t currentZA = Sensors_Current_Read(currSensorZA) * ((CURRENT_UNIT / 1000) / CURRENT_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, currentZA, "currentZA", CURRENT_MULTIPLIER, "uA");

  uint8_t voltageZB = Sensors_Current_ReadVoltage(currSensorZB) * (VOLTAGE_UNIT / VOLTAGE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, voltageZB, "voltageZB", VOLTAGE_MULTIPLIER, "mV");

  int16_t currentZB = Sensors_Current_Read(currSensorZB) * ((CURRENT_UNIT / 1000) / CURRENT_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, currentZB, "currentZB", CURRENT_MULTIPLIER, "uA");

  uint8_t voltageY = Sensors_Current_ReadVoltage(currSensorY) * (VOLTAGE_UNIT / VOLTAGE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, voltageY, "voltageY", VOLTAGE_MULTIPLIER, "mV");

  int16_t currentY = Sensors_Current_Read(currSensorY) * ((CURRENT_UNIT / 1000) / CURRENT_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, currentY, "currentY", CURRENT_MULTIPLIER, "uA");

  int16_t tempPanelY = Sensors_Temperature_Read(tempSensorPanelY) * (TEMPERATURE_UNIT / TEMPERATURE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, tempPanelY, "tempPanelY", TEMPERATURE_MULTIPLIER, "mdeg C");

  int16_t boardTemperature = Sensors_Temperature_Read(tempSensorTop) * (TEMPERATURE_UNIT / TEMPERATURE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, boardTemperature, "boardTemperature", TEMPERATURE_MULTIPLIER, "mdeg C");

  int16_t tempBottom = Sensors_Temperature_Read(tempSensorBottom) * (TEMPERATURE_UNIT / TEMPERATURE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, tempBottom, "tempBottom", TEMPERATURE_MULTIPLIER, "mdeg C");

  int16_t batteryTemperature = Sensors_Temperature_Read(tempSensorBattery) * (TEMPERATURE_UNIT / TEMPERATURE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, batteryTemperature, "batteryTemperature", TEMPERATURE_MULTIPLIER, "mdeg C");

  int16_t secBatteryTemperature = Sensors_Temperature_Read(tempSensorSecBattery) * (TEMPERATURE_UNIT / TEMPERATURE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, secBatteryTemperature, "secBatteryTemperature", TEMPERATURE_MULTIPLIER, "mdeg C");

  int16_t mcuTemperature = Sensors_Temperature_Read(tempSensorMCU) * (TEMPERATURE_UNIT / TEMPERATURE_MULTIPLIER);
  Communication_Frame_Add(&optDataPtr, mcuTemperature, "mcuTemperature", TEMPERATURE_MULTIPLIER, "mdeg C");

  float lightPanelY = Sensors_Read_Light(lightSensorPanelY);
  Communication_Frame_Add(&optDataPtr, lightPanelY, "lightPanelY", 1, "lux");

  float lightTop = Sensors_Read_Light(lightSensorTop);
  Communication_Frame_Add(&optDataPtr, lightTop, "lightTop", 1, "lux");

  uint8_t bridgeXfault = bridgeX.getFault();
  Communication_Frame_Add(&optDataPtr, bridgeXfault, "bridgeXfault", 1, "");

  uint8_t bridgeYfault = bridgeY.getFault();
  Communication_Frame_Add(&optDataPtr, bridgeYfault, "bridgeYfault", 1, "");

  uint8_t bridgeZfault = bridgeZ.getFault();
  Communication_Frame_Add(&optDataPtr, bridgeZfault, "bridgeZfault", 1, "");

  uint32_t errCounter = PersistentStorage_SystemInfo_Get<uint32_t>(FLASH_MEMORY_ERROR_COUNTER);
  Communication_Frame_Add(&optDataPtr, errCounter, "errCounter", 1, "");

  uint8_t fskRxLen = PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_FSK_RECEIVE_LEN);
  Communication_Frame_Add(&optDataPtr, fskRxLen, "fskRxLen", 1, "");

  uint8_t loraRxLen = PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_LORA_RECEIVE_LEN);
  Communication_Frame_Add(&optDataPtr, loraRxLen, "loraRxLen", 1, "");

  uint8_t sensorStates = (uint8_t)currSensorXA.available << 7 | (uint8_t)currSensorXB.available << 6 |
                         (uint8_t)currSensorZA.available << 5 | (uint8_t)currSensorZB.available << 4 |
                         (uint8_t)currSensorY.available << 3 | (uint8_t)currSensorMPPT.available << 2 |
                         (uint8_t)lightSensorPanelY.available << 1 | (uint8_t)lightSensorTop.available << 0;
  Communication_Frame_Add(&optDataPtr, sensorStates, "sensors", 1, "");

  uint8_t adcsResult = PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_LAST_ADCS_RESULT);
  Communication_Frame_Add(&optDataPtr, adcsResult, "adcsResult", 1, "");

  FOSSASAT_DEBUG_PRINTLN(F("--------------------"));

  // send response
  Communication_Send_Response(RESP_FULL_SYSTEM_INFO, optData, optDataLen);
}

void Communication_Send_Statistics(uint8_t flags) {
  // response will have maximum of 217 bytes if all stats are included
  uint8_t respOptData[217];
  uint8_t respOptDataLen = 1;
  uint8_t* respOptDataPtr = respOptData;

  // copy stat flags
  memcpy(respOptDataPtr, &flags, sizeof(uint8_t));
  respOptDataPtr += sizeof(uint8_t);

  if(flags & STATS_FLAGS_TEMPERATURES) {
    // temperatures
    PersistentStorage_Read(FLASH_STATS_TEMP_PANEL_Y, respOptDataPtr, 15*sizeof(int16_t));
    respOptDataPtr += 15*sizeof(int16_t);
    respOptDataLen += 15*sizeof(int16_t);
  }

  if(flags & STATS_FLAGS_CURRENTS) {
    // currents
    PersistentStorage_Read(FLASH_STATS_CURR_XA, respOptDataPtr, 18*sizeof(int16_t));
    respOptDataPtr += 18*sizeof(int16_t);
    respOptDataLen += 18*sizeof(int16_t);
  }

  if(flags & STATS_FLAGS_VOLTAGES) {
    // voltages
    PersistentStorage_Read(FLASH_STATS_VOLT_XA, respOptDataPtr, 18*sizeof(uint8_t));
    respOptDataPtr += 18*sizeof(uint8_t);
    respOptDataLen += 18*sizeof(uint8_t);
  }

  if(flags & STATS_FLAGS_LIGHT) {
    // lights
    PersistentStorage_Read(FLASH_STATS_LIGHT_PANEL_Y, respOptDataPtr, 6*sizeof(float));
    respOptDataPtr += 6*sizeof(float);
    respOptDataLen += 6*sizeof(float);
  }

  if(flags & STATS_FLAGS_IMU) {
    // IMU
    PersistentStorage_Read(FLASH_STATS_GYRO_X, respOptDataPtr, 27*sizeof(float));
    respOptDataPtr += 27*sizeof(float);
    respOptDataLen += 27*sizeof(float);
  }

  Communication_Send_Response(RESP_STATISTICS, respOptData, respOptDataLen);
}

template <typename T>
// cppcheck-suppress unusedFunction
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

void Communication_Set_ADCS_Param(uint8_t** optDataPtr, uint8_t* adcsPage, uint32_t addr) {
  float f;
  memcpy(&f, *optDataPtr, sizeof(float));
  (*optDataPtr) += sizeof(float);
  ADCS_CALC_TYPE val = (ADCS_CALC_TYPE)f;
  memcpy(adcsPage + (addr - FLASH_ADCS_PARAMETERS), &val, sizeof(ADCS_CALC_TYPE));
}

void Communication_Transfer_Picture(uint32_t imgAddress, uint32_t imgLen, uint16_t packetId, uint8_t respId) {
  // wait a bit before sending the first packet
  PowerControl_Wait(RESPONSE_DELAY, LOW_POWER_NONE);

  static const uint8_t respOptDataLen = 2 + MAX_IMAGE_PAYLOAD_LENGTH + IMAGE_PACKET_FEC_LENGTH;
  uint8_t respOptData[respOptDataLen];
  for(; packetId < imgLen / MAX_IMAGE_PAYLOAD_LENGTH; packetId++) {
    // write packet ID
    memcpy(respOptData, &packetId, sizeof(uint16_t));

    // read data
    PersistentStorage_Read(imgAddress + packetId*MAX_IMAGE_PAYLOAD_LENGTH, respOptData + 2, MAX_IMAGE_PAYLOAD_LENGTH);

    // add FEC
    encode_rs_8(respOptData + 2, respOptData + 2 + MAX_IMAGE_PAYLOAD_LENGTH, RS8_FULL_PACKET_LENGTH - IMAGE_PACKET_FEC_LENGTH - MAX_IMAGE_PAYLOAD_LENGTH);

    // send response
    Communication_Send_Response(respId, respOptData, respOptDataLen);
    PowerControl_Watchdog_Heartbeat();

    // check battery
    #ifdef ENABLE_TRANSMISSION_CONTROL
    if(PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_LOW_POWER_MODE) != LOW_POWER_NONE) {
      // battery check failed, stop sending data
      FOSSASAT_DEBUG_PRINTLN(F("Battery too low, stopped."));
      return;
    }
    #endif
  }

  // send the final packet (might not be full)
  uint16_t remLen = imgLen - packetId*MAX_IMAGE_PAYLOAD_LENGTH;
  memcpy(respOptData, &packetId, sizeof(uint16_t));

  // read data
  PersistentStorage_Read(imgAddress + packetId*MAX_IMAGE_PAYLOAD_LENGTH, respOptData + 2, remLen);

  // add FEC
  encode_rs_8(respOptData + 2, respOptData + 2 + remLen, RS8_FULL_PACKET_LENGTH - IMAGE_PACKET_FEC_LENGTH - remLen);

  // send response
  Communication_Send_Response(respId, respOptData, 2 + remLen + IMAGE_PACKET_FEC_LENGTH);
}

void Communication_Check_New_Packet() {
  if(digitalRead(RADIO_DIO1)) {
    radio.standby();
    Communication_Process_Packet();
    radio.startReceive();
  }
}

void Communication_Acknowledge(uint8_t functionId, uint8_t result) {
  uint8_t optData[] = { functionId, result };
  Communication_Send_Response(RESP_ACKNOWLEDGE, optData, 2);
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
    uint8_t callsignLen = PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_CALLSIGN_LEN);
    char callsign[MAX_STRING_LENGTH];
    PersistentStorage_Get_Callsign(callsign, callsignLen);
    if(memcmp(frame, (uint8_t*)callsign, callsignLen - 1) == 0) {
      // check passed
      Comunication_Parse_Frame(frame, len);
    } else {
      FOSSASAT_DEBUG_PRINTLN(F("Callsign mismatch!"));
      PersistentStorage_Increment_Frame_Counter(false);
      Communication_Acknowledge(0xFF, 0x01);
    }

  } else {
    FOSSASAT_DEBUG_PRINT(F("Reception failed, code "));
    FOSSASAT_DEBUG_PRINT(state);
    PersistentStorage_Increment_Frame_Counter(false);
    Communication_Acknowledge(0xFF, 0x02);
  }

  // reset flag
  dataReceived = false;

  // enable interrupts
  interruptsEnabled = true;
}

void Comunication_Parse_Frame(uint8_t* frame, uint8_t len) {
  // get callsign from EEPROM
  uint8_t callsignLen = PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_CALLSIGN_LEN);
  char callsign[MAX_STRING_LENGTH];
  PersistentStorage_Get_Callsign(callsign, callsignLen);

  // get functionID
  int16_t functionId = FCP_Get_FunctionID(callsign, frame, len);
  if (functionId < 0) {
    FOSSASAT_DEBUG_PRINT(F("Unable to get function ID 0x"));
    FOSSASAT_DEBUG_PRINTLN(functionId, HEX);
    Communication_Acknowledge(0xFF, 0x03);
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
      Communication_Acknowledge(0xFF, 0x04);
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
      Communication_Acknowledge(0xFF, 0x05);
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
    Communication_Acknowledge(0xFF, 0x06);
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

  // check science mode flag
  if(scienceModeActive) {
    // we're in science mode, allow only whitelisted commands
    const uint8_t whitelist[] = SCIENCE_MODE_CMD_WHITELIST;
    bool idAllowed = false;
    for(uint8_t i = 0; i < sizeof(whitelist); i++) {
      if(functionId == whitelist[i]) {
        idAllowed = true;
      }
    }

    // check if the command is on whitelist
    if(!idAllowed) {
      return;
    }
  }

  // acknowledge frame
  Communication_Acknowledge(functionId, 0x00);

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

        uint16_t loraValid = PersistentStorage_SystemInfo_Get<uint16_t>(FLASH_LORA_VALID_COUNTER);
        Communication_Frame_Add(&respOptDataPtr, loraValid, "LoRa valid", 1, "");

        uint16_t loraInvalid = PersistentStorage_SystemInfo_Get<uint16_t>(FLASH_LORA_INVALID_COUNTER);
        Communication_Frame_Add(&respOptDataPtr, loraInvalid, "LoRa invalid", 1, "");

        uint16_t fskValid = PersistentStorage_SystemInfo_Get<uint16_t>(FLASH_FSK_VALID_COUNTER);
        Communication_Frame_Add(&respOptDataPtr, fskValid, "FSK valid", 1, "");

        uint16_t fskInvalid = PersistentStorage_SystemInfo_Get<uint16_t>(FLASH_FSK_INVALID_COUNTER);
        Communication_Frame_Add(&respOptDataPtr, fskInvalid, "FSK invalid", 1, "");

        Communication_Send_Response(RESP_PACKET_INFO, respOptData, respOptDataLen);
      } break;

    case CMD_GET_STATISTICS: {
      if(Communication_Check_OptDataLen(1, optDataLen)) {
        // check FSK is active
        if((PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_FSK_ONLY_ENABLED) != 0x00) && (currentModem != MODEM_FSK)) {
          FOSSASAT_DEBUG_PRINTLN(F("FSK is required to get stats"));
          return;
        }

        Communication_Send_Statistics(optData[0]);
      }
    } break;

    case CMD_GET_FULL_SYSTEM_INFO: {
      // check FSK is active
      if((PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_FSK_ONLY_ENABLED) != 0x00) && (currentModem != MODEM_FSK)) {
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
        uint16_t storageLen = PersistentStorage_SystemInfo_Get<uint16_t>(FLASH_STORE_AND_FORWARD_LENGTH);
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
          PersistentStorage_SystemInfo_Set<uint16_t>(FLASH_STORE_AND_FORWARD_LENGTH, slotNum);
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
        uint16_t storageLen = PersistentStorage_SystemInfo_Get<uint16_t>(FLASH_STORE_AND_FORWARD_LENGTH);
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

    case CMD_REQUEST_PUBLIC_PICTURE: {
      // check FSK is active
      if((PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_FSK_ONLY_ENABLED) != 0x00) && (currentModem != MODEM_FSK)) {
        FOSSASAT_DEBUG_PRINTLN(F("FSK is required to transfer picture"));
        return;
      }

      if(Communication_Check_OptDataLen(3, optDataLen)) {
        // get the basic info
        FOSSASAT_DEBUG_PRINT(F("Reading slot: "));
        uint8_t slot = optData[0];
        FOSSASAT_DEBUG_PRINTLN(slot);

        if((slot < FLASH_PUBLIC_PICTURES_START) || (slot > FLASH_PUBLIC_PICTURES_END)) {
          FOSSASAT_DEBUG_PRINT(F("Private slot!"));
          uint8_t respOptData[] = {0, 0, 0, 0, 0, 0};
          Communication_Send_Response(RESP_PUBLIC_PICTURE, respOptData, 6);
          return;
        }

        FOSSASAT_DEBUG_PRINT(F("Starting at ID: "));
        uint16_t packetId = 0;
        memcpy(&packetId, optData + 1, sizeof(uint16_t));
        FOSSASAT_DEBUG_PRINTLN(packetId);
        FOSSASAT_DEBUG_PRINT(F("Starting at address: 0x"));
        uint32_t imgAddress = FLASH_IMAGES_START + slot*FLASH_IMAGE_SLOT_SIZE;
        FOSSASAT_DEBUG_PRINTLN(imgAddress, HEX);
        FOSSASAT_DEBUG_PRINT(F("Image length (bytes): "));
        uint32_t imgLen = PersistentStorage_Get_Image_Len(slot);
        FOSSASAT_DEBUG_PRINTLN(imgLen, HEX);
        if(imgLen == 0xFFFFFFFF) {
          FOSSASAT_DEBUG_PRINTLN(F("No image in that slot."));
          uint8_t respOptData[] = {0, 0, 0, 0, 0, 0};
          Communication_Send_Response(RESP_PUBLIC_PICTURE, respOptData, 6);
          return;
        }

        // send the data
        Communication_Transfer_Picture(imgAddress, imgLen, packetId, RESP_PUBLIC_PICTURE);
      }
    } break;

    // private frames below this line

    case CMD_DEPLOY: {
        // run deployment sequence
        PowerControl_Deploy();

        // get deployment counter value and send it
        uint8_t attemptNumber = PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_DEPLOYMENT_COUNTER);
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
            FOSSASAT_DEBUG_PRINTLN(F("Resetting stats"));
            PersistentStorage_Reset_Stats();
            PowerControl_Watchdog_Heartbeat();
          }

          if(optData[0] & 0b00000100) {
            // wipe store & forward
            FOSSASAT_DEBUG_PRINTLN(F("Wiping store & forward"));
            PersistentStorage_64kBlockErase(FLASH_STORE_AND_FORWARD_START);
            PowerControl_Watchdog_Heartbeat();

            // reset store & forward length
            PersistentStorage_SystemInfo_Set<uint32_t>(FLASH_STORE_AND_FORWARD_LENGTH, 0);
          }

          if(optData[0] & 0b00001000) {
            // wipe NMEA
            FOSSASAT_DEBUG_PRINTLN(F("Wiping NMEA storage"));
            for(uint32_t addr = FLASH_NMEA_LOG_START; addr < FLASH_NMEA_LOG_END; addr += FLASH_64K_BLOCK_SIZE) {
              PersistentStorage_64kBlockErase(addr);
              PowerControl_Watchdog_Heartbeat();
            }

            // reset NMEA log length, latest entry and latest fix
            PersistentStorage_SystemInfo_Set<uint32_t>(FLASH_NMEA_LOG_LENGTH, 0);
            PersistentStorage_SystemInfo_Set<uint32_t>(FLASH_NMEA_LOG_LATEST_ENTRY, FLASH_NMEA_LOG_START);
            PersistentStorage_SystemInfo_Set<uint32_t>(FLASH_NMEA_LOG_LATEST_FIX, 0);
          }

          if(optData[0] & 0b00010000) {
            // command with long execution time - enable reception
            radio.startReceive();

            // wipe image lengths
            FOSSASAT_DEBUG_PRINTLN(F("Wiping image lengths"));
            for(uint8_t i = 0; i < 6; i++) {
              PersistentStorage_SectorErase(FLASH_IMAGE_PROPERTIES + i*FLASH_SECTOR_SIZE);
              PowerControl_Watchdog_Heartbeat();

              // check new packets
              Communication_Check_New_Packet();
              if(abortExecution) {
                abortExecution = false;
                return;
              }
            }

            // wipe all 64k image blocks
            FOSSASAT_DEBUG_PRINTLN(F("Wiping images (will take about 3 minutes)"));
            for(uint32_t addr = FLASH_IMAGES_START; addr < FLASH_CHIP_SIZE; addr += FLASH_64K_BLOCK_SIZE) {
              PersistentStorage_64kBlockErase(addr);
              PowerControl_Watchdog_Heartbeat();

              // check new packets
              Communication_Check_New_Packet();
              if(abortExecution) {
                abortExecution = false;
                return;
              }
            }
            FOSSASAT_DEBUG_PRINTLN(F("Image wipe done"));
            radio.standby();
          }

          if(optData[0] & 0b00100000) {
            // reset ADCS parameters
            FOSSASAT_DEBUG_PRINTLN(F("Resetting ADCS parameters"));
            PersistentStorage_Reset_ADCS_Params();
            PowerControl_Watchdog_Heartbeat();
          }

          if(optData[0] & 0b01000000) {
            // wipe ephemerides data storage
            FOSSASAT_DEBUG_PRINTLN(F("Wiping ephemerides storage"));
            for(uint32_t addr = FLASH_ADCS_EPHEMERIDES_START; addr < FLASH_ADCS_EPHEMERIDES_END; addr += FLASH_64K_BLOCK_SIZE) {
              PersistentStorage_64kBlockErase(addr);
              PowerControl_Watchdog_Heartbeat();
            }
          }
        }
      } break;

    case CMD_SET_TRANSMIT_ENABLE: {
        // check optional data length
        if(Communication_Check_OptDataLen(2, optDataLen)) {
          PersistentStorage_SystemInfo_Set<uint8_t>(FLASH_TRANSMISSIONS_ENABLED, optData[0]);
          PersistentStorage_SystemInfo_Set<uint8_t>(FLASH_AUTO_STATISTICS, optData[1]);
          PersistentStorage_SystemInfo_Set<uint8_t>(FLASH_FSK_ONLY_ENABLED, optData[2]);
        }
      } break;

    case CMD_SET_CALLSIGN: {
        // check optional data is less than limit
        if(optDataLen < MAX_STRING_LENGTH) {
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
          PersistentStorage_SystemInfo_Set<uint8_t>(FLASH_LOW_POWER_MODE_ENABLED, lowPowerEnable);
        }
      } break;

    case CMD_SET_MPPT_MODE: {
      // check optional data is exactly 2 bytes
      if(Communication_Check_OptDataLen(2, optDataLen)) {
        FOSSASAT_DEBUG_PRINT(F("mpptTempSwitchEnabled="));
        FOSSASAT_DEBUG_PRINTLN(optData[0]);
        PersistentStorage_SystemInfo_Set<uint8_t>(FLASH_MPPT_TEMP_SWITCH_ENABLED, optData[0]);
        FOSSASAT_DEBUG_PRINT(F("mpptKeepAliveEnabled="));
        FOSSASAT_DEBUG_PRINTLN(optData[1]);
        PersistentStorage_SystemInfo_Set<uint8_t>(FLASH_MPPT_KEEP_ALIVE_ENABLED, optData[1]);
      }
    } break;

    case CMD_SET_RECEIVE_WINDOWS: {
      // check optional data is exactly 2 bytes
      if(Communication_Check_OptDataLen(2, optDataLen)) {
        // set LoRa receive length
        uint8_t loraRxLen = optData[1];
        FOSSASAT_DEBUG_PRINT(F("loraRxLen="));
        FOSSASAT_DEBUG_PRINTLN(loraRxLen);
        PersistentStorage_SystemInfo_Set<uint8_t>(FLASH_LORA_RECEIVE_LEN, loraRxLen);

        // set FSK receive length
        uint8_t fskRxLen = optData[0];
        FOSSASAT_DEBUG_PRINT(F("fskRxLen="));
        FOSSASAT_DEBUG_PRINTLN(fskRxLen);
        PersistentStorage_SystemInfo_Set<uint8_t>(FLASH_FSK_RECEIVE_LEN, fskRxLen);

        // check if there will be still some receive window open
        if((PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_LORA_RECEIVE_LEN) == 0) && (PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_FSK_RECEIVE_LEN) == 0)) {
          FOSSASAT_DEBUG_PRINT(F("Request to set both lengths to 0, restoring FSK default."));
          PersistentStorage_SystemInfo_Set<uint8_t>(FLASH_FSK_RECEIVE_LEN, FSK_RECEIVE_WINDOW_LENGTH);
        }
      }
    } break;

    // TODO implement CMD_RECORD_SOLAR_CELLS?

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
        digitalWrite(CAMERA_POWER_FET, POWER_FET_POLARITY_ON);

        // initialize
        uint32_t cameraState = (uint32_t)Camera_Init((JPEG_Size)pictureSize, (Light_Mode)lightMode, (Color_Saturation)saturation, (Brightness)brightness, (Contrast)contrast, (Special_Effects)special);
        if(cameraState != 0) {
          // initialization failed, send the error
          digitalWrite(CAMERA_POWER_FET, POWER_FET_POLARITY_OFF);
          FOSSASAT_DEBUG_PRINT(F("Camera init failed, code "));
          FOSSASAT_DEBUG_PRINTLN(cameraState);
          uint8_t respOptData[4];
          memcpy(respOptData, &cameraState, 4);
          Communication_Send_Response(RESP_CAMERA_STATE, respOptData, 4);
          return;
        }

        // take a picture
        uint32_t imgLen = Camera_Capture(optData[0]);
        digitalWrite(CAMERA_POWER_FET, POWER_FET_POLARITY_OFF);
        FOSSASAT_DEBUG_PRINT_FLASH(FLASH_IMAGE_PROPERTIES + (optData[0]/FLASH_IMAGE_PROPERTIES_SLOT_SIZE) * FLASH_SECTOR_SIZE, FLASH_EXT_PAGE_SIZE);

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
        memcpy(systemInfoBuffer + FLASH_DEPLOYMENT_BATTERY_VOLTAGE_LIMIT, optData, optDataLen);
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
        PersistentStorage_SystemInfo_Set<uint32_t>(FLASH_RTC_EPOCH, rtc.getEpoch());
      }
    } break;

    case CMD_RECORD_IMU: {
      // check FSK is active
      if((PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_FSK_ONLY_ENABLED) != 0x00) && (currentModem != MODEM_FSK)) {
        FOSSASAT_DEBUG_PRINTLN(F("FSK is required to record IMU"));
        return;
      }

      // check optional data
      if(Communication_Check_OptDataLen(5, optDataLen)) {
        // get number of samples
        uint16_t numSamples = 0;
        memcpy(&numSamples, optData, sizeof(uint16_t));
        FOSSASAT_DEBUG_PRINT(F("numSamples="));
        FOSSASAT_DEBUG_PRINTLN(numSamples);

        // get sample period
        uint16_t period = 0;
        memcpy(&period, optData + sizeof(uint16_t), sizeof(uint16_t));
        FOSSASAT_DEBUG_PRINT(F("period="));
        FOSSASAT_DEBUG_PRINTLN(period);

        // get flags
        uint8_t flags = optData[4];
        FOSSASAT_DEBUG_PRINT(F("flags="));
        FOSSASAT_DEBUG_PRINTLN(flags, HEX);

        // calculate response optional data length
        uint8_t respOptDataLen = 1;
        while(flags) {
          respOptDataLen += 3*sizeof(float);
          flags >>= 1;
        }

        // restore flags
        flags = optData[4];

        // prepare response buffer
        uint8_t respOptData[MAX_OPT_DATA_LENGTH];
        uint8_t* respOptDataPtr = respOptData;

        // set flags
        *respOptDataPtr++ = flags;

        // wait for a bit before measurement
        PowerControl_Wait(RESPONSE_DELAY, LOW_POWER_NONE);

        // get initial IMU update
        Sensors_IMU_Update();

        // command with long execution time - enable reception
        radio.startReceive();

        // measure all samples
        for(uint16_t i = 0; i < numSamples; i++) {
          // check if the battery is good enough to continue
          uint32_t start = millis();
          #ifdef ENABLE_TRANSMISSION_CONTROL
          if(PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_LOW_POWER_MODE) != LOW_POWER_NONE) {
             // battery check failed, stop measurement
             break;
          }
          #endif

          // read the values
          float val[3];

          // gyroscope
          if(flags & 0x01) {
            Sensors_IMU_CalcGyro(imu.gx, imu.gy, imu.gz, FLASH_IMU_OFFSET_GYRO_X, val);
            Communication_Frame_Add(&respOptDataPtr, val[0], "G X", 1, "rad/s");
            Communication_Frame_Add(&respOptDataPtr, val[1], "G Y", 1, "rad/s");
            Communication_Frame_Add(&respOptDataPtr, val[2], "G Z", 1, "rad/s");
          }

          // accelerometer
          if(flags & 0x02) {
            Sensors_IMU_CalcAccel(imu.ax, imu.ay, imu.az, FLASH_IMU_OFFSET_ACCEL_X, val);
            Communication_Frame_Add(&respOptDataPtr, val[0], "A X", 1, "m/s^2");
            Communication_Frame_Add(&respOptDataPtr, val[1], "A Y", 1, "m/s^2");
            Communication_Frame_Add(&respOptDataPtr, val[2], "A Z", 1, "m/s^2");
          }

          // magnetometer
          if(flags & 0x04) {
            Sensors_IMU_CalcMag(imu.mx, imu.my, imu.mz, FLASH_IMU_OFFSET_MAG_Z, val);
            Communication_Frame_Add(&respOptDataPtr, val[0], "M X", 1, "Tesla");
            Communication_Frame_Add(&respOptDataPtr, val[1], "M Y", 1, "Tesla");
            Communication_Frame_Add(&respOptDataPtr, val[2], "M Z", 1, "Tesla");
          }

          // send the sample
          radio.standby();
          Communication_Send_Response(RESP_RECORDED_IMU, respOptData, respOptDataLen);
          radio.startReceive();

          // reset pointer
          respOptDataPtr = respOptData + 1;

          // wait for for the next measurement
          while(millis() - start < period) {
            // update IMU
            Sensors_IMU_Update();

            // pet watchdog
            PowerControl_Watchdog_Heartbeat();

            // check new packets
            Communication_Check_New_Packet();
            if(abortExecution) {
              abortExecution = false;
              return;
            }
          }
        }
      }
    } break;

    case CMD_RUN_MANUAL_ACS: {
      if(Communication_Check_OptDataLen(19, optDataLen)) {
        int8_t xHigh = optData[0];
        FOSSASAT_DEBUG_PRINT(F("x high = "));
        FOSSASAT_DEBUG_PRINTLN(xHigh);

        int8_t xLow = optData[1];
        FOSSASAT_DEBUG_PRINT(F("x low = "));
        FOSSASAT_DEBUG_PRINTLN(xLow);

        int8_t yHigh = optData[2];
        FOSSASAT_DEBUG_PRINT(F("y high = "));
        FOSSASAT_DEBUG_PRINTLN(yHigh);

        int8_t yLow = optData[3];
        FOSSASAT_DEBUG_PRINT(F("y low = "));
        FOSSASAT_DEBUG_PRINTLN(yLow);

        int8_t zHigh = optData[4];
        FOSSASAT_DEBUG_PRINT(F("z high = "));
        FOSSASAT_DEBUG_PRINTLN(zHigh);

        int8_t zLow = optData[5];
        FOSSASAT_DEBUG_PRINT(F("z low = "));
        FOSSASAT_DEBUG_PRINTLN(zLow);

        uint32_t xLen = 0;
        memcpy(&xLen, optData + 6, sizeof(uint32_t));
        FOSSASAT_DEBUG_PRINT(F("x pulse len = "));
        FOSSASAT_DEBUG_PRINTLN(xLen);

        uint32_t yLen = 0;
        memcpy(&yLen, optData + 6 + sizeof(uint32_t), sizeof(uint32_t));
        FOSSASAT_DEBUG_PRINT(F("y pulse len = "));
        FOSSASAT_DEBUG_PRINTLN(yLen);

        uint32_t zLen = 0;
        memcpy(&zLen, optData + 6 + 2*sizeof(uint32_t), sizeof(uint32_t));
        FOSSASAT_DEBUG_PRINT(F("z pulse len = "));
        FOSSASAT_DEBUG_PRINTLN(zLen);

        uint32_t duration = 0;
        memcpy(&duration, optData + 6 + 3*sizeof(uint32_t), sizeof(uint32_t));
        FOSSASAT_DEBUG_PRINT(F("duration = "));
        FOSSASAT_DEBUG_PRINTLN(duration);

        uint8_t ignoreFlags = optData[18];
        FOSSASAT_DEBUG_PRINT(F("ignoreFlags = 0x"));
        FOSSASAT_DEBUG_PRINTLN(ignoreFlags, HEX);

        uint8_t respOptData[7];

        // command with long execution time - enable reception
        radio.startReceive();

        // clear faults
        bridgeX.getFault();
        bridgeY.getFault();
        bridgeZ.getFault();

        // run for the requested duration
        uint32_t start = millis();
        uint32_t elapsed = 0;
        uint32_t lastUpdateX = 0;
        uint32_t lastUpdateY = 0;
        uint32_t lastUpdateZ = 0;
        bool bridgeHighX = false;
        bool bridgeHighY = false;
        bool bridgeHighZ = false;
        while(millis() - start < duration) {
          // set H-bridge outputs
          if(millis() - lastUpdateX >= xLen) {
            if(bridgeHighX) {
              bridgeX.drive(xLow);
            } else {
              bridgeX.drive(xHigh);
            }
            bridgeHighX = !bridgeHighX;
            lastUpdateX = millis();
          }

          if(millis() - lastUpdateY >= yLen) {
            if(bridgeHighY) {
              bridgeY.drive(yLow);
            } else {
              bridgeY.drive(yHigh);
            }
            bridgeHighY = !bridgeHighY;
            lastUpdateY = millis();
          }

          if(millis() - lastUpdateZ >= zLen) {
            if(bridgeHighZ) {
              bridgeZ.drive(zLow);
            } else {
              bridgeZ.drive(zHigh);
            }
            bridgeHighZ = !bridgeHighZ;
            lastUpdateZ = millis();
          }

          // check battery
          #ifdef ENABLE_TRANSMISSION_CONTROL
          if(PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_LOW_POWER_MODE) != LOW_POWER_NONE) {
             // battery check failed, stop ADCS
             respOptData[0] = UVLO;
             respOptData[1] = UVLO;
             respOptData[2] = UVLO;
             break;
          }
          #endif

          // check faults
          respOptData[0] = bridgeX.getFault();
          if((xHigh != 0) && (xLow != 0) && (respOptData[0] & FAULT) && (respOptData[0] != 0) && ((ignoreFlags & 0x01) == 0x00)) {
            break;
          }
          respOptData[1] = bridgeY.getFault();
          if((yHigh != 0) && (yLow != 0) && (respOptData[1] & FAULT) && (respOptData[1] != 0) && ((ignoreFlags & 0x02) == 0x00)) {
            break;
          }
          respOptData[2] = bridgeZ.getFault();
          if((zHigh != 0) && (zLow != 0) && (respOptData[2] & FAULT) && (respOptData[2] != 0) && ((ignoreFlags & 0x04) == 0x00)) {
            break;
          }

          // pet watchdog
          PowerControl_Watchdog_Heartbeat();

          // check new packets
          Communication_Check_New_Packet();
          if(abortExecution) {
            abortExecution = false;
            radio.standby();
            break;
          }
        }

        // stop everything
        bridgeX.stop();
        bridgeY.stop();
        bridgeZ.stop();

        // send response
        elapsed = millis() - start;
        memcpy(respOptData + 3, &elapsed, sizeof(uint32_t));
        Communication_Send_Response(RESP_MANUAL_ACS_RESULT, respOptData, 7);
      }
    } break;

    case CMD_GET_PICTURE_BURST: {
      // check FSK is active
      if((PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_FSK_ONLY_ENABLED) != 0x00) && (currentModem != MODEM_FSK)) {
        FOSSASAT_DEBUG_PRINTLN(F("FSK is required to transfer picture"));
        return;
      }

      if(Communication_Check_OptDataLen(4, optDataLen)) {
        // get the basic info
        FOSSASAT_DEBUG_PRINT(F("Reading slot: "));
        uint8_t slot = optData[0];
        FOSSASAT_DEBUG_PRINTLN(slot);
        FOSSASAT_DEBUG_PRINT(F("Starting at ID: "));
        uint16_t packetId = 0;
        memcpy(&packetId, optData + 1, sizeof(uint16_t));
        FOSSASAT_DEBUG_PRINTLN(packetId);
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

        // check scan only flag
        uint8_t scanOnly = optData[3];
        if(scanOnly == 0x01) {
          FOSSASAT_DEBUG_PRINTLN(F("Scan only"));

          // set start address to scan start
          imgAddress = PersistentStorage_Get_Image_ScanStart(slot);

          // set scan length
          imgLen = PersistentStorage_Get_Image_ScanEnd(slot) - imgAddress;
        }

        // send the data
        Communication_Transfer_Picture(imgAddress, imgLen, packetId, RESP_CAMERA_PICTURE);
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
      if(Communication_Check_OptDataLen(8, optDataLen)) {
        // get parameters
        uint32_t duration = 0;
        memcpy(&duration, optData, sizeof(uint32_t));
        FOSSASAT_DEBUG_PRINT(F("GPS logging duration: "));
        FOSSASAT_DEBUG_PRINTLN(duration);
        uint32_t offset = 0;
        memcpy(&offset, optData + sizeof(uint32_t), sizeof(uint32_t));
        FOSSASAT_DEBUG_PRINT(F("GPS logging offset: "));
        FOSSASAT_DEBUG_PRINTLN(offset);

        // check battery
        #ifdef ENABLE_TRANSMISSION_CONTROL
        if(PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_LOW_POWER_MODE) != LOW_POWER_NONE) {
          // battery check failed
          FOSSASAT_DEBUG_PRINTLN(F("Battery too low."));
          return;
        }
        #endif

        // command with long execution time - enable reception
        radio.startReceive();

        // wipe GPS log
        Navigation_GNSS_Wipe_Log();

        // power up GPS prior to waiting for offset
        digitalWrite(GPS_POWER_FET, POWER_FET_POLARITY_ON);

        // wait for offset to elapse
        FOSSASAT_DEBUG_PRINTLN(F("Waiting for offset to elapse"));
        PowerControl_Wait(offset * 1000, LOW_POWER_SLEEP);

        // setup logging variables
        Navigation_GNSS_Setup_Logging();

        // run for the requested duration
        while(rtc.getEpoch() - gpsLogState.start < duration) {
          // check new data
          Navigation_GNSS_SerialEvent();

          // check new packets
          Communication_Check_New_Packet();
          if(abortExecution) {
            abortExecution = false;
            Navigation_GNSS_Finish_Logging();
            return;
          }

          // check battery
          #ifdef ENABLE_TRANSMISSION_CONTROL
          if(PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_LOW_POWER_MODE) != LOW_POWER_NONE) {
            FOSSASAT_DEBUG_PRINTLN(F("Battery too low."));
            break;
          }
          #endif

          // sleep for one second
          PowerControl_Wait(1000, LOW_POWER_SLEEP);
        }

        // finish logging
        uint32_t logged = Navigation_GNSS_Finish_Logging();
        radio.standby();

        // send response
        const uint8_t respOptDataLen = 3*sizeof(uint32_t);
        uint8_t respOptData[respOptDataLen];
        memcpy(respOptData, &logged, sizeof(uint32_t));
        memcpy(respOptData + sizeof(uint32_t), &(gpsLogState.flashPos), sizeof(uint32_t));
        memcpy(respOptData + 2*sizeof(uint32_t), &(gpsLogState.lastFixAddr), sizeof(uint32_t));
        Communication_Send_Response(RESP_GPS_LOG_STATE, respOptData, respOptDataLen);
      }
    } break;

    case CMD_GET_GPS_LOG: {
      // check FSK is active
      if((PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_FSK_ONLY_ENABLED) != 0x00) && (currentModem != MODEM_FSK)) {
        FOSSASAT_DEBUG_PRINTLN(F("FSK is required to transfer GPS log"));
        return;
      }

      if(Communication_Check_OptDataLen(5, optDataLen)) {
        // get parameters
        uint8_t dir = optData[0];
        uint16_t offset = 0;
        memcpy(&offset, optData + sizeof(uint8_t), sizeof(uint16_t));
        uint32_t len = 0;
        memcpy(&len, optData + sizeof(uint8_t) + sizeof(uint16_t), sizeof(uint16_t));
        FOSSASAT_DEBUG_PRINT(F("GPS log download direction: "));
        FOSSASAT_DEBUG_PRINTLN(dir);
        FOSSASAT_DEBUG_PRINT(F("GPS log download offset: "));
        FOSSASAT_DEBUG_PRINTLN(offset);
        offset *= FLASH_NMEA_LOG_SLOT_SIZE;

        // read log length from flash
        FOSSASAT_DEBUG_PRINT(F("GPS log download length: "));
        uint32_t logged = PersistentStorage_SystemInfo_Get<uint32_t>(FLASH_NMEA_LOG_LENGTH);
        if((len == 0) || (len > logged)) {
          FOSSASAT_DEBUG_PRINT(logged);
          FOSSASAT_DEBUG_PRINTLN(F(" (full log)"));
          len = logged / FLASH_NMEA_LOG_SLOT_SIZE;
        } else {
          FOSSASAT_DEBUG_PRINTLN(len);
        }
        FOSSASAT_DEBUG_PRINT(F("Total GPS log length: "));
        FOSSASAT_DEBUG_PRINTLN(logged);
        if(logged == 0) {
          FOSSASAT_DEBUG_PRINT(F("No GPS data logged"));
          uint8_t respOptData[4] = {0, 0, 0, 0};
          Communication_Send_Response(RESP_GPS_LOG, respOptData, 4);
          return;
        }

        // get the starting address (all address are offset from FLASH_NMEA_LOG_START, to allow modulo calculations)
        uint32_t latestAddr = PersistentStorage_SystemInfo_Get<uint32_t>(FLASH_NMEA_LOG_LATEST_ENTRY) - FLASH_NMEA_LOG_START;
        uint32_t startAddr = 0;
        if(logged < (FLASH_NMEA_LOG_END - FLASH_NMEA_LOG_START)) {
          if(dir == 0) {
            // log is not full AND downlink from oldest - start at log space start
            startAddr = 0;
          } else {
            // log is not full AND downlink from newest - start at last logged address
            startAddr = latestAddr;
          }

        } else {
          // log is full, so it might have wrapped around
          if(dir == 0) {
            // log is full AND downlink from oldest - start at address next to the latest
            startAddr = (latestAddr + 1) % (FLASH_NMEA_LOG_END - FLASH_NMEA_LOG_START);
          } else {
            // log is full AND downlink from newest - start at last logged address
            startAddr = latestAddr;
          }

        }

        // move by the reqested offset
        uint32_t addr = 0;
        if(dir == 0) {
          // possible overflow is handled by modulo
          addr = (startAddr + offset) % (FLASH_NMEA_LOG_END - FLASH_NMEA_LOG_START);
        } else {
          // check underflow
          if(offset > startAddr) {
            addr = (FLASH_NMEA_LOG_END - FLASH_NMEA_LOG_START - 1) - (offset - startAddr);
          } else {
            addr = startAddr - offset;
          }
        }

        // translate address back to global format
        addr += FLASH_NMEA_LOG_START;
        FOSSASAT_DEBUG_PRINT(F("Starting from address: 0x"));
        FOSSASAT_DEBUG_PRINTLN(addr, HEX);
        FOSSASAT_DEBUG_PRINT(F("Number of packets: "));
        FOSSASAT_DEBUG_PRINTLN(len);

        // wait a bit before sending the first packet
        PowerControl_Wait(RESPONSE_DELAY, LOW_POWER_NONE);

        // read data from flash
        uint8_t respOptData[FLASH_NMEA_LOG_SLOT_SIZE];
        for(uint16_t packetNum = 0; packetNum < len; packetNum++) {
          // read data into buffer
          FOSSASAT_DEBUG_PRINTLN(addr, HEX);
          PersistentStorage_Read(addr, respOptData, FLASH_NMEA_LOG_SLOT_SIZE);

          // get the next address
          if(dir == 0) {
            addr = addr + FLASH_NMEA_LOG_SLOT_SIZE;
            if(addr >= FLASH_NMEA_LOG_END) {
              addr = FLASH_NMEA_LOG_START;
            }
          } else {
            addr = addr - FLASH_NMEA_LOG_SLOT_SIZE;
            if(addr < FLASH_NMEA_LOG_START) {
              addr = FLASH_NMEA_LOG_END - FLASH_NMEA_LOG_SLOT_SIZE;
            }
          }

          // get the number of bytes in log entry
          uint8_t respOptDataLen = 4 + strlen((char*)respOptData + 4);
          if(respOptDataLen > FLASH_NMEA_LOG_SLOT_SIZE) {
            respOptDataLen = FLASH_NMEA_LOG_SLOT_SIZE;
          }

          // send response
          Communication_Send_Response(RESP_GPS_LOG, respOptData, respOptDataLen);

          // check battery
          PowerControl_Watchdog_Heartbeat();
          #ifdef ENABLE_TRANSMISSION_CONTROL
          if(PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_LOW_POWER_MODE) != LOW_POWER_NONE) {
            FOSSASAT_DEBUG_PRINTLN(F("Battery too low."));
            return;
          }
          #endif
        }
      }
    } break;

    case CMD_ROUTE:
      // just transmit the optional data
      Communication_Transmit(optData, optDataLen);
      break;

    case CMD_SET_FLASH_CONTENTS: {
      if (optDataLen >= sizeof(uint32_t) + 1) {
        uint32_t address = 0;
        memcpy(&address, optData, sizeof(uint32_t));
        uint8_t dataLen = optDataLen - sizeof(uint32_t);
        uint8_t flashBuff[FLASH_EXT_PAGE_SIZE];
        PersistentStorage_Read(address / FLASH_EXT_PAGE_SIZE, flashBuff, FLASH_EXT_PAGE_SIZE);
        memcpy(flashBuff + (address % FLASH_EXT_PAGE_SIZE), optData + sizeof(uint32_t), dataLen);
        PersistentStorage_Write(address / FLASH_EXT_PAGE_SIZE, flashBuff, FLASH_EXT_PAGE_SIZE, false);
      }
    } break;

    case CMD_SET_TLE: {
      if(Communication_Check_OptDataLen(138, optDataLen)) {
        char line[70];
        uint8_t tleBuff[FLASH_EXT_PAGE_SIZE];

        // get the first TLE line
        memcpy(line, optData, 69);
        line[69] = '\0';

        // parse first TLE line
        uint8_t b = Navigation_Get_EpochYear(line);
        memcpy(tleBuff + FLASH_TLE_EPOCH_YEAR, &b, sizeof(uint8_t));
        double d = Navigation_Get_EpochDay(line);
        memcpy(tleBuff + FLASH_TLE_EPOCH_DAY, &d, sizeof(double));
        d = Navigation_Get_BallisticCoeff(line);
        memcpy(tleBuff + FLASH_TLE_BALLISTIC_COEFF, &d, sizeof(double));
        d = Navigation_Get_MeanMotion2nd(line);
        memcpy(tleBuff + FLASH_TLE_MEAN_MOTION_2ND, &d, sizeof(double));
        d = Navigation_Get_DragTerm(line);
        memcpy(tleBuff + FLASH_TLE_DRAG_TERM, &d, sizeof(double));

        // get the second TLE line
        memcpy(line, optData + 69, 69);
        line[69] = '\0';

        // parse second TLE line
        d = Navigation_Get_Inclination(line);
        memcpy(tleBuff + FLASH_TLE_INCLINATION, &d, sizeof(double));
        d = Navigation_Get_RightAscension(line);
        memcpy(tleBuff + FLASH_TLE_RIGHT_ASCENTION, &d, sizeof(double));
        d = Navigation_Get_Eccentricity(line);
        memcpy(tleBuff + FLASH_TLE_ECCENTRICITY, &d, sizeof(double));
        d = Navigation_Get_PerigeeArgument(line);
        memcpy(tleBuff + FLASH_TLE_PERIGEE_ARGUMENT, &d, sizeof(double));
        d = Navigation_Get_MeanAnomaly(line);
        memcpy(tleBuff + FLASH_TLE_MEAN_ANOMALY, &d, sizeof(double));
        d = Navigation_Get_MeanMotion(line);
        memcpy(tleBuff + FLASH_TLE_MEAN_MOTION, &d, sizeof(double));
        uint32_t ul = Navigation_Get_RevolutionNumber(line);
        memcpy(tleBuff + FLASH_TLE_REVOLUTION_NUMBER, &ul, sizeof(uint32_t));

        // update system info page
        PersistentStorage_Set_Buffer(FLASH_TLE_EPOCH_DAY, tleBuff + FLASH_TLE_EPOCH_DAY, FLASH_TLE_EPOCH_YEAR - FLASH_TLE_EPOCH_DAY + sizeof(uint8_t));
      }
    } break;

    case CMD_GET_GPS_LOG_STATE: {
      // fetch GPS log state information
      uint32_t logged = PersistentStorage_SystemInfo_Get<uint32_t>(FLASH_NMEA_LOG_LENGTH);
      uint32_t flashPos = PersistentStorage_SystemInfo_Get<uint32_t>(FLASH_NMEA_LOG_LATEST_ENTRY);
      uint32_t lastFixAddr = PersistentStorage_SystemInfo_Get<uint32_t>(FLASH_NMEA_LOG_LATEST_FIX);

      // send the response
      const uint8_t respOptDataLen = 3*sizeof(uint32_t);
      uint8_t respOptData[respOptDataLen];
      memcpy(respOptData, &logged, sizeof(uint32_t));
      memcpy(respOptData + sizeof(uint32_t), &flashPos, sizeof(uint32_t));
      memcpy(respOptData + 2*sizeof(uint32_t), &lastFixAddr, sizeof(uint32_t));
      Communication_Send_Response(RESP_GPS_LOG_STATE, respOptData, respOptDataLen);
    } break;

    case CMD_RUN_GPS_COMMAND: {
      // create response buffer
      uint8_t respOptData[MAX_OPT_DATA_LENGTH];

      // run the command
      uint16_t respOptDataLen = Navigation_GNSS_Run_Cmd(optData, optDataLen, respOptData);

      // send the response
      if((respOptDataLen == 0) || (respOptDataLen == 0xFFFF)) {
        Communication_Send_Response(RESP_GPS_COMMAND_RESPONSE);
      } else {
        Communication_Send_Response(RESP_GPS_COMMAND_RESPONSE, respOptData, respOptDataLen);
      }

    } break;

    case CMD_SET_SLEEP_INTERVALS: {
      // parse the number of sleep intervals
      uint8_t intervalSize = sizeof(int16_t) + sizeof(uint16_t);
      uint8_t numIntervals = optDataLen / intervalSize;

      // check optDataLen is multiple of intervalSize, and the number of intervals is with limits
      if(!((optDataLen % intervalSize != 0) || (numIntervals == 0) || (numIntervals > 8))) {
        FOSSASAT_DEBUG_PRINT(F("numIntervals = "));
        FOSSASAT_DEBUG_PRINTLN(numIntervals);
        systemInfoBuffer[FLASH_NUM_SLEEP_INTERVALS] = numIntervals;

        // parse voltage thresholds and interval lengths
        for(uint8_t i = 0; i < numIntervals; i++) {
          FOSSASAT_DEBUG_PRINT(i);
          FOSSASAT_DEBUG_PRINT('\t');

          int16_t voltage = 0;
          memcpy(&voltage, optData + i*intervalSize, sizeof(int16_t));
          FOSSASAT_DEBUG_PRINT(voltage);
          FOSSASAT_DEBUG_PRINT('\t');
          memcpy(systemInfoBuffer + FLASH_SLEEP_INTERVALS + i*intervalSize, &voltage, sizeof(int16_t));

          uint16_t intervalLen = 0;
          memcpy(&intervalLen, optData + sizeof(int16_t) + i*intervalSize, sizeof(uint16_t));
          FOSSASAT_DEBUG_PRINTLN(intervalLen);
          memcpy(systemInfoBuffer + FLASH_SLEEP_INTERVALS + sizeof(int16_t) + i*intervalSize, &intervalLen, sizeof(uint16_t));

        }
      }

    } break;

    case CMD_ABORT: {
      FOSSASAT_DEBUG_PRINTLN(F("Aborting current operation"));
      abortExecution = true;
    } break;

    case CMD_MANEUVER: {
      if(Communication_Check_OptDataLen(5, optDataLen)) {
        // extract parameters
        uint32_t len = 0;
        memcpy(&len, optData + 1, sizeof(len));
        FOSSASAT_DEBUG_PRINT(F("len = "));
        FOSSASAT_DEBUG_PRINTLN(len);

        uint8_t controlFlags = optData[0];
        FOSSASAT_DEBUG_PRINT(F("controlFlags = "));
        FOSSASAT_DEBUG_PRINTLN(controlFlags, HEX);

        // read orbital period and inclination from TLE
        double orbitalInclination = PersistentStorage_SystemInfo_Get<double>(FLASH_TLE_INCLINATION) * (M_PI/180.0);
        double meanOrbitalMotion = (2.0 * M_PI * PersistentStorage_SystemInfo_Get<double>(FLASH_TLE_MEAN_MOTION)) / (24.0 * 3600.0);

        // initialize ADCS
        ADCS_ActiveControl_Init(controlFlags, len, orbitalInclination, meanOrbitalMotion);
      }
    } break;

    case CMD_DETUMBLE: {
      if(Communication_Check_OptDataLen(5, optDataLen)) {
        // extract parameters
        uint32_t len = 0;
        memcpy(&len, optData + 1, sizeof(len));
        FOSSASAT_DEBUG_PRINT(F("len = "));
        FOSSASAT_DEBUG_PRINTLN(len);

        uint8_t controlFlags = optData[0];
        FOSSASAT_DEBUG_PRINT(F("controlFlags = "));
        FOSSASAT_DEBUG_PRINTLN(controlFlags, HEX);

        // read orbital period and inclination from TLE
        double orbitalInclination = PersistentStorage_SystemInfo_Get<double>(FLASH_TLE_INCLINATION) * (M_PI/180.0);
        double meanOrbitalMotion = (2.0 * M_PI * PersistentStorage_SystemInfo_Get<double>(FLASH_TLE_MEAN_MOTION)) / (24.0 * 3600.0);

        // initialize ADCS
        ADCS_Detumble_Init(controlFlags, len, orbitalInclination, meanOrbitalMotion);
      }
    } break;

    case CMD_SET_ADCS_PARAMETERS: {
      if(Communication_Check_OptDataLen(34, optDataLen)) {
        // read the current ADCS parameters
        uint8_t adcsSector[FLASH_SECTOR_SIZE];
        PersistentStorage_Read(FLASH_ADCS_PARAMETERS, adcsSector, FLASH_SECTOR_SIZE);
        uint8_t* optDataPtr = optData;

        // set the parameters that require conversion
        Communication_Set_ADCS_Param(&optDataPtr, adcsSector, FLASH_ADCS_PULSE_MAX_INTENSITY);
        Communication_Set_ADCS_Param(&optDataPtr, adcsSector, FLASH_ADCS_PULSE_MAX_LENGTH);
        Communication_Set_ADCS_Param(&optDataPtr, adcsSector, FLASH_ADCS_DETUMB_OMEGA_TOLERANCE);
        Communication_Set_ADCS_Param(&optDataPtr, adcsSector, FLASH_ADCS_PULSE_AMPLITUDE);
        Communication_Set_ADCS_Param(&optDataPtr, adcsSector, FLASH_ADCS_CALCULATION_TOLERANCE);
        Communication_Set_ADCS_Param(&optDataPtr, adcsSector, FLASH_ADCS_MIN_INERTIAL_MOMENT);
        Communication_Set_ADCS_Param(&optDataPtr, adcsSector, FLASH_ADCS_ACTIVE_EULER_TOLERANCE);
        Communication_Set_ADCS_Param(&optDataPtr, adcsSector, FLASH_ADCS_ACTIVE_OMEGA_TOLERANCE);
        Communication_Set_ADCS_Param(&optDataPtr, adcsSector, FLASH_ADCS_ECLIPSE_THRESHOLD);
        Communication_Set_ADCS_Param(&optDataPtr, adcsSector, FLASH_ADCS_ROTATION_WEIGHT_RATIO);
        Communication_Set_ADCS_Param(&optDataPtr, adcsSector, FLASH_ADCS_ROTATION_TRIGGER);
        Communication_Set_ADCS_Param(&optDataPtr, adcsSector, FLASH_ADCS_DISTURBANCE_COVARIANCE);
        Communication_Set_ADCS_Param(&optDataPtr, adcsSector, FLASH_ADCS_NOISE_COVARIANCE);

        // set the rest
        memcpy(adcsSector + (FLASH_ADCS_TIME_STEP - FLASH_ADCS_PARAMETERS), optDataPtr, sizeof(uint32_t));
        optDataPtr += sizeof(uint32_t);
        memcpy(adcsSector + (FLASH_ADCS_BRIDGE_TIMER_UPDATE_PERIOD - FLASH_ADCS_PARAMETERS), optDataPtr, sizeof(uint32_t));
        optDataPtr += sizeof(uint32_t);
        memcpy(adcsSector + (FLASH_ADCS_BRIDGE_OUTPUT_HIGH - FLASH_ADCS_PARAMETERS), optDataPtr, sizeof(int8_t));
        optDataPtr += sizeof(int8_t);
        memcpy(adcsSector + (FLASH_ADCS_BRIDGE_OUTPUT_LOW - FLASH_ADCS_PARAMETERS), optDataPtr, sizeof(int8_t));
        optDataPtr += sizeof(int8_t);
        memcpy(adcsSector + (FLASH_ADCS_NUM_CONTROLLERS - FLASH_ADCS_PARAMETERS), optDataPtr, sizeof(uint8_t));
        optDataPtr += sizeof(uint8_t);

        // write all at once
        PersistentStorage_Write(FLASH_ADCS_PARAMETERS, adcsSector, FLASH_SECTOR_SIZE);
      }
    } break;

    case CMD_ERASE_FLASH: {
      if(Communication_Check_OptDataLen(4, optDataLen)) {
        uint32_t addr = 0;
        memcpy(&addr, optData, sizeof(uint32_t));
        FOSSASAT_DEBUG_PRINT(F("Erasing flash sector at address 0x"));
        FOSSASAT_DEBUG_PRINTLN(addr, HEX);
        PersistentStorage_SectorErase(addr);
      }

    } break;

    case CMD_SET_ADCS_CONTROLLER: {
      if(Communication_Check_OptDataLen(77, optDataLen)) {
        uint8_t id = optData[0];
        FOSSASAT_DEBUG_PRINT(F("ADCS controller ID: "));
        FOSSASAT_DEBUG_PRINTLN(id);

        FOSSASAT_DEBUG_PRINTLN(F("Controller: "));
        float val = 0;
        float controller[ADCS_NUM_AXES][2*ADCS_NUM_AXES];
        for(uint8_t i = 0; i < ADCS_NUM_AXES; i++) {
          for(uint8_t j = 0; j < 2*ADCS_NUM_AXES; j++) {
            memcpy(&val, optData + 1 + (2*ADCS_NUM_AXES*i + j) * sizeof(float), sizeof(float));
            FOSSASAT_DEBUG_PRINT(val, 4);
            FOSSASAT_DEBUG_PRINT('\t');
            controller[i][j] = val;
          }
          FOSSASAT_DEBUG_PRINTLN();
        }
        PersistentStorage_Set_ADCS_Controller(id, controller);

      }

    } break;

    case CMD_SET_ADCS_EPHEMERIDES: {
      if(optDataLen >= 27) {
        uint16_t chunkId = 0;
        memcpy(&chunkId, optData, sizeof(chunkId));
        FOSSASAT_DEBUG_PRINT(F("ADCS ephemerides chunk ID: "));
        FOSSASAT_DEBUG_PRINTLN(chunkId);

        uint8_t rowCount = (optDataLen - 2) / FLASH_ADCS_EPHEMERIDES_SLOT_SIZE;
        FOSSASAT_DEBUG_PRINT(F("Number of ephemerides rows: "));
        FOSSASAT_DEBUG_PRINTLN(rowCount);

        uint8_t controllerId = optData[optDataLen - 1];
        FOSSASAT_DEBUG_PRINT(F("Controller type: "));
        FOSSASAT_DEBUG_PRINTLN(controllerId);

        for(uint8_t row = 0; row < rowCount; row++) {
          FOSSASAT_DEBUG_PRINT(F("Ephe row #: "));
          FOSSASAT_DEBUG_PRINTLN(row);

          float val = 0;
          uint8_t i = 0;
          float ephemerides[2*ADCS_NUM_AXES];

          FOSSASAT_DEBUG_PRINTLN(F("Mag. ephemeris: "));
          for(; i < ADCS_NUM_AXES; i++) {
              memcpy(&val, optData + 2 + row*FLASH_ADCS_EPHEMERIDES_SLOT_SIZE + i*(sizeof(float)), sizeof(float));
              FOSSASAT_DEBUG_PRINT(val, 4);
              FOSSASAT_DEBUG_PRINT('\t');
              ephemerides[i] = val;
          }
          FOSSASAT_DEBUG_PRINTLN();

          FOSSASAT_DEBUG_PRINTLN(F("Solar ephemeris: "));
          for(; i < 2*ADCS_NUM_AXES; i++) {
              memcpy(&val, optData + 2 + row*FLASH_ADCS_EPHEMERIDES_SLOT_SIZE + i*(sizeof(float)), sizeof(float));
              FOSSASAT_DEBUG_PRINT(val, 4);
              FOSSASAT_DEBUG_PRINT('\t');
              ephemerides[i] = val;
          }
          FOSSASAT_DEBUG_PRINTLN();

          PersistentStorage_Set_ADCS_Ephemerides(chunkId*5 + row, ephemerides, controllerId);
        }
      }

    } break;

    case CMD_SET_IMU_OFFSET: {
      if(Communication_Check_OptDataLen(12, optDataLen)) {
        float x = 0;
        float y = 0;
        float z = 0;

        memcpy(&x, optData, sizeof(float));
        memcpy(&y, optData + sizeof(float), sizeof(float));
        memcpy(&z, optData + 2*sizeof(float), sizeof(float));

        FOSSASAT_DEBUG_PRINT(F("Gyroscope:\t"));
        FOSSASAT_DEBUG_PRINTLN(x);
        FOSSASAT_DEBUG_PRINT('\t');
        FOSSASAT_DEBUG_PRINTLN(y);
        FOSSASAT_DEBUG_PRINT('\t');
        FOSSASAT_DEBUG_PRINTLN(y);

        PersistentStorage_SystemInfo_Set<float>(FLASH_IMU_OFFSET_GYRO_X, x);
        PersistentStorage_SystemInfo_Set<float>(FLASH_IMU_OFFSET_GYRO_Y, y);
        PersistentStorage_SystemInfo_Set<float>(FLASH_IMU_OFFSET_GYRO_Z, z);

        memcpy(&x, optData + 3*sizeof(float), sizeof(float));
        memcpy(&y, optData + 4*sizeof(float), sizeof(float));
        memcpy(&z, optData + 5*sizeof(float), sizeof(float));

        FOSSASAT_DEBUG_PRINT(F("Accelerometer:\t"));
        FOSSASAT_DEBUG_PRINTLN(x);
        FOSSASAT_DEBUG_PRINT('\t');
        FOSSASAT_DEBUG_PRINTLN(y);
        FOSSASAT_DEBUG_PRINT('\t');
        FOSSASAT_DEBUG_PRINTLN(y);

        PersistentStorage_SystemInfo_Set<float>(FLASH_IMU_OFFSET_ACCEL_X, x);
        PersistentStorage_SystemInfo_Set<float>(FLASH_IMU_OFFSET_ACCEL_Y, y);
        PersistentStorage_SystemInfo_Set<float>(FLASH_IMU_OFFSET_ACCEL_Z, z);

        memcpy(&x, optData + 6*sizeof(float), sizeof(float));
        memcpy(&y, optData + 7*sizeof(float), sizeof(float));
        memcpy(&z, optData + 8*sizeof(float), sizeof(float));

        FOSSASAT_DEBUG_PRINT(F("Magnetometer:\t"));
        FOSSASAT_DEBUG_PRINTLN(x);
        FOSSASAT_DEBUG_PRINT('\t');
        FOSSASAT_DEBUG_PRINTLN(y);
        FOSSASAT_DEBUG_PRINT('\t');
        FOSSASAT_DEBUG_PRINTLN(y);

        PersistentStorage_SystemInfo_Set<float>(FLASH_IMU_OFFSET_MAG_X, x);
        PersistentStorage_SystemInfo_Set<float>(FLASH_IMU_OFFSET_MAG_Y, y);
        PersistentStorage_SystemInfo_Set<float>(FLASH_IMU_OFFSET_MAG_Z, z);
      }

    } break;

    case CMD_SET_IMU_CALIBRATION: {
      if(Communication_Check_OptDataLen(45, optDataLen)) {
        // TODO implement
      }
    } break;

    default:
      FOSSASAT_DEBUG_PRINT(F("Unknown function ID!"));
      return;
  }
}

int16_t Communication_Send_Response(uint8_t respId, uint8_t* optData, size_t optDataLen, bool overrideModem) {
  // get callsign from EEPROM
  uint8_t callsignLen = PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_CALLSIGN_LEN);
  char callsign[MAX_STRING_LENGTH];
  PersistentStorage_Get_Callsign(callsign, callsignLen);

  // build response frame
  uint8_t len = FCP_Get_Frame_Length(callsign, optDataLen);
  uint8_t frame[MAX_RADIO_BUFFER_LENGTH];
  int16_t state = FCP_Encode(frame, callsign, respId, optDataLen, optData);
  FOSSASAT_DEBUG_PRINT(F("Encoding state: "));
  FOSSASAT_DEBUG_PRINTLN(state);
  FOSSASAT_DEBUG_DELAY(10);

  // delay before responding
  if((respId == RESP_GPS_LOG) ||
     (respId == RESP_CAMERA_PICTURE) ||
     (respId == RESP_RECORDED_IMU)) {

    // use short length in case of "burst" transfers
    PowerControl_Wait(RESPONSE_DELAY_SHORT, LOW_POWER_NONE);

  } else {
    // use standard length in all other cases
    PowerControl_Wait(RESPONSE_DELAY, LOW_POWER_NONE);
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
  uint8_t txEnabled = PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_TRANSMISSIONS_ENABLED);
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
