#include "Navigation.h"

static uint32_t Navigation_Parse_Int(const char* tleLine, uint8_t pos, uint8_t len) {
  char str[32];
  memcpy(str, tleLine + pos, len);
  str[len] = '\0';
  return(strtol(str, NULL, 10));
}

static double Navigation_Parse_Double(const char* tleLine, uint8_t pos, uint8_t len) {
  // copy TLE number
  char str[32];
  memcpy(str, tleLine + pos, len);
  str[len] = '\0';

  // check sign
  double sign = 1;
  if(str[0] == '-') {
    str[0] = '0';
    sign = -1;
  }

  // parse
  return(sign * strtod(str, NULL));
}

static double Navigation_Parse_Decimal(const char* tleLine, uint8_t pos, uint8_t len) {
  // copy TLE number
  char str[32];
  memcpy(str, tleLine + pos, len);
  str[len] = '\0';
  uint8_t mantLen = len;
  char* strPtr = str;

  // check sign
  double sign = 1;
  if(str[0] == '-') {
    sign = -1;
    mantLen -= 1;
    strPtr++;
    str[0] = ' ';
  }

  // get exponent
  double exponent = 0;
  if((str[len - 2] == '-') || (str[len - 2] == '+')) {
    exponent = strtod(str + len - 2, NULL);
    mantLen -= 2;
    str[len - 2] = ' ';
  }

  // get base
  double mant = strtod(strPtr, NULL);
  mant = mant / pow(10, mantLen - 1);

  // get the result
  return(sign * mant * pow(10, exponent));
}

uint8_t Navigation_Get_EpochYear(const char* tleLine) {
  return((uint8_t)Navigation_Parse_Int(tleLine, 18, 2));
}

double Navigation_Get_EpochDay(const char* tleLine) {
  return(Navigation_Parse_Double(tleLine, 20, 12));
}

double Navigation_Get_BallisticCoeff(const char* tleLine) {
  return(Navigation_Parse_Double(tleLine, 33, 10));
}

double Navigation_Get_MeanMotion2nd(const char* tleLine) {
  return(Navigation_Parse_Decimal(tleLine, 44, 8));
}

double Navigation_Get_DragTerm(const char* tleLine) {
  return(Navigation_Parse_Decimal(tleLine, 53, 8));
}

double Navigation_Get_Inclination(const char* tleLine) {
  return(Navigation_Parse_Double(tleLine, 8, 8));
}

double Navigation_Get_RightAscension(const char* tleLine) {
  return(Navigation_Parse_Double(tleLine, 17, 8));
}

double Navigation_Get_Eccentricity(const char* tleLine) {
  return(Navigation_Parse_Decimal(tleLine, 26, 7));
}

double Navigation_Get_PerigeeArgument(const char* tleLine) {
  return(Navigation_Parse_Double(tleLine, 34, 8));
}

double Navigation_Get_MeanAnomaly(const char* tleLine) {
  return(Navigation_Parse_Double(tleLine, 43, 8));
}

double Navigation_Get_MeanMotion(const char* tleLine) {
  return(Navigation_Parse_Double(tleLine, 52, 11));
}

double Navigation_Get_RevolutionNumber(const char* tleLine) {
  return(Navigation_Parse_Int(tleLine, 63, 5));
}

uint16_t Navigation_GNSS_Run_Cmd(uint8_t* payload, uint16_t payloadLen, uint8_t* resp, uint32_t timeout) {
  // initialize UART interface
  GpsSerial.begin(9600);

  // power up GPS
  digitalWrite(GPS_POWER_FET, POWER_FET_POLARITY_ON);

  // wait for GPS to boot up
  PowerControl_Wait(50);

  // dump serial buffer
  while(GpsSerial.available()) {
    GpsSerial.read();
  }

  // send the command
  if(!Navigation_GNSS_Send_Cmd(payload, payloadLen)) {
    // command returned NACK or timed out
    digitalWrite(GPS_POWER_FET, POWER_FET_POLARITY_OFF);
    GpsSerial.end();
    return(0);
  }

  // read response
  uint16_t responseLen = Navigation_GNSS_Get_Resp(resp, timeout);

  // turn GPS off
  digitalWrite(GPS_POWER_FET, POWER_FET_POLARITY_OFF);

  // stop UART interface (to prevent it from waking up the MCU)
  GpsSerial.end();

  // return the number of received bytes
  return(responseLen);
}

bool Navigation_GNSS_Send_Cmd(uint8_t* payload, uint16_t payloadLen) {
  // create buffer
  uint8_t buff[MAX_OPT_DATA_LENGTH];
  uint8_t* buffPtr = buff;
  uint16_t buffLen = 2 + 2 + payloadLen + 1 + 2;

  // set start of sequence characters
  *buffPtr++ = SKYTRAQ_START_OF_SEQ_1;
  *buffPtr++ = SKYTRAQ_START_OF_SEQ_2;

  // set payload length
  *buffPtr++ = (uint8_t)((payloadLen >> 8) & 0xFF);
  *buffPtr++ = (uint8_t)(payloadLen & 0xFF);

  // set payload
  memcpy(buffPtr, payload, payloadLen);
  buffPtr += payloadLen;

  // set checksum
  uint8_t checkSum = 0;
  for(uint16_t i = 0; i < payloadLen; i++) {
    checkSum ^= payload[i];
  }
  *buffPtr++ = checkSum;

  // set end of sequence
  *buffPtr++ = SKYTRAQ_END_OF_SEQ_1;
  *buffPtr++ = SKYTRAQ_END_OF_SEQ_2;

  // send the command
  FOSSASAT_DEBUG_PRINTLN(F("GPS command:"));
  FOSSASAT_DEBUG_PRINT_BUFF(buff, buffLen);
  for(uint16_t i = 0; i < buffLen; i++) {
    GpsSerial.write(buff[i]);
  }

  // pet the watchdog
  PowerControl_Watchdog_Heartbeat();

  // get ACK/NACK
  uint8_t ackBuff[2];
  uint16_t respLen = Navigation_GNSS_Get_Resp(ackBuff);
  PowerControl_Watchdog_Heartbeat();
  if(respLen == 0xFFFF) {
    return(false);
  }

  // return ACK/NACK
  return(ackBuff[0] == SKYTRAQ_ACK);
}

uint16_t Navigation_GNSS_Get_Resp(uint8_t* resp, uint32_t timeout) {
  // wait for sequence start
  uint8_t prev = '\0';
  uint32_t start = millis();
  bool gotStart = false;
  while(millis() - start < timeout) {
    if(GpsSerial.available()) {
      uint8_t b = GpsSerial.read();

      // check start of sequence bytes
      if((b == SKYTRAQ_START_OF_SEQ_2) && (prev == SKYTRAQ_START_OF_SEQ_1)) {
        gotStart = true;
        break;
      } else {
        prev = b;
      }
    }
  }

  // check timeout
  if(!gotStart) {
    FOSSASAT_DEBUG_PRINTLN(F("GPS timed out waiting for sequence start!"));
    return(0xFFFF);
  }

  // sequence start was received, wait for payload length
  start = millis();
  while(GpsSerial.available() < 2) {
    if(millis() - start > timeout) {
      FOSSASAT_DEBUG_PRINTLN(F("GPS timed out waiting for payload length!"));
      return(0xFFFF);
    }
  }

  // read payload length
  uint16_t msb = GpsSerial.read();
  uint16_t lsb = GpsSerial.read();
  uint16_t payloadLen = ((msb & 0xFF) << 8) | (lsb & 0xFF);
  FOSSASAT_DEBUG_PRINT(F("Expected payload length: 0x"));
  FOSSASAT_DEBUG_PRINTLN(payloadLen, HEX);

  // wait for payload, checksum and sequence end bytes
  start = millis();
  while(GpsSerial.available() < (payloadLen + 1 + 2)) {
    if(millis() - start > timeout) {
      FOSSASAT_DEBUG_PRINTLN(F("GPS timed out waiting for payload!"));
      return(0xFFFF);
    }
  }

  // read payload
  uint16_t receivedLen = 0;
  for(; receivedLen < payloadLen; receivedLen++) {
    resp[receivedLen] = GpsSerial.read();
  }

  FOSSASAT_DEBUG_PRINTLN(F("GPS response:"));
  FOSSASAT_DEBUG_PRINT_BUFF(resp, receivedLen);

  // read checksum and frame end bytes (not processed, just removed from the Serial buffer)
  for(uint8_t i = 0; i < 3; i++) {
    GpsSerial.read();
  }

  // return the number of received bytes
  return(receivedLen + 1);
}

void Navigation_GNSS_Wipe_Log() {
  // wipe NMEA log
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

void Navigation_GNSS_Setup_Logging() {
   // ensure GPS is on
  digitalWrite(GPS_POWER_FET, POWER_FET_POLARITY_ON);

  // initialize UART interface
  GpsSerial.begin(9600);
  FOSSASAT_DEBUG_PRINTLN(F("GPS logging start"));

  // log entries are saved in 128-byte chunks (to fit two chunks in one flash page)
  memset(gpsLogState.buff, 0x00, FLASH_NMEA_LOG_SLOT_SIZE);
  gpsLogState.buffPos = sizeof(uint32_t);

  // log starts from the first address
  gpsLogState.flashPos = FLASH_NMEA_LOG_START;

  // address of the last fix
  gpsLogState.lastFixAddr = 0;

  // whether we need to overwrite the current page
  gpsLogState.overwrite = false;

  // run for the requested duration
  gpsLogState.start = rtc.getEpoch();

  // set science mode flag
  scienceModeActive = true;
}

void Navigation_GNSS_SerialEvent() {
  while(GpsSerial.available() > 0) {
    char c = GpsSerial.read();

    // check if we got line ending or the buffer is full
    if((c == '\n') || (gpsLogState.buffPos == FLASH_NMEA_LOG_SLOT_SIZE)) {
      // add timestamp
      uint32_t stamp = millis() - gpsLogState.start;
      memcpy(gpsLogState.buff, &stamp, sizeof(uint32_t));
      FOSSASAT_DEBUG_PRINTLN(stamp, HEX);
      FOSSASAT_DEBUG_PRINTLN(gpsLogState.buffPos);

      // add null terminator instead of CR
      gpsLogState.buff[gpsLogState.buffPos - 1] = '\0';
      FOSSASAT_DEBUG_PRINTLN((char*)gpsLogState.buff + 4);
      FOSSASAT_DEBUG_PRINTLN(gpsLogState.flashPos, HEX);
      FOSSASAT_DEBUG_DELAY(10);

      // check if we got GPS fix
      if(memcmp(gpsLogState.buff + 7, "GSA", 3) == 0) {
        // GSA message, check fix value
        if((gpsLogState.buff[13] == '2') || (gpsLogState.buff[13] == '3')) {
          // got fix, save last fix address
          gpsLogState.lastFixAddr = gpsLogState.flashPos;
          FOSSASAT_DEBUG_PRINTLN(F("Got fix"));
        }
      }

      // check if we are overwriting old data
      if(gpsLogState.overwrite && (gpsLogState.flashPos % FLASH_SECTOR_SIZE == 0)) {
        // reading sector to RAM, erasing and then writing back to flash would be too slow
        PersistentStorage_SectorErase(gpsLogState.flashPos);
        PowerControl_Watchdog_Heartbeat();
        FOSSASAT_DEBUG_PRINTLN(F("Erased sector "));
        FOSSASAT_DEBUG_PRINTLN(gpsLogState.flashPos, HEX);
      }

      // write the buffer
      PersistentStorage_Write(gpsLogState.flashPos, gpsLogState.buff, gpsLogState.buffPos, false);
      FOSSASAT_DEBUG_PRINTLN(F("-----"));
      FOSSASAT_DEBUG_DELAY(10);

      // update address of the latest log entry
      PersistentStorage_SystemInfo_Set<uint32_t>(FLASH_NMEA_LOG_LATEST_ENTRY, gpsLogState.flashPos);

      // reset buffer position
      gpsLogState.buffPos = sizeof(uint32_t);

      // check there's still space left
      gpsLogState.flashPos += FLASH_NMEA_LOG_SLOT_SIZE;
      if(gpsLogState.flashPos >= FLASH_NMEA_LOG_END) {
        // reached end of flash reserved for GPS log, set overwrite flag and start over
        gpsLogState.flashPos = FLASH_NMEA_LOG_START;
        gpsLogState.overwrite = true;
      }

      PowerControl_Watchdog_Heartbeat();

    } else {
      // add to buffer
      gpsLogState.buff[gpsLogState.buffPos] = c;
      gpsLogState.buffPos++;
    }
  }
}

uint32_t Navigation_GNSS_Finish_Logging() {
  // update last fix addres
  PersistentStorage_SystemInfo_Set<uint32_t>(FLASH_NMEA_LOG_LATEST_FIX, gpsLogState.lastFixAddr);

  // turn GPS off
  digitalWrite(GPS_POWER_FET, POWER_FET_POLARITY_OFF);

  // stop UART interface (to prevent it from waking up the MCU)
  GpsSerial.end();

  // clear science mode flag
  scienceModeActive = false;

  // save the number of logged bytes and send it
  uint32_t logged = gpsLogState.flashPos - FLASH_NMEA_LOG_START;
  if(gpsLogState.overwrite) {
    // log is full when the overwrite flag is set
    logged = FLASH_NMEA_LOG_END - FLASH_NMEA_LOG_START;
  }
  FOSSASAT_DEBUG_PRINT(F("Logged total of (bytes): "));
  FOSSASAT_DEBUG_PRINTLN(logged);
  PersistentStorage_SystemInfo_Set<uint32_t>(FLASH_NMEA_LOG_LENGTH, logged);
  return(logged);
}
