#include "PersistentStorage.h"

void PersistentStorage_Increment_Counter(uint16_t addr) {
  uint16_t counter = PersistentStorage_Get<uint16_t>(addr);
  counter++;
  PersistentStorage_Set(addr, counter);
}

void PersistentStorage_Increment_Frame_Counter(bool valid) {
  uint16_t addr = FLASH_LORA_VALID_COUNTER_ADDR;
  if(currentModem == MODEM_LORA) {
    if(!valid) {
      addr += 2;
    }
  } else {
    if(valid) {
      addr += 4;
    } else {
      addr += 6;
    }
  }

  PersistentStorage_Increment_Counter(addr);
}

void PersistentStorage_Get_Callsign(char* buff, uint8_t len) {
  PersistentStorage_Read(FLASH_CALLSIGN_ADDR, (uint8_t*)buff, len);
}

void PersistentStorage_Set_Callsign(char* newCallsign) {
  // get length of the new callsign
  uint8_t newCallsignLen = (uint8_t)strlen(newCallsign);

  // check new callsign length
  if (newCallsignLen > MAX_STRING_LENGTH) {
    FOSSASAT_DEBUG_PRINTLN(F("New callsign too long!"));
    return;
  }

  // read the current system info page
  uint8_t sysInfoPage[FLASH_SYSTEM_INFO_LEN];
  PersistentStorage_Read(FLASH_SYSTEM_INFO_START, sysInfoPage, FLASH_SYSTEM_INFO_LEN);

  // update callsign entries
  sysInfoPage[FLASH_CALLSIGN_LEN_ADDR] = newCallsignLen;
  memcpy(sysInfoPage + FLASH_CALLSIGN_ADDR, newCallsign, newCallsignLen);
  PersistentStorage_Write(FLASH_SYSTEM_INFO_START, sysInfoPage, FLASH_SYSTEM_INFO_LEN);
}

void PersistentStorage_Reset_System_Info() {
  // build a completely new system info page
  uint8_t sysInfoPage[FLASH_SYSTEM_INFO_LEN + 1];
  uint8_t* sysInfoPagePtr = sysInfoPage;

  // set reset counter to 0
  sysInfoPage[FLASH_RESTART_COUNTER_ADDR] = 0;
  sysInfoPage[FLASH_RESTART_COUNTER_ADDR + 1] = 0;

  // set deployment counter to 0
  sysInfoPage[FLASH_DEPLOYMENT_COUNTER_ADDR] = 0;

  // set default transmission configuration
  sysInfoPage[FLASH_TRANSMISSIONS_ENABLED] = 1;

  // set default callsign length
  sysInfoPage[FLASH_CALLSIGN_LEN_ADDR] = strlen(CALLSIGN_DEFAULT);

  // set default callsign
  memcpy(sysInfoPagePtr + FLASH_CALLSIGN_ADDR, CALLSIGN_DEFAULT, strlen(CALLSIGN_DEFAULT));

  // set default receive windows
  sysInfoPage[FLASH_FSK_RECEIVE_LEN_ADDR] = FSK_RECEIVE_WINDOW_LENGTH;
  sysInfoPage[FLASH_LORA_RECEIVE_LEN_ADDR] = LORA_RECEIVE_WINDOW_LENGTH;

  // reset uptime counter
  uint32_t uptimeCounter = 0;
  memcpy(sysInfoPagePtr + FLASH_UPTIME_COUNTER_ADDR, &uptimeCounter, sizeof(uint32_t));

  // reset frame counters
  uint16_t frameCounter = 0;
  memcpy(sysInfoPagePtr + FLASH_LORA_VALID_COUNTER_ADDR, &frameCounter, sizeof(uint16_t));
  memcpy(sysInfoPagePtr + FLASH_LORA_INVALID_COUNTER_ADDR, &frameCounter, sizeof(uint16_t));
  memcpy(sysInfoPagePtr + FLASH_FSK_VALID_COUNTER_ADDR, &frameCounter, sizeof(uint16_t));
  memcpy(sysInfoPagePtr + FLASH_FSK_INVALID_COUNTER_ADDR, &frameCounter, sizeof(uint16_t));

  // set default low power mode configuration
  sysInfoPage[FLASH_LOW_POWER_MODE_ENABLED] = 1;
  sysInfoPage[FLASH_LOW_POWER_MODE_ACTIVE] = 0;

  // write the default system info
  PersistentStorage_Write(FLASH_SYSTEM_INFO_START, sysInfoPage, FLASH_SYSTEM_INFO_LEN + 1);
}

void PersistentStorage_Read(uint32_t addr, uint8_t* buff, size_t len) {
  uint8_t cmdBuff[] = {MX25L51245G_CMD_READ, (uint8_t)((addr >> 24) & 0xFF), (uint8_t)((addr >> 16) & 0xFF), (uint8_t)((addr >> 8) & 0xFF), (uint8_t)(addr & 0xFF)};
  PersistentStorage_SPItranscation(cmdBuff, 5, false, buff, len);
}

void PersistentStorage_Write(uint32_t addr, uint8_t* buff, size_t len) {
  // erase requested sector
  PersistentStorage_SectorErase(addr);

  // set WEL bit again
  PersistentStorage_WaitForWriteEnable();

  // write page
  uint8_t cmdBuff[] = {MX25L51245G_CMD_PP, (uint8_t)((addr >> 24) & 0xFF), (uint8_t)((addr >> 16) & 0xFF), (uint8_t)((addr >> 8) & 0xFF), (uint8_t)(addr & 0xFF)};
  PersistentStorage_SPItranscation(cmdBuff, 5, true, buff, len);

  // wait until page is written
  PersistentStorage_WaitForWriteInProgress();
}

void PersistentStorage_SectorErase(uint32_t addr) {
  // set WEL bit
  PersistentStorage_WaitForWriteEnable();

  // erase required sector
  uint8_t cmdBuf[] = {MX25L51245G_CMD_SE, (uint8_t)((addr >> 24) & 0xFF), (uint8_t)((addr >> 16) & 0xFF), (uint8_t)((addr >> 8) & 0xFF), (uint8_t)(addr & 0xFF)};
  PersistentStorage_SPItranscation(cmdBuf, 5, false, NULL, 0);

  // wait until sector is erased
  PersistentStorage_WaitForWriteInProgress();
}

void PersistentStorage_WriteEnable() {
  PersistentStorage_SPItranscation(MX25L51245G_CMD_WREN);
}

void PersistentStorage_WriteDisable() {
  PersistentStorage_SPItranscation(MX25L51245G_CMD_WRDI);
}

uint8_t PersistentStorage_ReadManufacturerID() {
  uint8_t cmdBuf[] = {MX25L51245G_CMD_REMS, 0x00, 0x00, 0x00};
  uint8_t buf[2];
  PersistentStorage_SPItranscation(cmdBuf, 4, false, buf, 2);
  return(buf[0]);
}

uint8_t PersistentStorage_ReadStatusRegister() {
  uint8_t buf[1];
  PersistentStorage_SPItranscation(MX25L51245G_CMD_RDSR, false, buf, 1);
  return(buf[0]);
}

uint8_t PersistentStorage_ReadConfigRegister() {
  uint8_t buf[1];
  PersistentStorage_SPItranscation(MX25L51245G_CMD_RDCR, false, buf, 1);
  return(buf[0]);
}

uint8_t PersistentStorage_ReadSecurityRegister() {
  uint8_t buf[1];
  PersistentStorage_SPItranscation(MX25L51245G_CMD_RDSCUR, false, buf, 1);
  return(buf[0]);
}

void PersistentStorage_Enter4ByteMode() {
  PersistentStorage_SPItranscation(MX25L51245G_CMD_EN4B);
}

void PersistentStorage_Exit4ByteMode() {
  PersistentStorage_SPItranscation(MX25L51245G_CMD_EX4B);
}

void PersistentStorage_Reset() {
  pinMode(FLASH_RESET, OUTPUT);
  digitalWrite(FLASH_RESET, LOW);
  delayMicroseconds(100);
  pinMode(FLASH_RESET, INPUT);
}

void PersistentStorage_WriteStatusRegister(uint8_t sr, uint8_t cr) {
  uint8_t buf[] = {sr, cr};
  PersistentStorage_WaitForWriteEnable();
  PersistentStorage_SPItranscation(MX25L51245G_CMD_WRSR, true, buf, 2);
  PersistentStorage_WriteDisable();
}

bool PersistentStorage_WaitForWriteEnable(uint8_t timeout) {
  // start the timer
  uint32_t start = millis();

  // repeat until WEL bit is set
  while(!(PersistentStorage_ReadStatusRegister() & MX25L51245G_SR_WEL)) {
    PersistentStorage_WriteEnable();
    delayMicroseconds(10);

    // check timeout
    if(millis() - start >= timeout) {
      return(false);
    }
  }
  return(true);
}

bool PersistentStorage_WaitForWriteInProgress(uint8_t timeout) {
  // start the timer
  uint32_t start = millis();

  // repeat as long as WIP bit is set
  while(PersistentStorage_ReadStatusRegister() & MX25L51245G_SR_WIP) {
    delayMicroseconds(10);

    // check timeout
    if(millis() - start >= timeout) {
      return(false);
    }
  }
  return(true);
}

void PersistentStorage_SPItranscation(uint8_t cmd, bool write, uint8_t* data, size_t numBytes) {
  uint8_t cmdBuf[] = {cmd};
  PersistentStorage_SPItranscation(cmdBuf, 1, write, data, numBytes);
}

void PersistentStorage_SPItranscation(uint8_t* cmd, uint8_t cmdLen, bool write, uint8_t* data, size_t numBytes) {
  digitalWrite(FLASH_CS, LOW);
  FlashSPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));

  // send command
  for(uint8_t n = 0; n < cmdLen; n++) {
    // send byte
    FlashSPI.transfer(cmd[n]);
  }

  // send data
  if(write) {
    for(size_t n = 0; n < numBytes; n++) {
      // send byte
      FlashSPI.transfer(data[n]);
    }

  } else {
    for(size_t n = 0; n < numBytes; n++) {
      data[n] = FlashSPI.transfer(MX25L51245G_CMD_NOP);
    }
  }

  FlashSPI.endTransaction();
  digitalWrite(FLASH_CS, HIGH);
}
