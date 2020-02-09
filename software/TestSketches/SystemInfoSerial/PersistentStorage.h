#ifndef _FOSSASAT_PERSISTENT_STORAGE_H
#define _FOSSASAT_PERSISTENT_STORAGE_H

#include "FossaSat2.h"

#define MX25L51245G_CMD_NOP                             0x00
#define MX25L51245G_CMD_WRSR                            0x01
#define MX25L51245G_CMD_PP                              0x02
#define MX25L51245G_CMD_READ                            0x03
#define MX25L51245G_CMD_WRDI                            0x04
#define MX25L51245G_CMD_RDSR                            0x05
#define MX25L51245G_CMD_WREN                            0x06
#define MX25L51245G_CMD_RDCR                            0x15
#define MX25L51245G_CMD_RDSCUR                          0x2B
#define MX25L51245G_CMD_SE                              0x20
#define MX25L51245G_CMD_EN4B                            0xB7
#define MX25L51245G_CMD_BE                              0xD8
#define MX25L51245G_CMD_EX4B                            0xE9
#define MX25L51245G_CMD_REMS                            0x90

#define MX25L51245G_SR_WEL                              0b00000010
#define MX25L51245G_SR_WIP                              0b00000001

#define FLASH_EXT_PAGE_SIZE                             0x00000100
#define FLASH_STATS                                     0x00001000
#define FLASH_SYSTEM_INFO_START                         0x00000000
#define FLASH_SYSTEM_INFO_LEN                          (0x0000004E + 1)  // final address in Flash map + 1 byte

void PersistentStorage_Increment_Counter(uint16_t addr);
void PersistentStorage_Increment_Frame_Counter(bool valid);
void PersistentStorage_Get_Callsign(char* buff, uint8_t len);
void PersistentStorage_Set_Callsign(char* newCallsign);
uint32_t PersistentStorage_Get_Image_Len(uint8_t slot);
void PersistentStorage_Set_Image_Len(uint8_t slot, uint32_t len);
void PersistentStorage_Set_Buffer(uint8_t addr, uint8_t* buff, uint8_t len);
void PersistentStorage_Reset_System_Info();
uint8_t PersistentStorage_Get_Message(uint16_t slotNum, uint8_t* buff);
void PersistentStorage_Set_Message(uint16_t slotNum, uint8_t* buff, uint8_t len);

void PersistentStorage_Read(uint32_t addr, uint8_t* buff, size_t len);
void PersistentStorage_Write(uint32_t addr, uint8_t* buff, size_t len, bool autoErase = true);

void PersistentStorage_SectorErase(uint32_t addr);
void PersistentStorage_64kBlockErase(uint32_t addr);

void PersistentStorage_WriteEnable();
void PersistentStorage_WriteDisable();
uint8_t PersistentStorage_ReadManufacturerID();
uint8_t PersistentStorage_ReadStatusRegister();
uint8_t PersistentStorage_ReadConfigRegister();
uint8_t PersistentStorage_ReadSecurityRegister();
void PersistentStorage_Enter4ByteMode();
void PersistentStorage_Exit4ByteMode();
void PersistentStorage_Reset();
void PersistentStorage_WriteStatusRegister(uint8_t sr, uint8_t cr);
bool PersistentStorage_WaitForWriteEnable(uint32_t timeout = 50);
bool PersistentStorage_WaitForWriteInProgress(uint32_t timeout = 50);

void PersistentStorage_SPItranscation(uint8_t cmd, bool write = true, uint8_t* data = NULL, size_t numBytes = 0);
void PersistentStorage_SPItranscation(uint8_t* cmd, uint8_t cmdLen, bool write, uint8_t* data, size_t numBytes);

// get/set only works in system info page!
template<typename T>
T PersistentStorage_Get(uint8_t addr) {
  uint8_t buff[FLASH_SYSTEM_INFO_LEN];
  PersistentStorage_Read(FLASH_SYSTEM_INFO_START + addr, buff, sizeof(T));
  T t;
  memcpy(&t, buff, sizeof(T));
  return(t);
}

template<typename T>
void PersistentStorage_Set(uint8_t addr, T t) {
  // check address is in system info
  if(addr > FLASH_SYSTEM_INFO_LEN) {
    return;
  }

  // read the current system info page
  uint8_t currSysInfoPage[FLASH_SYSTEM_INFO_LEN];
  PersistentStorage_Read(FLASH_SYSTEM_INFO_START, currSysInfoPage, FLASH_SYSTEM_INFO_LEN);

  // check if we need to update
  uint8_t newSysInfoPage[FLASH_SYSTEM_INFO_LEN];
  memcpy(newSysInfoPage, currSysInfoPage, FLASH_SYSTEM_INFO_LEN);
  memcpy(newSysInfoPage + addr, &t, sizeof(T));
  if(memcmp(currSysInfoPage, newSysInfoPage, FLASH_SYSTEM_INFO_LEN) == 0) {
    // the value is already there, no need to write
    return;
  }

  // we need to update
  PersistentStorage_Write(FLASH_SYSTEM_INFO_START, newSysInfoPage, FLASH_SYSTEM_INFO_LEN);
}

template <typename T>
void PersistentStorage_Update_Stat(uint32_t addr, T val) {
  // read the current page
  uint8_t statBuff[FLASH_EXT_PAGE_SIZE];
  PersistentStorage_Read(FLASH_STATS, statBuff, FLASH_EXT_PAGE_SIZE);

  // get min/avg/max
  T min;
  memcpy(&min, statBuff + addr, sizeof(T));
  T avg;
  memcpy(&avg, statBuff + addr + sizeof(T), sizeof(T));
  T max;
  memcpy(&max, statBuff + addr + 2*sizeof(T), sizeof(T));

  // update stats
  if(val < min) {
    memcpy(statBuff + addr, &val, sizeof(T));
  }
  
  avg = (avg + val)/2;
  memcpy(statBuff + addr + sizeof(T), &avg, sizeof(T));

  if(val > max) {
    memcpy(statBuff + addr + 2*sizeof(T), &val, sizeof(T));
  }

  // write the updated buffer
  PersistentStorage_Write(FLASH_STATS, statBuff, FLASH_EXT_PAGE_SIZE);
}

void PersistentStorage_Update_Stats(uint8_t flags);

#endif
