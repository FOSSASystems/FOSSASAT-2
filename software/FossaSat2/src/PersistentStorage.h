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

bool PersistentStorage_Check_CRC(uint8_t* buff, uint32_t crcPos);

// system info functions - these only change the RAM buffer
void PersistentStorage_Increment_Counter(uint16_t addr);
void PersistentStorage_Increment_Frame_Counter(bool valid);
void PersistentStorage_Get_Callsign(char* buff, uint8_t len);
void PersistentStorage_Set_Callsign(char* newCallsign);
void PersistentStorage_Set_Buffer(uint8_t addr, uint8_t* buff, size_t len);
void PersistentStorage_Reset_System_Info();
void PersistentStorage_Reset_ADCS_Params();

// image properties functions
uint32_t PersistentStorage_Get_Image_Len(uint8_t slot);
uint32_t PersistentStorage_Get_Image_ScanStart(uint8_t slot);
uint32_t PersistentStorage_Get_Image_ScanEnd(uint8_t slot);
uint32_t PersistentStorage_Get_Image_Property(uint8_t slot, uint8_t offset);
void PersistentStorage_Set_Image_Properties(uint8_t slot, uint32_t len, uint32_t scanStart, uint32_t scanEnd);

// store & forward functions
uint8_t PersistentStorage_Get_Message(uint16_t slotNum, uint8_t* buff);
void PersistentStorage_Set_Message(uint16_t slotNum, uint8_t* buff, uint8_t len);

void PersistentStorage_Set_ADCS_Ephemerides(uint32_t row, float ephemerides[2*ADCS_NUM_AXES], uint8_t controllerId);
void PersistentStorage_Set_ADCS_Controller(uint8_t id, float controller[ADCS_NUM_AXES][2*ADCS_NUM_AXES]);

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

void PersistentStorage_SPItransaction(uint8_t cmd, bool write = true, uint8_t* data = NULL, size_t numBytes = 0);
void PersistentStorage_SPItransaction(uint8_t* cmd, uint8_t cmdLen, bool write, uint8_t* data, size_t numBytes);

// system info get/set only affect RAM buffer, changed values will be saved to flash only on main loop end!
template<typename T>
T PersistentStorage_SystemInfo_Get(uint8_t addr);
template<typename T>
void PersistentStorage_SystemInfo_Set(uint8_t addr, T t);

// generic get/set
template<typename T>
T PersistentStorage_Get(uint32_t addr);
template<typename T>
void PersistentStorage_Set(uint32_t addr, T t);

template <typename T>
void PersistentStorage_Update_Stat(uint8_t* statBuff, uint32_t addr, T val);
void PersistentStorage_Update_Stats(uint8_t flags);
void PersistentStorage_Reset_Stats();

#endif
