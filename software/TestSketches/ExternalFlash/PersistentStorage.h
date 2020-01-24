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
#define MX25L51245G_CMD_EX4B                            0xE9
#define MX25L51245G_CMD_REMS                            0x90

#define MX25L51245G_SR_WEL                              0b00000010
#define MX25L51245G_SR_WIP                              0b00000001

// get/set only works in system info page!
uint8_t PersistentStorage_Get(uint8_t addr);
void PersistentStorage_Set(uint8_t addr, uint8_t val);
void PersistentStorage_Get_Callsign(char* buff, uint8_t len);
void PersistentStorage_Set_Callsign(char* newCallsign);
void PersistentStorage_Reset_System_Info();

void PersistentStorage_Read(uint32_t addr, uint8_t* buff, size_t len);
void PersistentStorage_Write(uint32_t addr, uint8_t* buff, size_t len);

void PersistentStorage_SectorErase(uint32_t addr);

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
bool PersistentStorage_WaitForWriteEnable(uint8_t timeout = 50);
bool PersistentStorage_WaitForWriteInProgress(uint8_t timeout = 50);

void PersistentStorage_SPItranscation(uint8_t cmd, bool write = true, uint8_t* data = NULL, size_t numBytes = 0);
void PersistentStorage_SPItranscation(uint8_t* cmd, uint8_t cmdLen, bool write, uint8_t* data, size_t numBytes);

#endif
