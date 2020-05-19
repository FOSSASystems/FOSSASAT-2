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
#define FLASH_SYSTEM_INFO_LEN                          (FLASH_EXT_PAGE_SIZE)
#define FLASH_SYSTEM_INFO_CRC                           0x000000F8  //  0x000000F8    0x000000FB
#define FLASH_MEMORY_ERROR_COUNTER                      0x000000FC  //  0x000000FC    0x000000FF

/*
    CRC32 Implementation based on GCC libiberty library, under the following license:

    Copyright (C) 2009-2020 Free Software Foundation, Inc.
    
    This file is part of the libiberty library.
    
    This file is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    In addition to the permissions in the GNU General Public License, the
    Free Software Foundation gives you unlimited permission to link the
    compiled version of this file into combinations with other programs,
    and to distribute those combinations without any restriction coming
    from the use of this file.  (The General Public License restrictions
    do apply in other respects; for example, they cover modification of
    the file, and distribution when not linked into a combined
    executable.)
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street - Fifth Floor, Boston, MA 02110-1301, USA.
*/

static const uint32_t crc32_table[] PROGMEM =
{
  0x00000000, 0x04c11db7, 0x09823b6e, 0x0d4326d9,
  0x130476dc, 0x17c56b6b, 0x1a864db2, 0x1e475005,
  0x2608edb8, 0x22c9f00f, 0x2f8ad6d6, 0x2b4bcb61,
  0x350c9b64, 0x31cd86d3, 0x3c8ea00a, 0x384fbdbd,
  0x4c11db70, 0x48d0c6c7, 0x4593e01e, 0x4152fda9,
  0x5f15adac, 0x5bd4b01b, 0x569796c2, 0x52568b75,
  0x6a1936c8, 0x6ed82b7f, 0x639b0da6, 0x675a1011,
  0x791d4014, 0x7ddc5da3, 0x709f7b7a, 0x745e66cd,
  0x9823b6e0, 0x9ce2ab57, 0x91a18d8e, 0x95609039,
  0x8b27c03c, 0x8fe6dd8b, 0x82a5fb52, 0x8664e6e5,
  0xbe2b5b58, 0xbaea46ef, 0xb7a96036, 0xb3687d81,
  0xad2f2d84, 0xa9ee3033, 0xa4ad16ea, 0xa06c0b5d,
  0xd4326d90, 0xd0f37027, 0xddb056fe, 0xd9714b49,
  0xc7361b4c, 0xc3f706fb, 0xceb42022, 0xca753d95,
  0xf23a8028, 0xf6fb9d9f, 0xfbb8bb46, 0xff79a6f1,
  0xe13ef6f4, 0xe5ffeb43, 0xe8bccd9a, 0xec7dd02d,
  0x34867077, 0x30476dc0, 0x3d044b19, 0x39c556ae,
  0x278206ab, 0x23431b1c, 0x2e003dc5, 0x2ac12072,
  0x128e9dcf, 0x164f8078, 0x1b0ca6a1, 0x1fcdbb16,
  0x018aeb13, 0x054bf6a4, 0x0808d07d, 0x0cc9cdca,
  0x7897ab07, 0x7c56b6b0, 0x71159069, 0x75d48dde,
  0x6b93dddb, 0x6f52c06c, 0x6211e6b5, 0x66d0fb02,
  0x5e9f46bf, 0x5a5e5b08, 0x571d7dd1, 0x53dc6066,
  0x4d9b3063, 0x495a2dd4, 0x44190b0d, 0x40d816ba,
  0xaca5c697, 0xa864db20, 0xa527fdf9, 0xa1e6e04e,
  0xbfa1b04b, 0xbb60adfc, 0xb6238b25, 0xb2e29692,
  0x8aad2b2f, 0x8e6c3698, 0x832f1041, 0x87ee0df6,
  0x99a95df3, 0x9d684044, 0x902b669d, 0x94ea7b2a,
  0xe0b41de7, 0xe4750050, 0xe9362689, 0xedf73b3e,
  0xf3b06b3b, 0xf771768c, 0xfa325055, 0xfef34de2,
  0xc6bcf05f, 0xc27dede8, 0xcf3ecb31, 0xcbffd686,
  0xd5b88683, 0xd1799b34, 0xdc3abded, 0xd8fba05a,
  0x690ce0ee, 0x6dcdfd59, 0x608edb80, 0x644fc637,
  0x7a089632, 0x7ec98b85, 0x738aad5c, 0x774bb0eb,
  0x4f040d56, 0x4bc510e1, 0x46863638, 0x42472b8f,
  0x5c007b8a, 0x58c1663d, 0x558240e4, 0x51435d53,
  0x251d3b9e, 0x21dc2629, 0x2c9f00f0, 0x285e1d47,
  0x36194d42, 0x32d850f5, 0x3f9b762c, 0x3b5a6b9b,
  0x0315d626, 0x07d4cb91, 0x0a97ed48, 0x0e56f0ff,
  0x1011a0fa, 0x14d0bd4d, 0x19939b94, 0x1d528623,
  0xf12f560e, 0xf5ee4bb9, 0xf8ad6d60, 0xfc6c70d7,
  0xe22b20d2, 0xe6ea3d65, 0xeba91bbc, 0xef68060b,
  0xd727bbb6, 0xd3e6a601, 0xdea580d8, 0xda649d6f,
  0xc423cd6a, 0xc0e2d0dd, 0xcda1f604, 0xc960ebb3,
  0xbd3e8d7e, 0xb9ff90c9, 0xb4bcb610, 0xb07daba7,
  0xae3afba2, 0xaafbe615, 0xa7b8c0cc, 0xa379dd7b,
  0x9b3660c6, 0x9ff77d71, 0x92b45ba8, 0x9675461f,
  0x8832161a, 0x8cf30bad, 0x81b02d74, 0x857130c3,
  0x5d8a9099, 0x594b8d2e, 0x5408abf7, 0x50c9b640,
  0x4e8ee645, 0x4a4ffbf2, 0x470cdd2b, 0x43cdc09c,
  0x7b827d21, 0x7f436096, 0x7200464f, 0x76c15bf8,
  0x68860bfd, 0x6c47164a, 0x61043093, 0x65c52d24,
  0x119b4be9, 0x155a565e, 0x18197087, 0x1cd86d30,
  0x029f3d35, 0x065e2082, 0x0b1d065b, 0x0fdc1bec,
  0x3793a651, 0x3352bbe6, 0x3e119d3f, 0x3ad08088,
  0x2497d08d, 0x2056cd3a, 0x2d15ebe3, 0x29d4f654,
  0xc5a92679, 0xc1683bce, 0xcc2b1d17, 0xc8ea00a0,
  0xd6ad50a5, 0xd26c4d12, 0xdf2f6bcb, 0xdbee767c,
  0xe3a1cbc1, 0xe760d676, 0xea23f0af, 0xeee2ed18,
  0xf0a5bd1d, 0xf464a0aa, 0xf9278673, 0xfde69bc4,
  0x89b8fd09, 0x8d79e0be, 0x803ac667, 0x84fbdbd0,
  0x9abc8bd5, 0x9e7d9662, 0x933eb0bb, 0x97ffad0c,
  0xafb010b1, 0xab710d06, 0xa6322bdf, 0xa2f33668,
  0xbcb4666d, 0xb8757bda, 0xb5365d03, 0xb1f740b4
};

/*
  This CRC can be specified as:

  Width  : 32
  Poly   : 0x04c11db7
  Init   : parameter, typically 0xffffffff
  RefIn  : false
  RefOut : false
  XorOut : 0

  This differs from the "standard" CRC-32 algorithm in that the values
  are not reflected, and there is no final XOR value.  These differences
  make it easy to compose the values of multiple blocks.
 */

uint32_t CRC32_Get(uint8_t* buff, size_t len, uint32_t initial = 0xFFFFFFFF);

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

void PersistentStorage_SPItransaction(uint8_t cmd, bool write = true, uint8_t* data = NULL, size_t numBytes = 0);
void PersistentStorage_SPItransaction(uint8_t* cmd, uint8_t cmdLen, bool write, uint8_t* data, size_t numBytes);

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
  if(addr > (FLASH_SYSTEM_INFO_LEN - 1)) {
    return;
  }

  // read the current system info page
  uint8_t currSysInfoPage[FLASH_SYSTEM_INFO_LEN];
  PersistentStorage_Read(FLASH_SYSTEM_INFO_START, currSysInfoPage, FLASH_SYSTEM_INFO_LEN);

  // check CRC of the current page
  uint32_t currCrc = 0;
  memcpy(&currCrc, currSysInfoPage + FLASH_SYSTEM_INFO_CRC, sizeof(uint32_t));
  if(currCrc != CRC32_Get(currSysInfoPage, FLASH_SYSTEM_INFO_CRC)) {
    // memory error happened between last write and now, increment the counter
    uint32_t errCounter = 0;
    memcpy(&errCounter, currSysInfoPage + FLASH_MEMORY_ERROR_COUNTER, sizeof(uint32_t));
    errCounter++;
    memcpy(currSysInfoPage + FLASH_MEMORY_ERROR_COUNTER, &errCounter, sizeof(uint32_t));
  }
  
  // check if we need to update
  uint8_t newSysInfoPage[FLASH_SYSTEM_INFO_LEN];
  memcpy(newSysInfoPage, currSysInfoPage, FLASH_SYSTEM_INFO_LEN);
  memcpy(newSysInfoPage + addr, &t, sizeof(T));
  if(memcmp(currSysInfoPage, newSysInfoPage, FLASH_SYSTEM_INFO_LEN) == 0) {
    // the value is already there, no need to write
    return;
  }

  // update CRC
  uint32_t crc = CRC32_Get(newSysInfoPage, FLASH_SYSTEM_INFO_CRC);
  memcpy(newSysInfoPage + FLASH_SYSTEM_INFO_CRC, &crc, sizeof(uint32_t));

  // update the page
  PersistentStorage_Write(FLASH_SYSTEM_INFO_START, newSysInfoPage, FLASH_SYSTEM_INFO_LEN);
}

template <typename T>
void PersistentStorage_Update_Stat(uint32_t addr, T val) {
  uint32_t statAddr = addr - FLASH_STATS;
  size_t typeSize = sizeof(T);
  
  // read the current page
  uint8_t statBuff[FLASH_EXT_PAGE_SIZE];
  PersistentStorage_Read(FLASH_STATS, statBuff, FLASH_EXT_PAGE_SIZE);

  // get min/avg/max
  T min = 0;
  memcpy(&min, statBuff + statAddr, typeSize);
  T avg = 0;
  memcpy(&avg, statBuff + statAddr + typeSize, typeSize);
  T max = 0;
  memcpy(&max, statBuff + statAddr + 2*typeSize, typeSize);

  // update stats
  if(val < min) {
    memcpy(statBuff + statAddr, &val, typeSize);
  }
  
  avg = (avg + val)/((T)2);
  memcpy(statBuff + statAddr + typeSize, &avg, typeSize);

  if(val > max) {
    memcpy(statBuff + statAddr + 2*typeSize, &val, typeSize);
  }

  // write the updated buffer
  PersistentStorage_Write(FLASH_STATS, statBuff, FLASH_EXT_PAGE_SIZE);
}

void PersistentStorage_Update_Stats(uint8_t flags);
void PersistentStorage_Reset_Stats();

#endif
