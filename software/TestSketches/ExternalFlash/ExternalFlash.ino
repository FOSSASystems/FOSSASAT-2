#include "FossaSat2.h"

#define NUM_BYTES                                       256

void showBytes(uint32_t addr, size_t len) {
  uint8_t readBuff[NUM_BYTES];
  PersistentStorage_Read(addr, readBuff, len);
  char buff[16];

  if(len < 16) {
    for(uint8_t i = 0; i < len; i++) {
      sprintf(buff, "%02x ", readBuff[i]);
      FOSSASAT_DEBUG_PORT.print(buff);
    }
    FOSSASAT_DEBUG_PORT.println();
  } else {
    for(size_t i = 0; i < len/16; i++) {
      for(uint8_t j = 0; j < 16; j++) {
        sprintf(buff, "%02x ", readBuff[i*16 + j]);
        FOSSASAT_DEBUG_PORT.print(buff);
      }
      FOSSASAT_DEBUG_PORT.println();
    }
  }  
}

void setup() {
  FOSSASAT_DEBUG_PORT.begin(FOSSASAT_DEBUG_SPEED);
  while(!FOSSASAT_DEBUG_PORT);
  FOSSASAT_DEBUG_PORT.println();

  FlashSPI.begin();
  
  pinMode(FLASH_CS, OUTPUT);
  digitalWrite(FLASH_CS, HIGH);
  pinMode(FLASH_RESET, INPUT);

  PersistentStorage_Reset();
  PersistentStorage_Enter4ByteMode();

  uint8_t mfgID = PersistentStorage_ReadManufacturerID();
  if(mfgID != 0xC2) {
    FOSSASAT_DEBUG_PORT.println(F("ERROR - Unexpected manufacturer ID!"));
    FOSSASAT_DEBUG_PORT.print(F("Got 0x"));
    FOSSASAT_DEBUG_PORT.print(mfgID, HEX);
    FOSSASAT_DEBUG_PORT.println(F(", expected 0xC2."));
    while(true);
  }

  FOSSASAT_DEBUG_PORT.print(F("Mfg ID   = 0x"));
  FOSSASAT_DEBUG_PORT.println(mfgID, HEX);
  FOSSASAT_DEBUG_PORT.print(F("Status   = 0b"));
  FOSSASAT_DEBUG_PORT.println(PersistentStorage_ReadStatusRegister(), BIN);
  FOSSASAT_DEBUG_PORT.print(F("Config   = 0b"));
  FOSSASAT_DEBUG_PORT.println(PersistentStorage_ReadConfigRegister(), BIN);
  FOSSASAT_DEBUG_PORT.print(F("Security = 0b"));
  FOSSASAT_DEBUG_PORT.println(PersistentStorage_ReadSecurityRegister(), BIN);

  // read image 1
  uint32_t len = PersistentStorage_Get<uint32_t>(FLASH_IMAGE1_LENGTH);
  FOSSASAT_DEBUG_PORT.print(F("Image 1 length: "));
  FOSSASAT_DEBUG_PORT.println(len);

  // read the complete sectors first
  uint32_t i;
  for(i = 0; i < len / FLASH_SECTOR_SIZE; i++) {
    showBytes(FLASH_IMAGE1 + i*FLASH_SECTOR_SIZE, FLASH_SECTOR_SIZE);
  }

  // read the remaining sector
  uint32_t remLen = len - i*FLASH_SECTOR_SIZE;
  showBytes(FLASH_IMAGE1 + i*FLASH_SECTOR_SIZE, remLen);
}

void loop() {
  
}
