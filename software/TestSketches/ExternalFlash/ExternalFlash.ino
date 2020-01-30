#include "FossaSat2.h"

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

  // write something
  FOSSASAT_DEBUG_PRINT_FLASH(FLASH_IMAGES_START, FLASH_EXT_PAGE_SIZE);
  FOSSASAT_DEBUG_PRINTLN();
  FOSSASAT_DEBUG_PRINT_FLASH(FLASH_IMAGES_START + FLASH_64K_BLOCK_SIZE, FLASH_EXT_PAGE_SIZE);
  FOSSASAT_DEBUG_PRINTLN();
  uint8_t buff[FLASH_EXT_PAGE_SIZE];
  for(uint32_t i = 0; i < FLASH_EXT_PAGE_SIZE; i++) {
    buff[i] = (uint8_t)random(0, 255);
  }
  PersistentStorage_Write(FLASH_IMAGES_START, buff, FLASH_EXT_PAGE_SIZE);
  PersistentStorage_Write(FLASH_IMAGES_START + FLASH_64K_BLOCK_SIZE, buff, FLASH_EXT_PAGE_SIZE);
  FOSSASAT_DEBUG_PRINT_FLASH(FLASH_IMAGES_START, FLASH_EXT_PAGE_SIZE);
  FOSSASAT_DEBUG_PRINTLN();
  FOSSASAT_DEBUG_PRINT_FLASH(FLASH_IMAGES_START + FLASH_64K_BLOCK_SIZE, FLASH_EXT_PAGE_SIZE);
  FOSSASAT_DEBUG_PRINTLN();

  // erase
  PersistentStorage_64kBlockErase(FLASH_IMAGES_START);
  FOSSASAT_DEBUG_PRINT_FLASH(FLASH_IMAGES_START, FLASH_EXT_PAGE_SIZE);
  FOSSASAT_DEBUG_PRINTLN();
  FOSSASAT_DEBUG_PRINT_FLASH(FLASH_IMAGES_START + FLASH_64K_BLOCK_SIZE, FLASH_EXT_PAGE_SIZE);
  FOSSASAT_DEBUG_PRINTLN();

  
}

void loop() {
  
}
