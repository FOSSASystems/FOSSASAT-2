#include "FossaSat2.h"

#include <SoftwareSerial.h>

SoftwareSerial softSerial(PA13, PA14);

#define NUM_BYTES                                       256
#define NUM_BYTES_W                                     256
#define ADDRESS                                         0x00000000

void showBytes(uint32_t addr, size_t len) {
  FOSSASAT_DEBUG_PORT.println(F("Read"));
  uint8_t readBuff[NUM_BYTES];
  PersistentStorage_Read(addr, readBuff, len);
  for(size_t i = 0; i < len/16; i++) {
    for(uint8_t j = 0; j < 16; j++) {
      FOSSASAT_DEBUG_PORT.print(readBuff[i*16 + j], HEX);
      FOSSASAT_DEBUG_PORT.print('\t');
    }
    FOSSASAT_DEBUG_PORT.println();
  }
}

void setup() {
  FOSSASAT_DEBUG_PORT.begin(FOSSASAT_DEBUG_SPEED);
  while(!FOSSASAT_DEBUG_PORT);
  FOSSASAT_DEBUG_PORT.println();

  SPI.setSCLK(SPI1_SCK);
  SPI.setMISO(SPI1_MISO);
  SPI.setMOSI(SPI1_MOSI);
  SPI.begin();
  
  pinMode(FLASH_CS, OUTPUT);
  digitalWrite(FLASH_CS, HIGH);

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

  // reset system info
  //PersistentStorage_Reset_System_Info();
  showBytes(FLASH_SYSTEM_INFO_START, FLASH_SYSTEM_INFO_LEN);

  // read reset counter
  uint16_t resetCounter = 0xFFFF;
  PersistentStorage_Get(FLASH_RESTART_COUNTER_ADDR, resetCounter);
  FOSSASAT_DEBUG_PORT.print(F("Reset counter = "));
  FOSSASAT_DEBUG_PORT.println(resetCounter);
  showBytes(FLASH_SYSTEM_INFO_START, FLASH_SYSTEM_INFO_LEN);

  // increment reset counter
  resetCounter++;
  PersistentStorage_Set(FLASH_RESTART_COUNTER_ADDR, resetCounter);
  showBytes(FLASH_SYSTEM_INFO_START, FLASH_SYSTEM_INFO_LEN);

  // read reset counter
  resetCounter = 0xFFFF;
  PersistentStorage_Get(FLASH_RESTART_COUNTER_ADDR, resetCounter);
  FOSSASAT_DEBUG_PORT.print(F("Reset counter = "));
  FOSSASAT_DEBUG_PORT.println(resetCounter);
  showBytes(FLASH_SYSTEM_INFO_START, FLASH_SYSTEM_INFO_LEN);
}

void loop() {
  
}
