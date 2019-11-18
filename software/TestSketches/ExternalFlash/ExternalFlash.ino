/*
    FossaSat2.h
*/

#include <SPI.h>
#include "Debug.h"

/*
    Configuration.h
*/

// SPI
#define SPI1_SCK                                        PA5
#define SPI1_MISO                                       PA6
#define SPI1_MOSI                                       PA7

// external flash
#define FLASH_CS                                        PC11
#define FLASH_RESET                                     PC12

void setup() {
  FOSSASAT_DEBUG_PORT.begin(FOSSASAT_DEBUG_SPEED);
  while(!FOSSASAT_DEBUG_PORT);
  FOSSASAT_DEBUG_PORT.println();

  SPI.setSCLK(SPI1_SCK);
  SPI.setMISO(SPI1_MISO);
  SPI.setMOSI(SPI1_MOSI);
  SPI.begin();

  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  digitalWrite(FLASH_CS, LOW);
  SPI.transfer(0x90); // REMS command
  SPI.transfer(0x00); // dummy byte
  SPI.transfer(0x00); // dummy byte
  SPI.transfer(0x00); // address byte
  uint8_t mfgID = SPI.transfer(0x00);
  uint8_t devID = SPI.transfer(0x00);
  SPI.endTransaction();
  digitalWrite(FLASH_CS, HIGH);

  FOSSASAT_DEBUG_PORT.print(mfgID, HEX);
  FOSSASAT_DEBUG_PORT.print('\t');
  FOSSASAT_DEBUG_PORT.println(devID, HEX);

  if(mfgID != 0xC0) {
    FOSSASAT_DEBUG_PORT.println(F("ERROR - Unexpected manufacturer ID!"));
    FOSSASAT_DEBUG_PORT.print(F("Got 0x"));
    FOSSASAT_DEBUG_PORT.print(mfgID, HEX);
    FOSSASAT_DEBUG_PORT.println(F(", expected 0xC0."));
  }
  
}

void loop() {
  
}
