#include "FossaSat2.h"

void setup() {
  FOSSASAT_DEBUG_PORT.begin(FOSSASAT_DEBUG_SPEED);
  while(!FOSSASAT_DEBUG_PORT);

  while(!FOSSASAT_DEBUG_PORT.available());

  FlashSPI.begin();
  
  pinMode(FLASH_CS, OUTPUT);
  digitalWrite(FLASH_CS, HIGH);
  pinMode(FLASH_RESET, INPUT);

  PersistentStorage_Reset();
  PersistentStorage_Enter4ByteMode();

  uint8_t buff[FLASH_EXT_PAGE_SIZE];
  uint32_t startAddr = 0;
  for(uint32_t i = startAddr; i < FLASH_CHIP_SIZE; i += FLASH_EXT_PAGE_SIZE) {
    PersistentStorage_Read(i, buff, FLASH_EXT_PAGE_SIZE);
    for(uint32_t j = 0; j < FLASH_EXT_PAGE_SIZE; j++) {
      FOSSASAT_DEBUG_PORT.write(buff[j]);
    }
    delay(200);
  }
  
}

void loop() {
  
}
