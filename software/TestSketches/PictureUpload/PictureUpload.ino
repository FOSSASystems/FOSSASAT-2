#include "FossaSat2.h"

uint32_t fetchData(uint8_t slot) {
  uint8_t buff[FLASH_EXT_PAGE_SIZE];
  uint32_t pictureLen = 0;
  for(uint32_t addr = FLASH_IMAGES_START + slot*FLASH_IMAGE_SLOT_SIZE; addr < FLASH_IMAGES_START + (slot + 1)*FLASH_IMAGE_SLOT_SIZE; addr += FLASH_EXT_PAGE_SIZE) {
    uint16_t buffPos = 0;
    uint32_t start = millis();
    while(buffPos < FLASH_EXT_PAGE_SIZE) {
      while(FOSSASAT_DEBUG_PORT.available()) {
        buff[buffPos++] = FOSSASAT_DEBUG_PORT.read();
        //FOSSASAT_DEBUG_PORT.println(buffPos);
      }
      if(millis() - start >= 5000) {
        PersistentStorage_Write(addr, buff, buffPos, false);
        pictureLen += buffPos;
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        digitalWrite(WATCHDOG_IN, !digitalRead(WATCHDOG_IN));
        return(pictureLen);
      }
    }
    PersistentStorage_Write(addr, buff, buffPos, false);
    pictureLen += buffPos;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    digitalWrite(WATCHDOG_IN, !digitalRead(WATCHDOG_IN));
  }
  return(pictureLen);
}

void setup() {
  FOSSASAT_DEBUG_PORT.begin(FOSSASAT_DEBUG_SPEED);
  while(!FOSSASAT_DEBUG_PORT);
  FOSSASAT_DEBUG_PORT.println();

  FlashSPI.begin();

  pinMode(WATCHDOG_IN, OUTPUT);
  pinMode(FLASH_CS, OUTPUT);
  digitalWrite(FLASH_CS, HIGH);
  pinMode(FLASH_RESET, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  PersistentStorage_Reset();
  PersistentStorage_Enter4ByteMode();

  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  uint32_t lastBeat = millis();
  while(!FOSSASAT_DEBUG_PORT.available()) {
    if(millis() - lastBeat >= 1000) {
      digitalWrite(WATCHDOG_IN, !digitalRead(WATCHDOG_IN));
      lastBeat = millis();
    }
  }
  uint8_t slot = FOSSASAT_DEBUG_PORT.read();

  // erase slot
  for(uint32_t addr = FLASH_IMAGES_START + slot*FLASH_IMAGE_SLOT_SIZE; addr < FLASH_IMAGES_START + ((uint32_t)slot + (uint32_t)1)*FLASH_IMAGE_SLOT_SIZE; addr += FLASH_64K_BLOCK_SIZE) {
    PersistentStorage_64kBlockErase(addr);
    digitalWrite(WATCHDOG_IN, !digitalRead(WATCHDOG_IN));
  }
  
  // read data
  uint32_t writtenBytes = fetchData(slot);
  PersistentStorage_Set_Image_Properties(slot, writtenBytes, FLASH_IMAGES_START + slot*FLASH_IMAGE_SLOT_SIZE, FLASH_IMAGES_START + slot*FLASH_IMAGE_SLOT_SIZE + writtenBytes);
  digitalWrite(LED_BUILTIN, LOW);
}
