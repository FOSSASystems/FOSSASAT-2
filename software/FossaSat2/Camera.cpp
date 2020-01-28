#include "Camera.h"

uint8_t Camera_Init(uint8_t pictureSize) {
  // check provided value
  if(!((pictureSize >= OV2640_160x120) && (pictureSize <= OV2640_1600x1200))) {
    FOSSASAT_DEBUG_PRINT(F("Error - invalid picture size: "));
    FOSSASAT_DEBUG_PRINTLN(pictureSize);
    return 1;
  }

  FOSSASAT_DEBUG_PRINT(F("Picture size: "));
  FOSSASAT_DEBUG_PRINTLN(pictureSize);
  
  // reset CPLD
  camera.write_reg(0x07, 0x80);
  PowerControl_Wait(500);
  camera.write_reg(0x07, 0x00);
  PowerControl_Wait(500);

  // check camera SPI
  uint32_t start = millis();
  uint8_t state = 0;
  while(millis() - start <= 5000) {
    PowerControl_Watchdog_Heartbeat();
    
    // write to test register
    camera.write_reg(ARDUCHIP_TEST1, 0x55);

    // read the value back
    uint8_t testValue = camera.read_reg(ARDUCHIP_TEST1);
    if(testValue != 0x55){
      FOSSASAT_DEBUG_PRINT(F("Camera SPI test failed, got 0x"));
      FOSSASAT_DEBUG_PRINTLN(testValue, HEX);
      state = 2;
    } else {
      FOSSASAT_DEBUG_PRINTLN(F("Camera SPI test OK"));
      state = 0;
      break;
    }
    
    PowerControl_Wait(500);
  }

  if(state != 0) {
    return(state);
  }

  // check camera I2C
  start = millis();
  while(millis() - start <= 5000) {
    uint8_t vid = 0;
    uint8_t pid = 0;
    camera.wrSensorReg8_8(0xff, 0x01);
    camera.rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid); //CD
    camera.rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);  //CD
    if((vid != 0x26) && ((pid != 0x41) || (pid != 0x42))){
      FOSSASAT_DEBUG_PRINTLN(F("Unexpected vendor/product ID!"));
      FOSSASAT_DEBUG_PRINT(F("Expected 0x26 0x41/0x42, got 0x"));
      FOSSASAT_DEBUG_PRINT(vid, HEX);
      FOSSASAT_DEBUG_PRINT(F(" 0x"));
      FOSSASAT_DEBUG_PRINTLN(pid, HEX);
      state = 2;
    } else {
      FOSSASAT_DEBUG_PRINTLN(F("Detected OV2640"));
      state = 0;
      break;
    }
    
    PowerControl_Wait(500);
  }

  if(state != 0) {
    return(state);
  }
  
  // set JPEG mode and initialize
  camera.set_format(JPEG);
  camera.InitCAM();

  // set default size
  camera.OV2640_set_JPEG_size(pictureSize);
  PowerControl_Wait(1000);
  camera.clear_fifo_flag();

  return(state);
}

void Camera_Capture() {
  // flush FIFO
  camera.flush_fifo();
  camera.clear_fifo_flag();
  
  // start capture
  FOSSASAT_DEBUG_PRINTLN(F("Capture start."));
  camera.start_capture();

  // wait for capture done
  uint32_t start = millis();
  while(!camera.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)) {
    PowerControl_Wait(500);
    if(millis() - start >= 5000) {
      FOSSASAT_DEBUG_PRINTLN(F("Timed out waiting for capture end!"));
      return;
    }
  }

  // read image length
  FOSSASAT_DEBUG_PRINTLN(F("Capture done, reading data."));
  uint32_t len = camera.read_fifo_length();
  if(len >= MAX_FIFO_SIZE) {
    FOSSASAT_DEBUG_PRINT(F("Image size is too large ("));
    FOSSASAT_DEBUG_PRINT(len)
    FOSSASAT_DEBUG_PRINTLN(F(" B)!"));
    return;
  } else if(len == 0) {
    FOSSASAT_DEBUG_PRINTLN(F("Image size is 0 B!"));
    return;
  }

  // write image length
  FOSSASAT_DEBUG_PRINT(F("Image size (bytes): "));
  FOSSASAT_DEBUG_PRINTLN(len);
  PersistentStorage_Set<uint32_t>(FLASH_IMAGE1_LENGTH, len);

  // erase image blocks in flash
  for(uint32_t i = 0; i < FLASH_IMAGE_NUM_64K_BLOCKS; i++) {
    PersistentStorage_64kBlockErase(FLASH_IMAGE1 + i*FLASH_64K_BLOCK_SIZE);
  }

  // read data and write them to flash
  camera.CS_LOW();
  camera.set_fifo_burst();

  // write the complete sectors first
  uint8_t dataBuffer[FLASH_SECTOR_SIZE];
  uint32_t i;
  for(i = 0; i < len / FLASH_SECTOR_SIZE; i++) {
    for(uint32_t j = 0; j < FLASH_SECTOR_SIZE; j++) {
      dataBuffer[j] = SPI.transfer(0x00);
    }

    // write a single sector
    PersistentStorage_Write(FLASH_IMAGE1 + i*FLASH_SECTOR_SIZE, dataBuffer, FLASH_SECTOR_SIZE, false);
  }

  // write the remaining sector
  uint32_t remLen = len - i*FLASH_SECTOR_SIZE;
  for(uint32_t j = 0; j < remLen; j++) {
    dataBuffer[j] = SPI.transfer(0x00);
  }
  PersistentStorage_Write(FLASH_IMAGE1 + i*FLASH_SECTOR_SIZE, dataBuffer, remLen, false);
  FOSSASAT_DEBUG_PRINTLN(F("Writing done"));
  
  camera.CS_HIGH();
  camera.clear_fifo_flag();
}
