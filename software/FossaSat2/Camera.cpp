#include "Camera.h"

uint8_t Camera_Init() {
  // power up camera
  digitalWrite(CAMERA_POWER_FET, HIGH);
  
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
      state = 1;
    } else {
      FOSSASAT_DEBUG_PRINTLN(F("Camera SPI test OK"));
      state = 0;
      break;
    }
    
    PowerControl_Wait(500);
  }

  if(state != 0) {
    // power down camera
    digitalWrite(CAMERA_POWER_FET, LOW);
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
    // power down camera
    digitalWrite(CAMERA_POWER_FET, LOW);
    return(state);
  }
  
  // set JPEG mode and initialize
  camera.set_format(JPEG);
  camera.InitCAM();

  // set default size
  camera.OV2640_set_JPEG_size(OV2640_320x240);
  PowerControl_Wait(1000);
  camera.clear_fifo_flag();

  
  // power down camera
  digitalWrite(CAMERA_POWER_FET, LOW);
  return(state);
}

void Camera_Capture() {
  // power up camera
  digitalWrite(CAMERA_POWER_FET, HIGH);

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
      // power down camera
      digitalWrite(CAMERA_POWER_FET, LOW);
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
    // power down camera
    digitalWrite(CAMERA_POWER_FET, LOW);
    return;
  } else if(len == 0) {
    FOSSASAT_DEBUG_PRINTLN(F("Image size is 0 B!"));
    // power down camera
    digitalWrite(CAMERA_POWER_FET, LOW);
    return;
  }

  // write image length
  FOSSASAT_DEBUG_PRINT(F("Image size (bytes): "));
  FOSSASAT_DEBUG_PRINTLN(len);
  PersistentStorage_Set<uint32_t>(FLASH_IMAGE1_LENGTH, len);

  // erase image blocks in flash
  for(uint32_t i = 0; i < 8; i++) {
    PersistentStorage_64kBlockErase(FLASH_IMAGE1 + i*0x00010000);
  }

  // read data and write them to flash
  camera.CS_LOW();
  camera.set_fifo_burst();
  uint8_t dataBuffer[256];
  for(uint32_t i = 0; i < len; i += 256) {
    // read data 256 bytes at a time
    for(uint16_t j = 0; j < 0xFF; j++) {
      dataBuffer[j] = SPI.transfer(0x00);
    }

    // write a single sector
    PersistentStorage_Write(FLASH_IMAGE1 + i, dataBuffer, 256, false);
  }
  camera.CS_HIGH();

  // power down camera
  digitalWrite(CAMERA_POWER_FET, LOW);
}
