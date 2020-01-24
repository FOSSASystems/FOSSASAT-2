#include "Camera.h"

void Camera_Init() {
  // reset CPLD
  camera.write_reg(0x07, 0x80);
  delay(100);
  camera.write_reg(0x07, 0x00);
  delay(100);

  // check camera SPI
  uint32_t start = millis();
  while(millis() - start <= 5000) {
    // write to test register
    camera.write_reg(ARDUCHIP_TEST1, 0x55);

    // read the value back
    uint8_t testValue = camera.read_reg(ARDUCHIP_TEST1);
    if(testValue != 0x55){
      FOSSASAT_DEBUG_PRINT(F("Camera SPI test failed, got 0x"));
      FOSSASAT_DEBUG_PRINTLN(testValue, HEX);
      delay(500);
    } else {
      FOSSASAT_DEBUG_PRINT(F("Camera SPI test OK"));
      break;
    }

    PowerControl_Watchdog_Heartbeat();
  }

  // check camera type
  uint8_t vid = 0;
  uint8_t pid = 0;
  camera.wrSensorReg16_8(0xff, 0x01);
  camera.rdSensorReg16_8(OV2640_CHIPID_HIGH, &vid);
  camera.rdSensorReg16_8(OV2640_CHIPID_LOW, &pid);
  if((vid != 0x26) && ((pid != 0x41) || (pid != 0x42))){
    FOSSASAT_DEBUG_PRINTLN(F("Unexpected vendor/product ID!"));
    FOSSASAT_DEBUG_PRINT(F("Expected 0x26 0x41/0x42, got 0x"));
    FOSSASAT_DEBUG_PRINT(vid, HEX);
    FOSSASAT_DEBUG_PRINT(F(" 0x"));
    FOSSASAT_DEBUG_PRINTLN(pid, HEX);
  }

  // set JPEG mode and initialize
  camera.set_format(JPEG);
  camera.InitCAM();

  // set default size
  camera.OV2640_set_JPEG_size(OV2640_320x240);
  delay(1000);
  camera.clear_fifo_flag();
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
  //PersistentStorage_Write_External<uint32_t>(FLASH_IMAGE_CAPTURE_LENGTH, len);

  // read data
  camera.CS_LOW();
  camera.set_fifo_burst();
  for(uint32_t i = 0; i < len; i++) {
    //PersistentStorage_Write_External<uint8_t>(FLASH_IMAGE_CAPTURE, SPI.transfer(0x00));
  }
  camera.CS_HIGH();
}
