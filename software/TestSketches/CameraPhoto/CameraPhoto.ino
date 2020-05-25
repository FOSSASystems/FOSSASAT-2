#include "FossaSat2.h"

void setup() {
  // initialize debug port
  FOSSASAT_DEBUG_PORT.begin(FOSSASAT_DEBUG_SPEED);
  FOSSASAT_DEBUG_PORT.println();

  // setup hardware interfaces
  Configuration_Setup();

  // initialize external flash
  PersistentStorage_Reset();
  PersistentStorage_Enter4ByteMode();

  Sensors_Setup_Current(currSensorMPPT, CURR_SENSOR_MPPT_OUTPUT_BUS, CURR_SENSOR_MPPT_OUTPUT_ADDRESS);

  // initialize camera
  digitalWrite(CAMERA_POWER_FET, HIGH);
  FOSSASAT_DEBUG_PORT.print(F("Camera init:\t"));
  FOSSASAT_DEBUG_PORT.println(Camera_Init(p320x240, Auto, Saturation0, Brightness0, Contrast0, Normal));
}

uint8_t slot = 0;
void loop() {
  FOSSASAT_DEBUG_PORT.print(F("Send anything to save picture to slot #"));
  FOSSASAT_DEBUG_PORT.println(slot);

  // wait for prompt
  while(!FOSSASAT_DEBUG_PORT.available());

  // dump the serial buffer
  delay(500);
  while(FOSSASAT_DEBUG_PORT.available()) {
    FOSSASAT_DEBUG_PORT.read();
  }

  // take a photo
  uint32_t len = Camera_Capture(slot);

  // show the bytes in serial monitor
  FOSSASAT_DEBUG_PRINT(F("Starting at address: 0x"));
  uint32_t imgAddress = FLASH_IMAGES_START + slot*FLASH_IMAGE_SLOT_SIZE;
  FOSSASAT_DEBUG_PRINTLN(imgAddress, HEX);
  for(uint32_t i = 0; i <= len / FLASH_EXT_PAGE_SIZE; i++) {
    FOSSASAT_DEBUG_PRINT_FLASH(imgAddress + i*FLASH_EXT_PAGE_SIZE, FLASH_EXT_PAGE_SIZE);
  }
}
