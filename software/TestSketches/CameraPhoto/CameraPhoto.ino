#include "FossaSat2.h"

/*
 * To turn the Serial dump into an actual JPEG:
 * 
 * 1. Make sure timetamp is disabled in Serial Monitor
 * 2. Take a picture
 * 3. Ctrl+C all the non-FF bytes
 * 4. Go to https://hexed.it/
 * 5. Insert clipboard data (Ctrl+V anywhere)
 * 6. Select "Create a new file" and specify data as "Hexadecimal values"
 * 7. In the top bar, click Export
 * 8. Rename the downloaded file .jpeg
 * 
 */


void setup() {
  // initialize debug port
  FOSSASAT_DEBUG_PORT.begin(FOSSASAT_DEBUG_SPEED);

  // setup hardware interfaces
  Configuration_Setup();

  // initialize external flash
  PersistentStorage_Reset();
  PersistentStorage_Enter4ByteMode();

  Sensors_Current_Setup(currSensorMPPT);

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
  uint32_t stepSize = FLASH_EXT_PAGE_SIZE / 2;
  for(uint32_t i = 0; i <= len / stepSize; i++) {
    FOSSASAT_DEBUG_PRINT_FLASH(imgAddress + i*stepSize, stepSize);

    // wait for a bit to emulate how long it takes to downlink picture of this size
    delay(2*RESPONSE_DELAY_SHORT + 1);
  }
}
