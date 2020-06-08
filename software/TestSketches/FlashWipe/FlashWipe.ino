#include "FossaSat2.h"

void setup() {
  FOSSASAT_DEBUG_PORT.begin(FOSSASAT_DEBUG_SPEED);
  while(!FOSSASAT_DEBUG_PORT);
  FOSSASAT_DEBUG_PORT.println();

  FlashSPI.begin();
  
  pinMode(FLASH_CS, OUTPUT);
  digitalWrite(FLASH_CS, HIGH);
  pinMode(FLASH_RESET, INPUT);

  pinMode(WATCHDOG_IN, OUTPUT);

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
  PowerControl_Watchdog_Heartbeat();

  FOSSASAT_DEBUG_PORT.println(F("-------"));
  FOSSASAT_DEBUG_PRINT_FLASH(FLASH_SYSTEM_INFO, FLASH_EXT_PAGE_SIZE);
  FOSSASAT_DEBUG_PORT.println(F("System info wipe start"));
  PersistentStorage_Reset_System_Info();
  PowerControl_Watchdog_Heartbeat();
  FOSSASAT_DEBUG_PORT.println(F("System info wipe done"));
  FOSSASAT_DEBUG_PRINT_FLASH(FLASH_SYSTEM_INFO, FLASH_EXT_PAGE_SIZE);

  FOSSASAT_DEBUG_PORT.println(F("-------"));
  FOSSASAT_DEBUG_PRINT_FLASH(FLASH_STATS, FLASH_EXT_PAGE_SIZE);
  FOSSASAT_DEBUG_PORT.println(F("Stats wipe start"));
  PersistentStorage_Reset_Stats();
  PowerControl_Watchdog_Heartbeat();
  FOSSASAT_DEBUG_PORT.println(F("Stats wipe done"));
  FOSSASAT_DEBUG_PRINT_FLASH(FLASH_STATS, FLASH_EXT_PAGE_SIZE);

  FOSSASAT_DEBUG_PORT.println(F("-------"));
  FOSSASAT_DEBUG_PRINT_FLASH(FLASH_STORE_AND_FORWARD_LENGTH, FLASH_EXT_PAGE_SIZE);
  FOSSASAT_DEBUG_PORT.println(F("Store & forward wipe start"));
  PersistentStorage_64kBlockErase(FLASH_STORE_AND_FORWARD_START);
  PowerControl_Watchdog_Heartbeat();
  PersistentStorage_Set<uint32_t>(FLASH_STORE_AND_FORWARD_LENGTH, 0);
  FOSSASAT_DEBUG_PORT.println(F("Store & forward wipe done"));
  FOSSASAT_DEBUG_PRINT_FLASH(FLASH_STORE_AND_FORWARD_LENGTH, FLASH_EXT_PAGE_SIZE);

  FOSSASAT_DEBUG_PORT.println(F("-------"));
  FOSSASAT_DEBUG_PRINT_FLASH(FLASH_NMEA_LOG_START, FLASH_EXT_PAGE_SIZE);
  FOSSASAT_DEBUG_PORT.println(F("NMEA wipe start"));
  PersistentStorage_64kBlockErase(FLASH_NMEA_LOG_START);
  PowerControl_Watchdog_Heartbeat();
  PersistentStorage_Set<uint32_t>(FLASH_NMEA_LOG_LENGTH, 0);
  FOSSASAT_DEBUG_PORT.println(F("NMEA wipe done"));
  FOSSASAT_DEBUG_PRINT_FLASH(FLASH_NMEA_LOG_START, FLASH_EXT_PAGE_SIZE);

  FOSSASAT_DEBUG_PORT.println(F("-------"));
  FOSSASAT_DEBUG_PRINT_FLASH(FLASH_IMAGE_PROPERTIES, FLASH_EXT_PAGE_SIZE);
  FOSSASAT_DEBUG_PORT.println(F("Image wipe start (will take about 3 minutes)"));
  for(uint8_t i = 0; i < 6; i++) {
    PersistentStorage_SectorErase(FLASH_IMAGE_PROPERTIES + i*FLASH_SECTOR_SIZE);
    PowerControl_Watchdog_Heartbeat();
  }
  for(uint32_t addr = FLASH_IMAGES_START; addr < FLASH_CHIP_SIZE; addr += FLASH_64K_BLOCK_SIZE) {
    PersistentStorage_64kBlockErase(addr);
    PowerControl_Watchdog_Heartbeat();
  }
  FOSSASAT_DEBUG_PORT.println(F("Image wipe done"));
  FOSSASAT_DEBUG_PRINT_FLASH(FLASH_IMAGE_PROPERTIES, FLASH_EXT_PAGE_SIZE);

  FOSSASAT_DEBUG_PORT.println(F("-------"));
  FOSSASAT_DEBUG_PRINT_FLASH(FLASH_ADCS_PARAMETERS, FLASH_EXT_PAGE_SIZE);
  FOSSASAT_DEBUG_PORT.println(F("ADCS config wipe start"));
  PersistentStorage_Reset_ADCS_Params();
  PowerControl_Watchdog_Heartbeat();
  FOSSASAT_DEBUG_PORT.println(F("ADCS config wipe done"));
  FOSSASAT_DEBUG_PRINT_FLASH(FLASH_ADCS_PARAMETERS, FLASH_EXT_PAGE_SIZE);
}

void loop() {
  
}
