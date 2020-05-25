#include "Configuration.h"

uint8_t systemInfoBuffer[FLASH_EXT_PAGE_SIZE];
STM32RTC& rtc = STM32RTC::getInstance();
SPIClass FlashSPI(FLASH_MOSI, FLASH_MISO, FLASH_SCK);
uint8_t currentModem = MODEM_FSK;
