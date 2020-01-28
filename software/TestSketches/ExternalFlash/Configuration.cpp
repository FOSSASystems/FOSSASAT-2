#include "Configuration.h"

SPIClass FlashSPI(FLASH_MOSI, FLASH_MISO, FLASH_SCK);

uint8_t currentModem = MODEM_FSK;
