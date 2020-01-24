#ifndef _FOSSASAT_CONFIGURATION_H
#define _FOSSASAT_CONFIGURATION_H

#include "FossaSat2.h"

// string length limit
#define MAX_STRING_LENGTH                               32

// SPI
#define SPI1_SCK                                        PA5
#define SPI1_MISO                                       PA6
#define SPI1_MOSI                                       PA7

// external flash
#define FLASH_CS                                        PC11
#define FLASH_RESET                                     PC12

#define CALLSIGN_DEFAULT                                "FOSSASAT-2"

// Flash address map                                                   LSB           MSB
// sector 0 - system info
#define FLASH_RESTART_COUNTER_ADDR                      0x00000000  // 0x00000000    0x00000001
#define FLASH_DEPLOYMENT_COUNTER_ADDR                   0x00000002  // 0x00000002    0x00000002
#define FLASH_TRANSMISSIONS_ENABLED                     0x00000003  // 0x00000003    0x00000003
#define FLASH_CALLSIGN_LEN_ADDR                         0x00000004  // 0x00000004    0x00000004
#define FLASH_CALLSIGN_ADDR                             0x00000005  // 0x00000005    0x00000025

// sector 1-X - images
#define FLASH_IMAGE_CAPTURE_LENGTH                      0x00001000  // 0x00001000    0x00001003
#define FLASH_IMAGE_CAPTURE                             0x00001004  // 0x00001004    0x00321004  assuming 400 kB JPEG as the maximum size

#endif
