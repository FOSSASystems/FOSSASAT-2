#ifndef _FOSSASAT_CONFIGURATION_H
#define _FOSSASAT_CONFIGURATION_H

#include "FossaSat2.h"

// string length limit
#define MAX_STRING_LENGTH                               32

// SPI
#define FLASH_MOSI                                      PA7
#define FLASH_MISO                                      PA6
#define FLASH_SCK                                       PA5
#define FLASH_CS                                        PC4
#define FLASH_RESET                                     PC5

#define CALLSIGN_DEFAULT                                "FOSSASAT-2"
#define MODEM_FSK                                       'F'
#define MODEM_LORA                                      'L'
#define LORA_RECEIVE_WINDOW_LENGTH                      40
#define FSK_RECEIVE_WINDOW_LENGTH                       20

#define FLASH_SECTOR_SIZE                               0x00000100
#define FLASH_64K_BLOCK_SIZE                            0x00010000
#define FLASH_IMAGE_NUM_64K_BLOCKS                      8

// Flash address map                                                    LSB           MSB
// 64kB block 0 - system info
#define FLASH_RESTART_COUNTER                           0x00000000  //  0x00000000    0x00000001
#define FLASH_DEPLOYMENT_COUNTER                        0x00000002  //  0x00000002    0x00000002
#define FLASH_TRANSMISSIONS_ENABLED                     0x00000003  //  0x00000003    0x00000003
#define FLASH_CALLSIGN_LEN                              0x00000004  //  0x00000004    0x00000004
#define FLASH_CALLSIGN                                  0x00000005  //  0x00000005    0x00000025
#define FLASH_FSK_RECEIVE_LEN                           0x00000026  //  0x00000026    0x00000026
#define FLASH_LORA_RECEIVE_LEN                          0x00000027  //  0x00000027    0x00000027
#define FLASH_UPTIME_COUNTER                            0x00000028  //  0x00000028    0x0000002A
#define FLASH_LORA_VALID_COUNTER                        0x0000002B  //  0x0000002B    0x0000002C
#define FLASH_LORA_INVALID_COUNTER                      0x0000002D  //  0x0000002D    0x0000002E
#define FLASH_FSK_VALID_COUNTER                         0x0000002F  //  0x0000002F    0x00000030
#define FLASH_FSK_INVALID_COUNTER                       0x00000031  //  0x00000031    0x00000032
#define FLASH_LOW_POWER_MODE_ENABLED                    0x00000033  //  0x00000033    0x00000033
#define FLASH_LOW_POWER_MODE_ACTIVE                     0x00000034  //  0x00000034    0x00000034
#define FLASH_IMAGE1_LENGTH                             0x00000035  //  0x00000035    0x00000038
// todo stats

// 64kB block 1-8 - image 1 (assuming 512 kB JPEG as the maximum size)
#define FLASH_IMAGE1                                    0x00020000  //  0x00010000    0x0007FFFF

extern SPIClass FlashSPI;
extern uint8_t currentModem;

#endif
