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
#define DEPLOYMENT_BATTERY_VOLTAGE_LIMIT                3700    // mV
#define HEATER_BATTERY_VOLTAGE_LIMIT                    3800    // mV
#define BATTERY_CW_BEEP_VOLTAGE_LIMIT                   3800          /*!< Battery voltage limit to switch into morse beep (mV). */
#define LOW_POWER_MODE_VOLTAGE_LIMIT                    3800
#define BATTERY_HEATER_TEMP_LIMIT                       5.0     // deg. C
#define MPPT_TEMP_LIMIT                                 0.0     // deg. C
#define BATTERY_HEATER_DUTY_CYCLE                       255     // PWM duty cycle 0 - 255

#define FLASH_EXT_PAGE_SIZE                             0x00000100
#define FLASH_SECTOR_SIZE                               0x00001000
#define FLASH_64K_BLOCK_SIZE                            0x00010000
#define FLASH_CHIP_SIZE                                 0x04000000
#define FLASH_IMAGE_NUM_64K_BLOCKS                      8

// Flash address map                                                    LSB           MSB
// 64kB block 0 - system info, stats, image lengths
// sector 0 - system info/configuration
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
#define FLASH_LOW_POWER_MODE                            0x00000034  //  0x00000034    0x00000034
#define FLASH_DEPLOYMENT_BATTERY_VOLTAGE_LIMIT          0x00000035  //  0x00000035    0x00000036
#define FLASH_HEATER_BATTERY_VOLTAGE_LIMIT              0x00000037  //  0x00000037    0x00000038
#define FLASH_BATTERY_CW_BEEP_VOLTAGE_LIMIT             0x00000038  //  0x00000039    0x0000003A
#define FLASH_LOW_POWER_MODE_VOLTAGE_LIMIT              0x0000003B  //  0x0000003B    0x0000003C
#define FLASH_BATTERY_HEATER_TEMP_LIMIT                 0x0000003D  //  0x0000003D    0x00000040
#define FLASH_MPPT_TEMP_LIMIT                           0x00000041  //  0x00000041    0x00000044
#define FLASH_BATTERY_HEATER_DUTY_CYCLE                 0x00000045  //  0x00000045    0x00000045
#define FLASH_MPPT_TEMP_SWITCH_ENABLED                  0x00000046  //  0x00000046    0x00000046
#define FLASH_MPPT_KEEP_ALIVE_ENABLED                   0x00000047  //  0x00000047    0x00000047

// sector 1 - stats
// todo stats
#define FLASH_STATS                                     0x00001000  //  0x00001000    0x00001FFF

// sector 2 - image lengths: 4 bytes per length
#define FLASH_IMAGE_LENGTHS                             0x00002000  //  0x00002000    0x00002FFF

// 64kB block 1 - store & forward slots
#define FLASH_STORE_AND_FORWARD_START                   0x00010000  //  0x00010000    0x0001FFFF

// 64kB block 2 - NMEA sentences: null-terminated C-strings, each starts with 4-byte timestamp (offset since recording start)
#define FLASH_NMEA_START                                0x00020000  //  0x00020000    0x0002FFFF

// 64kB blocks 8 - 1023 - image slots: 8 blocks per slot
#define FLASH_IMAGES_START                              0x00070000  //  0x00070000    0x03FFFFFF
#define FLASH_IMAGE_SLOT_SIZE                           (FLASH_IMAGE_NUM_64K_BLOCKS * FLASH_64K_BLOCK_SIZE)

extern SPIClass FlashSPI;
extern uint8_t currentModem;

#endif
