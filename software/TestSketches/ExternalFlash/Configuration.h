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
// sector 0 page 0 - system info and configuration
#define FLASH_SYSTEM_INFO                               0x00000000  //  0x00000000    0x000000FF
#define FLASH_RESTART_COUNTER                           0x00000000  //  0x00000000    0x00000001
#define FLASH_DEPLOYMENT_COUNTER                        0x00000002  //  0x00000002    0x00000002
#define FLASH_TRANSMISSIONS_ENABLED                     0x00000003  //  0x00000003    0x00000003
#define FLASH_CALLSIGN_LEN                              0x00000004  //  0x00000004    0x00000004
#define FLASH_CALLSIGN                                  0x00000005  //  0x00000005    0x00000025
#define FLASH_FSK_RECEIVE_LEN                           0x00000026  //  0x00000026    0x00000026
#define FLASH_LORA_RECEIVE_LEN                          0x00000027  //  0x00000027    0x00000027
#define FLASH_RTC_EPOCH                                 0x00000028  //  0x00000028    0x0000002B
#define FLASH_LORA_VALID_COUNTER                        0x0000002C  //  0x0000002C    0x0000002D
#define FLASH_LORA_INVALID_COUNTER                      0x0000002E  //  0x0000002E    0x0000002F
#define FLASH_FSK_VALID_COUNTER                         0x00000030  //  0x00000030    0x00000031
#define FLASH_FSK_INVALID_COUNTER                       0x00000032  //  0x00000032    0x00000033
#define FLASH_LOW_POWER_MODE_ENABLED                    0x00000034  //  0x00000034    0x00000034
#define FLASH_LOW_POWER_MODE                            0x00000035  //  0x00000035    0x00000035
#define FLASH_DEPLOYMENT_BATTERY_VOLTAGE_LIMIT          0x00000036  //  0x00000036    0x00000037
#define FLASH_HEATER_BATTERY_VOLTAGE_LIMIT              0x00000038  //  0x00000038    0x00000039
#define FLASH_BATTERY_CW_BEEP_VOLTAGE_LIMIT             0x0000003A  //  0x0000003A    0x0000003B
#define FLASH_LOW_POWER_MODE_VOLTAGE_LIMIT              0x0000003C  //  0x0000003C    0x0000003D
#define FLASH_BATTERY_HEATER_TEMP_LIMIT                 0x0000003E  //  0x0000003E    0x00000041
#define FLASH_MPPT_TEMP_LIMIT                           0x00000042  //  0x00000042    0x00000045
#define FLASH_BATTERY_HEATER_DUTY_CYCLE                 0x00000046  //  0x00000046    0x00000046
#define FLASH_MPPT_TEMP_SWITCH_ENABLED                  0x00000047  //  0x00000047    0x00000047
#define FLASH_MPPT_KEEP_ALIVE_ENABLED                   0x00000048  //  0x00000048    0x00000048
#define FLASH_NMEA_LOG_LENGTH                           0x00000049  //  0x00000049    0x0000004C

// sector 1 page 0 - stats
#define FLASH_STATS                                     0x00001000  //  0x00001000    0x000010FF

#define FLASH_STATS_TEMP_PANEL_Y                        0x00001000  //  0x00001000    0x00001005
#define FLASH_STATS_TEMP_TOP                            0x00001006  //  0x00001006    0x0000100B
#define FLASH_STATS_TEMP_BOTTOM                         0x0000100C  //  0x0000100C    0x00001011
#define FLASH_STATS_TEMP_BATTERY                        0x00001012  //  0x00001012    0x00001017
#define FLASH_STATS_TEMP_SEC_BATTERY                    0x00001018  //  0x00001018    0x0000101D

#define FLASH_STATS_CURR_XA                             0x0000101E  //  0x0000101E    0x00001023
#define FLASH_STATS_CURR_XB                             0x00001024  //  0x00001024    0x00001029
#define FLASH_STATS_CURR_ZA                             0x0000102A  //  0x0000102A    0x0000102F
#define FLASH_STATS_CURR_ZB                             0x00001030  //  0x00001030    0x00001035
#define FLASH_STATS_CURR_Y                              0x00001036  //  0x00001036    0x0000103B
#define FLASH_STATS_CURR_MPPT                           0x0000103C  //  0x0000103C    0x00001041

#define FLASH_STATS_VOLT_XA                             0x00001042  //  0x00001042    0x00001044
#define FLASH_STATS_VOLT_XB                             0x00001045  //  0x00001045    0x00001047
#define FLASH_STATS_VOLT_ZA                             0x00001048  //  0x00001048    0x0000104A
#define FLASH_STATS_VOLT_ZB                             0x0000104B  //  0x0000104B    0x0000104D
#define FLASH_STATS_VOLT_Y                              0x0000104E  //  0x0000104E    0x00001050
#define FLASH_STATS_VOLT_MPPT                           0x00001051  //  0x00001051    0x00001053

#define FLASH_STATS_LIGHT_PANEL_Y                       0x00001054  //  0x00001054    0x0000105F
#define FLASH_STATS_LIGHT_TOP                           0x00001060  //  0x00001060    0x0000106B

#define FLASH_STATS_GYRO_X                              0x0000106C  //  0x0000106C    0x00001077
#define FLASH_STATS_GYRO_Y                              0x00001078  //  0x00001078    0x00001083
#define FLASH_STATS_GYRO_Z                              0x00001084  //  0x00001084    0x0000108F
#define FLASH_STATS_ACCEL_X                             0x00001090  //  0x00001090    0x0000109B
#define FLASH_STATS_ACCEL_Y                             0x0000109C  //  0x0000109C    0x000010A7
#define FLASH_STATS_ACCEL_Z                             0x000010A8  //  0x000010A8    0x000010B3
#define FLASH_STATS_MAG_X                               0x000010B4  //  0x000010B4    0x000010BF
#define FLASH_STATS_MAG_Y                               0x000010C0  //  0x000010C0    0x000010CB
#define FLASH_STATS_MAG_Z                               0x000010CC  //  0x000010CC    0x000010D7

// sectors 2 + 3 - image lengths: 4 bytes per length
#define FLASH_IMAGE_LENGTHS_1                           0x00002000  //  0x00002000    0x00002FFF
#define FLASH_IMAGE_LENGTHS_2                           0x00003000  //  0x00003000    0x00003FFF

// 64kB block 1 - store & forward slots
#define FLASH_STORE_AND_FORWARD_START                   0x00010000  //  0x00010000    0x0001FFFF

// 64kB blocks 2 - 7 - NMEA sentences: null-terminated C-strings, each starts with 4-byte timestamp (offset since recording start)
#define FLASH_NMEA_LOG_START                            0x00020000  //  0x00020000    0x0007FFFF

// 64kB blocks 8 - 1023 - image slots: 8 blocks per slot
#define FLASH_IMAGES_START                              0x00080000  //  0x00080000    0x03FFFFFF
#define FLASH_IMAGE_SLOT_SIZE                           (FLASH_IMAGE_NUM_64K_BLOCKS * FLASH_64K_BLOCK_SIZE)

extern SPIClass FlashSPI;
extern uint8_t currentModem;

#endif
