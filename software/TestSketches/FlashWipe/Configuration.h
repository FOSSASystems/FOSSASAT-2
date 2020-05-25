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

// other
#define WATCHDOG_IN                                     PC13

#define RTC_YEAR                                        20    // offset from 2000
#define RTC_MONTH                                       7
#define RTC_DAY                                         27
#define RTC_WEEKDAY                                     1     // Monday = 1
#define RTC_HOURS                                       13    // 24-hour format, no leading 0s
#define RTC_MINUTES                                     0
#define RTC_SECONDS                                     0

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

#define TLE_LINE_1                                      "1 00000U 20000A   20182.79906828 0.00000000  00000-0 -12345-3 0    10"
#define TLE_LINE_2                                      "2 00000 137.0503 217.9687 0010435 173.7291 322.4297 15.88896416000000"

#define DEFAULT_NUMBER_OF_SLEEP_INTERVALS               6
#define DEFAULT_SLEEP_INTERVAL_VOLTAGES                 { 4050, 4000, 3900, 3800, 3700,    0 }
#define DEFAULT_SLEEP_INTERVAL_LENGTHS                  {   20,   35,  100,  160,  180,  240 } 

#define FLASH_EXT_PAGE_SIZE                             0x00000100
#define FLASH_SECTOR_SIZE                               0x00001000
#define FLASH_64K_BLOCK_SIZE                            0x00010000
#define FLASH_CHIP_SIZE                                 0x04000000
#define FLASH_IMAGE_NUM_64K_BLOCKS                      8

// Flash address map                                                    LSB           MSB           type
// 64kB block 0 - system info, stats, image lengths
// sector 0 page 0 - system info and configuration
#define FLASH_SYSTEM_INFO                               0x00000000  //  0x00000000    0x000000FF
#define FLASH_RESTART_COUNTER                           0x00000000  //  0x00000000    0x00000001    uint16_t
#define FLASH_DEPLOYMENT_COUNTER                        0x00000002  //  0x00000002    0x00000002    uint8_t
#define FLASH_TRANSMISSIONS_ENABLED                     0x00000003  //  0x00000003    0x00000003    uint8_t
#define FLASH_CALLSIGN_LEN                              0x00000004  //  0x00000004    0x00000004    uint8_t
#define FLASH_CALLSIGN                                  0x00000005  //  0x00000005    0x00000025    char[]
#define FLASH_FSK_RECEIVE_LEN                           0x00000026  //  0x00000026    0x00000026    uint8_t
#define FLASH_LORA_RECEIVE_LEN                          0x00000027  //  0x00000027    0x00000027    uint8_t
#define FLASH_RTC_EPOCH                                 0x00000028  //  0x00000028    0x0000002B    uint32_t
#define FLASH_LORA_VALID_COUNTER                        0x0000002C  //  0x0000002C    0x0000002D    uint16_t
#define FLASH_LORA_INVALID_COUNTER                      0x0000002E  //  0x0000002E    0x0000002F    uint16_t
#define FLASH_FSK_VALID_COUNTER                         0x00000030  //  0x00000030    0x00000031    uint16_t
#define FLASH_FSK_INVALID_COUNTER                       0x00000032  //  0x00000032    0x00000033    uint16_t
#define FLASH_LOW_POWER_MODE_ENABLED                    0x00000034  //  0x00000034    0x00000034    uint8_t
#define FLASH_LOW_POWER_MODE                            0x00000035  //  0x00000035    0x00000035    uint8_t
#define FLASH_DEPLOYMENT_BATTERY_VOLTAGE_LIMIT          0x00000036  //  0x00000036    0x00000037    int16_t
#define FLASH_HEATER_BATTERY_VOLTAGE_LIMIT              0x00000038  //  0x00000038    0x00000039    int16_t
#define FLASH_BATTERY_CW_BEEP_VOLTAGE_LIMIT             0x0000003A  //  0x0000003A    0x0000003B    int16_t
#define FLASH_LOW_POWER_MODE_VOLTAGE_LIMIT              0x0000003C  //  0x0000003C    0x0000003D    int16_t
#define FLASH_BATTERY_HEATER_TEMP_LIMIT                 0x0000003E  //  0x0000003E    0x00000041    float
#define FLASH_MPPT_TEMP_LIMIT                           0x00000042  //  0x00000042    0x00000045    float
#define FLASH_BATTERY_HEATER_DUTY_CYCLE                 0x00000046  //  0x00000046    0x00000046    uint8_t
#define FLASH_MPPT_TEMP_SWITCH_ENABLED                  0x00000047  //  0x00000047    0x00000047    uint8_t
#define FLASH_MPPT_KEEP_ALIVE_ENABLED                   0x00000048  //  0x00000048    0x00000048    uint8_t
#define FLASH_NMEA_LOG_LENGTH                           0x00000049  //  0x00000049    0x0000004C    uint32_t
#define FLASH_STORE_AND_FORWARD_LENGTH                  0x0000004D  //  0x0000004D    0x0000004E    int16_t
#define FLASH_AUTO_STATISTICS                           0x0000004F  //  0x0000004F    0x0000004F    uint8_t
#define FLASH_TLE_EPOCH_DAY                             0x00000050  //  0x00000050    0x00000057    double
#define FLASH_TLE_BALLISTIC_COEFF                       0x00000058  //  0x00000058    0x0000005F    double
#define FLASH_TLE_MEAN_MOTION_2ND                       0x00000060  //  0x00000060    0x00000067    double
#define FLASH_TLE_DRAG_TERM                             0x00000068  //  0x00000068    0x0000006F    double
#define FLASH_TLE_INCLINATION                           0x00000070  //  0x00000070    0x00000077    double
#define FLASH_TLE_RIGHT_ASCENTION                       0x00000078  //  0x00000078    0x0000007F    double
#define FLASH_TLE_ECCENTRICITY                          0x00000080  //  0x00000080    0x00000087    double
#define FLASH_TLE_PERIGEE_ARGUMENT                      0x00000088  //  0x00000088    0x0000008F    double
#define FLASH_TLE_MEAN_ANOMALY                          0x00000090  //  0x00000090    0x00000097    double
#define FLASH_TLE_MEAN_MOTION                           0x00000098  //  0x00000098    0x0000009F    double
#define FLASH_TLE_REVOLUTION_NUMBER                     0x000000A0  //  0x000000A0    0x000000A3    uint32_t
#define FLASH_TLE_EPOCH_YEAR                            0x000000A4  //  0x000000A4    0x000000A4    uint8_t
#define FLASH_NMEA_LOG_LATEST_ENTRY                     0x000000A5  //  0x000000A5    0x000000A8    uint32_t
#define FLASH_NMEA_LOG_LATEST_FIX                       0x000000A9  //  0x000000A9    0x000000AC    uint32_t
#define FLASH_LOOP_COUNTER                              0x000000AD  //  0x000000AD    0x000000AD    uint8_t
#define FLASH_NUM_SLEEP_INTERVALS                       0x000000AE  //  0x000000AE    0x000000AE    uint8_t
#define FLASH_SLEEP_INTERVALS                           0x000000B0  //  0x000000B0    0x000000BF    FLASH_NUM_SLEEP_INTERVALS x (int16_t + uint16_t)
#define FLASH_SYSTEM_INFO_CRC                           0x000000F8  //  0x000000F8    0x000000FB    uint32_t
#define FLASH_MEMORY_ERROR_COUNTER                      0x000000FC  //  0x000000FC    0x000000FF    uint32_t

// sector 1 page 0 - stats
#define FLASH_STATS                                     0x00001000  //  0x00001000    0x000010FF

#define FLASH_STATS_TEMP_PANEL_Y                        0x00001000  //  0x00001000    0x00001005    3x int16_t
#define FLASH_STATS_TEMP_TOP                            0x00001006  //  0x00001006    0x0000100B    3x int16_t
#define FLASH_STATS_TEMP_BOTTOM                         0x0000100C  //  0x0000100C    0x00001011    3x int16_t
#define FLASH_STATS_TEMP_BATTERY                        0x00001012  //  0x00001012    0x00001017    3x int16_t
#define FLASH_STATS_TEMP_SEC_BATTERY                    0x00001018  //  0x00001018    0x0000101D    3x int16_t
#define FLASH_STATS_TEMP_MCU                            0x0000101E  //  0x0000101E    0x00001023    3x int16_t

#define FLASH_STATS_CURR_XA                             0x00001024  //  0x00001024    0x00001029    3x int16_t
#define FLASH_STATS_CURR_XB                             0x0000102A  //  0x0000102A    0x0000102F    3x int16_t
#define FLASH_STATS_CURR_ZA                             0x00001030  //  0x00001030    0x00001035    3x int16_t
#define FLASH_STATS_CURR_ZB                             0x00001036  //  0x00001036    0x0000103B    3x int16_t
#define FLASH_STATS_CURR_Y                              0x0000103C  //  0x0000103C    0x00001041    3x int16_t
#define FLASH_STATS_CURR_MPPT                           0x00001042  //  0x00001042    0x00001047    3x int16_t

#define FLASH_STATS_VOLT_XA                             0x00001048  //  0x00001048    0x0000104A    3x uint8_t
#define FLASH_STATS_VOLT_XB                             0x0000104B  //  0x0000104B    0x0000104D    3x uint8_t
#define FLASH_STATS_VOLT_ZA                             0x0000104E  //  0x0000104E    0x00001050    3x uint8_t
#define FLASH_STATS_VOLT_ZB                             0x00001051  //  0x00001051    0x00001053    3x uint8_t
#define FLASH_STATS_VOLT_Y                              0x00001054  //  0x00001054    0x00001056    3x uint8_t
#define FLASH_STATS_VOLT_MPPT                           0x00001057  //  0x00001057    0x00001059    3x uint8_t

#define FLASH_STATS_LIGHT_PANEL_Y                       0x0000105A  //  0x0000105A    0x0000105F    3x float
#define FLASH_STATS_LIGHT_TOP                           0x00001066  //  0x00001066    0x00001071    3x float

#define FLASH_STATS_GYRO_X                              0x00001072  //  0x00001072    0x0000107D    3x float
#define FLASH_STATS_GYRO_Y                              0x0000107E  //  0x0000107E    0x00001089    3x float
#define FLASH_STATS_GYRO_Z                              0x0000108A  //  0x0000108A    0x00001095    3x float
#define FLASH_STATS_ACCEL_X                             0x00001096  //  0x00001096    0x000010A1    3x float
#define FLASH_STATS_ACCEL_Y                             0x000010A2  //  0x000010A2    0x000010AD    3x float
#define FLASH_STATS_ACCEL_Z                             0x000010AE  //  0x000010AE    0x000010B9    3x float
#define FLASH_STATS_MAG_X                               0x000010BA  //  0x000010BA    0x000010C5    3x float
#define FLASH_STATS_MAG_Y                               0x000010C6  //  0x000010C6    0x000010D1    3x float
#define FLASH_STATS_MAG_Z                               0x000010D2  //  0x000010D2    0x000010DD    3x float

// sectors 2 + 3 - image lengths: 4 bytes per length
#define FLASH_IMAGE_LENGTHS_1                           0x00002000  //  0x00002000    0x00002FFF
#define FLASH_IMAGE_LENGTHS_2                           0x00003000  //  0x00003000    0x00003FFF

// 64kB block 1 - store & forward slots
#define FLASH_STORE_AND_FORWARD_START                   0x00010000  //  0x00010000    0x0001FFFF
#define FLASH_STORE_AND_FORWARD_NUM_SLOTS               (FLASH_64K_BLOCK_SIZE / MAX_STRING_LENGTH)

// 64kB blocks 2 - 31 - NMEA sentences: null-terminated C-strings, each starts with 4-byte timestamp (offset since recording start)
#define FLASH_NMEA_LOG_START                            0x00020000  //  0x00030000    0x001FFFFF
#define FLASH_NMEA_LOG_END                              (FLASH_IMAGES_START)
#define FLASH_NMEA_LOG_SLOT_SIZE                        (MAX_IMAGE_PACKET_LENGTH)

// 64kB blocks 32 - 1023 - image slots: 8 blocks per slot
#define FLASH_IMAGES_START                              0x00200000  //  0x00200000    0x03FFFFFF
#define FLASH_IMAGE_SLOT_SIZE                           (FLASH_IMAGE_NUM_64K_BLOCKS * FLASH_64K_BLOCK_SIZE)

extern uint8_t systemInfoBuffer[];
extern STM32RTC& rtc;
extern SPIClass FlashSPI;
extern uint8_t currentModem;

#endif
