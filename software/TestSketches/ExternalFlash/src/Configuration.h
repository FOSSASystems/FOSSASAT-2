#ifndef _FOSSASAT_CONFIGURATION_H
#define _FOSSASAT_CONFIGURATION_H

#include "FossaSat2.h"

/*
    Configuration Macros
*/

// expected deployment time and date
#define RTC_YEAR                                        20    // offset from 2000
#define RTC_MONTH                                       7
#define RTC_DAY                                         27
#define RTC_WEEKDAY                                     1     // Monday = 1
#define RTC_HOURS                                       13    // 24-hour format, no leading 0s
#define RTC_MINUTES                                     0
#define RTC_SECONDS                                     0

// uncomment to reset system info (callsign, configuration etc.) on start
//#define RESET_SYSTEM_INFO

// comment out to disable deployment sequence
//#define ENABLE_DEPLOYMENT_SEQUENCE

// comment out to disable sleep during deployment sequence
//#define ENABLE_DEPLOYMENT_SLEEP

// comment out to disable charging period during deployment sequence
//#define ENABLE_DEPLOYMENT_CHARGING

// comment out to disable transmission control (transmission disable and no transmissions in low power mode)
//#define ENABLE_TRANSMISSION_CONTROL

/*
    Array Length Limits
*/

// string length limit
#define MAX_STRING_LENGTH                               32

// maximum number of image data bytes in packet
#define MAX_IMAGE_PAYLOAD_LENGTH                        128

// number of FEC bytes in picture downlink packet
#define IMAGE_PACKET_FEC_LENGTH                         32

// RS code full packet length
#define RS8_FULL_PACKET_LENGTH                          256

// optional data length limit
#define MAX_OPT_DATA_LENGTH                             220

// radio buffer length limit
#define MAX_RADIO_BUFFER_LENGTH                         (MAX_STRING_LENGTH + 2 + MAX_OPT_DATA_LENGTH)

/*
    Pin Mapping
*/

// I2C
#define I2C1_SDA                                        PA10
#define I2C1_SCL                                        PA9
#define I2C2_SDA                                        PB11
#define I2C2_SCL                                        PB10

// radio
#define RADIO_MOSI                                      PB15
#define RADIO_MISO                                      PB14
#define RADIO_SCK                                       PB13
#define RADIO_NSS                                       PC7
#define RADIO_DIO1                                      PC9
#define RADIO_NRST                                      PC8
#define RADIO_BUSY                                      PC6

// external flash
#define FLASH_MOSI                                      PA7
#define FLASH_MISO                                      PA6
#define FLASH_SCK                                       PA5
#define FLASH_CS                                        PC4
#define FLASH_RESET                                     PC5

// camera
#define CAMERA_MOSI                                     PC12
#define CAMERA_MISO                                     PC11
#define CAMERA_SCK                                      PC10
#define CAMERA_POWER_FET                                PB5
#define CAMERA_CS                                       PB6

// GPS
#define GPS_TX                                          PA0
#define GPS_RX                                          PA1
#define GPS_POWER_FET                                   PC3

// control FETs
#define DEPLOYMENT_FET_1                                PB1
#define DEPLOYMENT_FET_2                                PB2
#define BATTERY_HEATER_FET                              PA8

// other
#define WATCHDOG_IN                                     PC13
#define MPPT_OFF                                        PA11
#define ANALOG_IN_RANDOM_SEED                           PC2    // used as source for randomSeed(), should be left floating

#define POWER_FET_POLARITY_ON                           LOW
#define POWER_FET_POLARITY_OFF                          HIGH


/*
   Timing Definitions
*/

#define DEPLOYMENT_DEBUG_LENGTH                         60      // s
#define DEPLOYMENT_DEBUG_SAMPLE_PERIOD                  1000    // ms
#define DEPLOYMENT_SLEEP_LENGTH                         1800000 // ms
#define DEPLOYMENT_CHARGE_LIMIT                         3       // h
#define DEPLOYMENT_PULSE_LENGTH                         1200    // ms
#define WATCHDOG_LOOP_HEARTBEAT_PERIOD                  1000    // ms

/*
   Voltage Limits
*/

#define DEPLOYMENT_BATTERY_VOLTAGE_LIMIT                3700    // mV
#define HEATER_BATTERY_VOLTAGE_LIMIT                    3800    // mV
#define BATTERY_CW_BEEP_VOLTAGE_LIMIT                   3800          /*!< Battery voltage limit to switch into morse beep (mV). */
#define LOW_POWER_MODE_VOLTAGE_LIMIT                    3800

/*
   Temperature Limits
*/

#define BATTERY_HEATER_TEMP_LIMIT                       5.0     // deg. C
#define MPPT_TEMP_LIMIT                                -0.7     // deg. C
#define BATTERY_HEATER_DUTY_CYCLE                       255     // PWM duty cycle 0 - 255

/*
   Default sleep intervals
*/
#define DEFAULT_NUMBER_OF_SLEEP_INTERVALS               6       // maximum of 8
#define DEFAULT_SLEEP_INTERVAL_VOLTAGES                 { 4050, 4000, 3900, 3800, 3700,    0 }    // mV
#define DEFAULT_SLEEP_INTERVAL_LENGTHS                  {   20,   35,  100,  160,  180,  240 }    // sec

/*
   Default TLE
*/
#define TLE_LINE_1                                      "1 00000U 20000A   20182.79906828 0.00000000  00000-0 -12345-3 0    10"
#define TLE_LINE_2                                      "2 00000 137.0503 217.9687 0010435 173.7291 322.4297 15.88896416000000"

/*
   Default IMU static offsets
*/
#define IMU_OFFSET_GYRO_X                               0.0   // rad/s
#define IMU_OFFSET_GYRO_Y                               0.0   // rad/s
#define IMU_OFFSET_GYRO_Z                               0.0   // rad/s
#define IMU_OFFSET_ACCEL_X                              0.0   // m/s^2
#define IMU_OFFSET_ACCEL_Y                              0.0   // m/s^2
#define IMU_OFFSET_ACCEL_Z                              0.0   // m/s^2
#define IMU_OFFSET_MAG_X                                0.0   // Tesla
#define IMU_OFFSET_MAG_Y                                0.0   // Tesla
#define IMU_OFFSET_MAG_Z                                0.0   // Tesla

/*
   Flash Configuration
*/

#define FLASH_EXT_PAGE_SIZE                             0x00000100
#define FLASH_SECTOR_SIZE                               0x00001000
#define FLASH_64K_BLOCK_SIZE                            0x00010000
#define FLASH_CHIP_SIZE                                 0x04000000
#define FLASH_IMAGE_NUM_64K_BLOCKS                      8

// Flash address map                                                              LSB           MSB           type
// 64kB block 0 - system info, stats, image lengths
// sector 0 page 0 - system info and configuration
#define FLASH_SYSTEM_INFO                                         0x00000000  //  0x00000000    0x000000FF

#define FLASH_RESTART_COUNTER                     (FLASH_SYSTEM_INFO + 0x00)  //  0x00000000    0x00000001    uint16_t
#define FLASH_DEPLOYMENT_COUNTER                  (FLASH_SYSTEM_INFO + 0x02)  //  0x00000002    0x00000002    uint8_t
#define FLASH_TRANSMISSIONS_ENABLED               (FLASH_SYSTEM_INFO + 0x03)  //  0x00000003    0x00000003    uint8_t
#define FLASH_CALLSIGN_LEN                        (FLASH_SYSTEM_INFO + 0x04)  //  0x00000004    0x00000004    uint8_t
#define FLASH_CALLSIGN                            (FLASH_SYSTEM_INFO + 0x05)  //  0x00000005    0x00000025    char[]
#define FLASH_FSK_RECEIVE_LEN                     (FLASH_SYSTEM_INFO + 0x26)  //  0x00000026    0x00000026    uint8_t
#define FLASH_LORA_RECEIVE_LEN                    (FLASH_SYSTEM_INFO + 0x27)  //  0x00000027    0x00000027    uint8_t
#define FLASH_RTC_EPOCH                           (FLASH_SYSTEM_INFO + 0x28)  //  0x00000028    0x0000002B    uint32_t
#define FLASH_LORA_VALID_COUNTER                  (FLASH_SYSTEM_INFO + 0x2C)  //  0x0000002C    0x0000002D    uint16_t
#define FLASH_LORA_INVALID_COUNTER                (FLASH_SYSTEM_INFO + 0x2E)  //  0x0000002E    0x0000002F    uint16_t
#define FLASH_FSK_VALID_COUNTER                   (FLASH_SYSTEM_INFO + 0x30)  //  0x00000030    0x00000031    uint16_t
#define FLASH_FSK_INVALID_COUNTER                 (FLASH_SYSTEM_INFO + 0x32)  //  0x00000032    0x00000033    uint16_t
#define FLASH_LOW_POWER_MODE_ENABLED              (FLASH_SYSTEM_INFO + 0x34)  //  0x00000034    0x00000034    uint8_t
#define FLASH_LOW_POWER_MODE                      (FLASH_SYSTEM_INFO + 0x35)  //  0x00000035    0x00000035    uint8_t
#define FLASH_DEPLOYMENT_BATTERY_VOLTAGE_LIMIT    (FLASH_SYSTEM_INFO + 0x36)  //  0x00000036    0x00000037    int16_t
#define FLASH_HEATER_BATTERY_VOLTAGE_LIMIT        (FLASH_SYSTEM_INFO + 0x38)  //  0x00000038    0x00000039    int16_t
#define FLASH_BATTERY_CW_BEEP_VOLTAGE_LIMIT       (FLASH_SYSTEM_INFO + 0x3A)  //  0x0000003A    0x0000003B    int16_t
#define FLASH_LOW_POWER_MODE_VOLTAGE_LIMIT        (FLASH_SYSTEM_INFO + 0x3C)  //  0x0000003C    0x0000003D    int16_t
#define FLASH_BATTERY_HEATER_TEMP_LIMIT           (FLASH_SYSTEM_INFO + 0x3E)  //  0x0000003E    0x00000041    float
#define FLASH_MPPT_TEMP_LIMIT                     (FLASH_SYSTEM_INFO + 0x42)  //  0x00000042    0x00000045    float
#define FLASH_BATTERY_HEATER_DUTY_CYCLE           (FLASH_SYSTEM_INFO + 0x46)  //  0x00000046    0x00000046    uint8_t
#define FLASH_MPPT_TEMP_SWITCH_ENABLED            (FLASH_SYSTEM_INFO + 0x47)  //  0x00000047    0x00000047    uint8_t
#define FLASH_MPPT_KEEP_ALIVE_ENABLED             (FLASH_SYSTEM_INFO + 0x48)  //  0x00000048    0x00000048    uint8_t
#define FLASH_NMEA_LOG_LENGTH                     (FLASH_SYSTEM_INFO + 0x49)  //  0x00000049    0x0000004C    uint32_t
#define FLASH_STORE_AND_FORWARD_LENGTH            (FLASH_SYSTEM_INFO + 0x4D)  //  0x0000004D    0x0000004E    int16_t
#define FLASH_AUTO_STATISTICS                     (FLASH_SYSTEM_INFO + 0x4F)  //  0x0000004F    0x0000004F    uint8_t
#define FLASH_TLE_EPOCH_DAY                       (FLASH_SYSTEM_INFO + 0x50)  //  0x00000050    0x00000057    double
#define FLASH_TLE_BALLISTIC_COEFF                 (FLASH_SYSTEM_INFO + 0x58)  //  0x00000058    0x0000005F    double
#define FLASH_TLE_MEAN_MOTION_2ND                 (FLASH_SYSTEM_INFO + 0x60)  //  0x00000060    0x00000067    double
#define FLASH_TLE_DRAG_TERM                       (FLASH_SYSTEM_INFO + 0x68)  //  0x00000068    0x0000006F    double
#define FLASH_TLE_INCLINATION                     (FLASH_SYSTEM_INFO + 0x70)  //  0x00000070    0x00000077    double
#define FLASH_TLE_RIGHT_ASCENTION                 (FLASH_SYSTEM_INFO + 0x78)  //  0x00000078    0x0000007F    double
#define FLASH_TLE_ECCENTRICITY                    (FLASH_SYSTEM_INFO + 0x80)  //  0x00000080    0x00000087    double
#define FLASH_TLE_PERIGEE_ARGUMENT                (FLASH_SYSTEM_INFO + 0x88)  //  0x00000088    0x0000008F    double
#define FLASH_TLE_MEAN_ANOMALY                    (FLASH_SYSTEM_INFO + 0x90)  //  0x00000090    0x00000097    double
#define FLASH_TLE_MEAN_MOTION                     (FLASH_SYSTEM_INFO + 0x98)  //  0x00000098    0x0000009F    double
#define FLASH_TLE_REVOLUTION_NUMBER               (FLASH_SYSTEM_INFO + 0xA0)  //  0x000000A0    0x000000A3    uint32_t
#define FLASH_TLE_EPOCH_YEAR                      (FLASH_SYSTEM_INFO + 0xA4)  //  0x000000A4    0x000000A4    uint8_t
#define FLASH_NMEA_LOG_LATEST_ENTRY               (FLASH_SYSTEM_INFO + 0xA5)  //  0x000000A5    0x000000A8    uint32_t
#define FLASH_NMEA_LOG_LATEST_FIX                 (FLASH_SYSTEM_INFO + 0xA9)  //  0x000000A9    0x000000AC    uint32_t
#define FLASH_LOOP_COUNTER                        (FLASH_SYSTEM_INFO + 0xAD)  //  0x000000AD    0x000000AD    uint8_t
#define FLASH_LAST_ADCS_RESULT                    (FLASH_SYSTEM_INFO + 0xAE)  //  0x000000AE    0x000000AE    uint8_t
#define FLASH_NUM_SLEEP_INTERVALS                 (FLASH_SYSTEM_INFO + 0xAF)  //  0x000000AF    0x000000AF    uint8_t
#define FLASH_SLEEP_INTERVALS                     (FLASH_SYSTEM_INFO + 0xB0)  //  0x000000B0    0x000000CF    FLASH_NUM_SLEEP_INTERVALS x (int16_t + uint16_t)
#define FLASH_IMU_OFFSET_GYRO_X                   (FLASH_SYSTEM_INFO + 0xD0)  //  0x000000D0    0x000000D3    float
#define FLASH_IMU_OFFSET_GYRO_Y                   (FLASH_SYSTEM_INFO + 0xD4)  //  0x000000D4    0x000000D7    float
#define FLASH_IMU_OFFSET_GYRO_Z                   (FLASH_SYSTEM_INFO + 0xD8)  //  0x000000D8    0x000000DB    float
#define FLASH_IMU_OFFSET_ACCEL_X                  (FLASH_SYSTEM_INFO + 0xDC)  //  0x000000DC    0x000000DF    float
#define FLASH_IMU_OFFSET_ACCEL_Y                  (FLASH_SYSTEM_INFO + 0xE0)  //  0x000000E0    0x000000E3    float
#define FLASH_IMU_OFFSET_ACCEL_Z                  (FLASH_SYSTEM_INFO + 0xE4)  //  0x000000E4    0x000000E7    float
#define FLASH_IMU_OFFSET_MAG_X                    (FLASH_SYSTEM_INFO + 0xE8)  //  0x000000E8    0x000000EB    float
#define FLASH_IMU_OFFSET_MAG_Y                    (FLASH_SYSTEM_INFO + 0xEC)  //  0x000000EC    0x000000EF    float
#define FLASH_IMU_OFFSET_MAG_Z                    (FLASH_SYSTEM_INFO + 0xF0)  //  0x000000F0    0x000000F3    float
#define FLASH_FSK_ONLY_ENABLED                    (FLASH_SYSTEM_INFO + 0xF1)  //  0x000000F1    0x000000F1    uint8_t
#define FLASH_SYSTEM_INFO_CRC                     (FLASH_SYSTEM_INFO + 0xF8)  //  0x000000F8    0x000000FB    uint32_t
#define FLASH_MEMORY_ERROR_COUNTER                (FLASH_SYSTEM_INFO + 0xFC)  //  0x000000FC    0x000000FF    uint32_t

// sector 1 - ADCS parameters
#define FLASH_ADCS_PARAMETERS                                     0x00001000  //  0x00001000    0x000010FF

#define FLASH_ADCS_PULSE_MAX_INTENSITY        (FLASH_ADCS_PARAMETERS + 0x00)  //  0x00001000    0x00001007    float or double
#define FLASH_ADCS_PULSE_MAX_LENGTH           (FLASH_ADCS_PARAMETERS + 0x08)  //  0x00001008    0x0000100F    float or double
#define FLASH_ADCS_DETUMB_OMEGA_TOLERANCE     (FLASH_ADCS_PARAMETERS + 0x10)  //  0x00001010    0x00001017    float or double
#define FLASH_ADCS_MIN_INERTIAL_MOMENT        (FLASH_ADCS_PARAMETERS + 0x18)  //  0x00001018    0x0000101F    float or double
#define FLASH_ADCS_PULSE_AMPLITUDE            (FLASH_ADCS_PARAMETERS + 0x20)  //  0x00001020    0x00001027    float or double
#define FLASH_ADCS_CALCULATION_TOLERANCE      (FLASH_ADCS_PARAMETERS + 0x28)  //  0x00001028    0x0000102F    float or double
#define FLASH_ADCS_ACTIVE_EULER_TOLERANCE     (FLASH_ADCS_PARAMETERS + 0x30)  //  0x00001030    0x00001037    float or double
#define FLASH_ADCS_ACTIVE_OMEGA_TOLERANCE     (FLASH_ADCS_PARAMETERS + 0x38)  //  0x00001038    0x0000103F    float or double
#define FLASH_ADCS_ECLIPSE_THRESHOLD          (FLASH_ADCS_PARAMETERS + 0x40)  //  0x00001040    0x00001047    float or double
#define FLASH_ADCS_ROTATION_WEIGHT_RATIO      (FLASH_ADCS_PARAMETERS + 0x48)  //  0x00001048    0x0000104F    float or double
#define FLASH_ADCS_ROTATION_TRIGGER           (FLASH_ADCS_PARAMETERS + 0x50)  //  0x00001050    0x00001057    float or double
#define FLASH_ADCS_DISTURBANCE_COVARIANCE     (FLASH_ADCS_PARAMETERS + 0x58)  //  0x00001058    0x0000105F    float or double
#define FLASH_ADCS_NOISE_COVARIANCE           (FLASH_ADCS_PARAMETERS + 0x58)  //  0x00001060    0x00001067    float or double
#define FLASH_ADCS_TIME_STEP                  (FLASH_ADCS_PARAMETERS + 0x70)  //  0x00001070    0x00001073    uint32_t
#define FLASH_ADCS_BRIDGE_TIMER_UPDATE_PERIOD (FLASH_ADCS_PARAMETERS + 0x74)  //  0x00001074    0x00001077    uint32_t
#define FLASH_ADCS_BRIDGE_OUTPUT_HIGH         (FLASH_ADCS_PARAMETERS + 0x78)  //  0x00001078    0x00001078    int8_t
#define FLASH_ADCS_BRIDGE_OUTPUT_LOW          (FLASH_ADCS_PARAMETERS + 0x79)  //  0x00001079    0x00001079    int8_t
#define FLASH_ADCS_NUM_CONTROLLERS            (FLASH_ADCS_PARAMETERS + 0x7A)  //  0x0000107A    0x0000107A    uint8_t
#define FLASH_ADCS_COIL_CHAR_MATRIX           (FLASH_ADCS_PARAMETERS + 0x100) //  0x00001100    0x00001123    9x float
#define FLASH_ADCS_INERTIA_TENSOR_MATRIX      (FLASH_ADCS_PARAMETERS + 0x124) //  0x00001124    0x00001147    9x float

// sector 2 - stats
#define FLASH_STATS                                               0x00002000  //  0x00002000    0x000020FF

#define FLASH_STATS_TEMP_PANEL_Y                        (FLASH_STATS + 0x00)  //  0x00002000    0x00002005    3x int16_t
#define FLASH_STATS_TEMP_TOP                            (FLASH_STATS + 0x06)  //  0x00002006    0x0000200B    3x int16_t
#define FLASH_STATS_TEMP_BOTTOM                         (FLASH_STATS + 0x0C)  //  0x0000200C    0x00002011    3x int16_t
#define FLASH_STATS_TEMP_BATTERY                        (FLASH_STATS + 0x12)  //  0x00002012    0x00002017    3x int16_t
#define FLASH_STATS_TEMP_SEC_BATTERY                    (FLASH_STATS + 0x18)  //  0x00002018    0x0000201D    3x int16_t
#define FLASH_STATS_TEMP_MCU                            (FLASH_STATS + 0x1E)  //  0x0000201E    0x00002023    3x int16_t

#define FLASH_STATS_CURR_XA                             (FLASH_STATS + 0x24)  //  0x00002024    0x00002029    3x int16_t
#define FLASH_STATS_CURR_XB                             (FLASH_STATS + 0x2A)  //  0x0000202A    0x0000202F    3x int16_t
#define FLASH_STATS_CURR_ZA                             (FLASH_STATS + 0x30)  //  0x00002030    0x00002035    3x int16_t
#define FLASH_STATS_CURR_ZB                             (FLASH_STATS + 0x36)  //  0x00002036    0x0000203B    3x int16_t
#define FLASH_STATS_CURR_Y                              (FLASH_STATS + 0x3C)  //  0x0000203C    0x00002041    3x int16_t
#define FLASH_STATS_CURR_MPPT                           (FLASH_STATS + 0x42)  //  0x00002042    0x00002047    3x int16_t

#define FLASH_STATS_VOLT_XA                             (FLASH_STATS + 0x48)  //  0x00002048    0x0000204A    3x uint8_t
#define FLASH_STATS_VOLT_XB                             (FLASH_STATS + 0x4B)  //  0x0000204B    0x0000204D    3x uint8_t
#define FLASH_STATS_VOLT_ZA                             (FLASH_STATS + 0x4E)  //  0x0000204E    0x00002050    3x uint8_t
#define FLASH_STATS_VOLT_ZB                             (FLASH_STATS + 0x51)  //  0x00002051    0x00002053    3x uint8_t
#define FLASH_STATS_VOLT_Y                              (FLASH_STATS + 0x54)  //  0x00002054    0x00002056    3x uint8_t
#define FLASH_STATS_VOLT_MPPT                           (FLASH_STATS + 0x57)  //  0x00002057    0x00002059    3x uint8_t

#define FLASH_STATS_LIGHT_PANEL_Y                       (FLASH_STATS + 0x5A)  //  0x0000205A    0x0000205F    3x float
#define FLASH_STATS_LIGHT_TOP                           (FLASH_STATS + 0x66)  //  0x00002066    0x00002071    3x float

#define FLASH_STATS_GYRO_X                              (FLASH_STATS + 0x72)  //  0x00002072    0x0000207D    3x float
#define FLASH_STATS_GYRO_Y                              (FLASH_STATS + 0x7E)  //  0x0000207E    0x00002089    3x float
#define FLASH_STATS_GYRO_Z                              (FLASH_STATS + 0x8A)  //  0x0000208A    0x00002095    3x float
#define FLASH_STATS_ACCEL_X                             (FLASH_STATS + 0x96)  //  0x00002096    0x000020A1    3x float
#define FLASH_STATS_ACCEL_Y                             (FLASH_STATS + 0xA2)  //  0x000020A2    0x000020AD    3x float
#define FLASH_STATS_ACCEL_Z                             (FLASH_STATS + 0xAE)  //  0x000020AE    0x000020B9    3x float
#define FLASH_STATS_MAG_X                               (FLASH_STATS + 0xBA)  //  0x000020BA    0x000020C5    3x float
#define FLASH_STATS_MAG_Y                               (FLASH_STATS + 0xC6)  //  0x000020C6    0x000020D1    3x float
#define FLASH_STATS_MAG_Z                               (FLASH_STATS + 0xD2)  //  0x000020D2    0x000020DD    3x float

#define FLASH_STATS_POWER_XA                            (FLASH_STATS + 0x100) //  0x00002100    0x0000210B    3x float
#define FLASH_STATS_POWER_XB                            (FLASH_STATS + 0x10C) //  0x0000210C    0x00002117    3x float
#define FLASH_STATS_POWER_ZA                            (FLASH_STATS + 0x118) //  0x00002118    0x00002123    3x float
#define FLASH_STATS_POWER_ZB                            (FLASH_STATS + 0x124) //  0x00002124    0x0000212F    3x float
#define FLASH_STATS_POWER_Y                             (FLASH_STATS + 0x130) //  0x00002130    0x0000213B    3x float

// sector 3 - ADCS controllers
#define FLASH_ADCS_CONTROLLERS                                    0x00003000  //  0x00003000    0x00003FFF
#define FLASH_ADCS_CONTROLLER_SLOT_SIZE                           (3*6*sizeof(float))

// sectors 4 - 9 - image properites: 12 bytes per picture, 4 bytes total length, 4 bytes scan start address, 4 bytes scan end address
#define FLASH_IMAGE_PROPERTIES                                    0x00004000  //  0x00004000    0x00009FFF
#define FLASH_IMAGE_PROPERTIES_SLOT_SIZE                          (21)        //  properties of 21 pictures in each sector

// 64kB block 1 - store & forward slots
#define FLASH_STORE_AND_FORWARD_START                             0x00010000  //  0x00010000    0x0001FFFF
#define FLASH_STORE_AND_FORWARD_NUM_SLOTS                         (FLASH_64K_BLOCK_SIZE / MAX_STRING_LENGTH)

// 64kB blocks 2 - 31 - NMEA sentences: null-terminated C-strings, each starts with 4-byte timestamp (offset since recording start)
#define FLASH_NMEA_LOG_START                                      0x00020000  //  0x00020000    0x001FFFFF
#define FLASH_NMEA_LOG_END                                        (FLASH_ADCS_EPHEMERIDES_START)
#define FLASH_NMEA_LOG_SLOT_SIZE                                  (128)

// 64kB blocks 32 - 39 - ADCS ephemerides storage. 25 bytes per ephemeris model row, 5 rows in each 128-byte chunk
#define FLASH_ADCS_EPHEMERIDES_START                              0x00200000  //  0x00200000    0x0027FFFF
#define FLASH_ADCS_EPHEMERIDES_END                                (FLASH_IMAGES_START)
#define FLASH_ADCS_EPHEMERIDES_SLOT_SIZE                          (6*sizeof(float) + 1)

// 64kB blocks 40 - 1023 - image slots: 8 blocks per slot
#define FLASH_IMAGES_START                                        0x00280000  //  0x00280000    0x03FFFFFF
#define FLASH_IMAGE_SLOT_SIZE                                     (FLASH_IMAGE_NUM_64K_BLOCKS * FLASH_64K_BLOCK_SIZE)
#define FLASH_PUBLIC_PICTURES_START                               (80)
#define FLASH_PUBLIC_PICTURES_END                                 (100)

/*
    JPEG markers
*/

#define JPEG_MARKER_ESCAPE                              0xFF
#define JPEG_MARKER_SOF0                                0xC0
#define JPEG_MARKER_EOI                                 0xD9

/*
    Radio Configuration
*/

// modem definitions
#define MODEM_FSK                                       'F'
#define MODEM_LORA                                      'L'

// common
#define CALLSIGN_DEFAULT                                "FOSSASAT-2"
#define SYNC_WORD                                       0x12        /*!< Ensure this sync word is compatable with all devices. */
#define TCXO_VOLTAGE                                    1.6         /*!< Sets the radio's TCX0 voltage. (V) */
#define MAX_NUM_OF_BLOCKS                               3           /*!< maximum number of AES128 blocks that will be accepted */
#define LORA_RECEIVE_WINDOW_LENGTH                      40          /*!< How long to listen out for LoRa transmissions for (s) */
#define FSK_RECEIVE_WINDOW_LENGTH                       20          /*!< How long to listen out for FSK transmissions for (s) */
#define RESPONSE_DELAY                                  500         /*!< How long to wait for before responding to a transmission (ms) */
#define RESPONSE_DELAY_SHORT                            0           /*!< Shorter version of RESPONSE_DELAY to be used for certain frames (e.g. picture downlink) (ms) */
#define WHITENING_INITIAL                               0x1FF       /*!< Whitening LFSR initial value, to ensure SX127x compatibility */
#define SCIENCE_MODE_CMD_WHITELIST                      { \
                                                          /* CMD_PING,*/ \
                                                          /* CMD_RETRANSMIT,*/ \
                                                          /* CMD_RETRANSMIT_CUSTOM,*/ \
                                                          CMD_TRANSMIT_SYSTEM_INFO, \
                                                          CMD_GET_PACKET_INFO, \
                                                          /* CMD_GET_STATISTICS,*/ \
                                                          CMD_GET_FULL_SYSTEM_INFO, \
                                                          /* CMD_STORE_AND_FORWARD_ADD,*/ \
                                                          /* CMD_STORE_AND_FORWARD_REQUEST,*/ \
                                                          /* CMD_REQUEST_PUBLIC_PICTURE, */ \
                                                          CMD_DEPLOY, \
                                                          CMD_RESTART, \
                                                          /* CMD_WIPE_EEPROM,*/ \
                                                          CMD_SET_CALLSIGN, \
                                                          CMD_SET_SF_MODE, \
                                                          CMD_SET_MPPT_MODE, \
                                                          CMD_SET_LOW_POWER_ENABLE, \
                                                          CMD_SET_RECEIVE_WINDOWS, \
                                                          /* CMD_RECORD_SOLAR_CELLS,*/ \
                                                          CMD_CAMERA_CAPTURE, \
                                                          CMD_SET_POWER_LIMITS, \
                                                          CMD_SET_RTC, \
                                                          CMD_RECORD_IMU, \
                                                          CMD_RUN_MANUAL_ACS, \
                                                          CMD_LOG_GPS, \
                                                          /* CMD_GET_GPS_LOG,*/ \
                                                          CMD_GET_FLASH_CONTENTS, \
                                                          /* CMD_GET_PICTURE_LENGTH,*/ \
                                                          /* CMD_GET_PICTURE_BURST,*/ \
                                                          CMD_ROUTE, \
                                                          CMD_SET_FLASH_CONTENTS, \
                                                          CMD_SET_TLE, \
                                                          /* CMD_GET_GPS_LOG_STATE,*/ \
                                                          /* CMD_RUN_GPS_COMMAND,*/ \
                                                          CMD_SET_SLEEP_INTERVALS, \
                                                          CMD_ABORT, \
                                                          CMD_MANEUVER, \
                                                          /* CMD_SET_ADCS_PARAMETERS, */ \
                                                          CMD_ERASE_FLASH, \
                                                          /* CMD_SET_ADCS_CONTROLLER, */ \
                                                          /* CMD_SET_ADCS_EPHEMERIDES, */ \
                                                          /* CMD_DETUMBLE */ \
                                                         }          /*!< List of function IDs that remain available in "science mode" (e.g. logging GPS or ADCS closed-loop control) */

// LoRa
#define LORA_FREQUENCY                                  436.7       /*!< MHz */
#define LORA_BANDWIDTH                                  125.0       /*!< kHz dual sideband */
#define LORA_SPREADING_FACTOR                           11
#define LORA_SPREADING_FACTOR_ALT                       10
#define LORA_CODING_RATE                                8           /*!< 4/8, Extended Hamming */
#define LORA_OUTPUT_POWER                               20          /*!< dBm */
#define LORA_CURRENT_LIMIT                              140.0       /*!< mA */
#define LORA_PREAMBLE_LENGTH                            8           /*!< symbols */

// GFSK
#define FSK_FREQUENCY                                   436.9       /*!< MHz */
#define FSK_BIT_RATE                                    9.6         /*!< kbps nominal */
#define FSK_FREQUENCY_DEVIATION                         5.0         /*!< kHz single-sideband */
#define FSK_RX_BANDWIDTH                                39.0        /*!< kHz single-sideband */
#define FSK_OUTPUT_POWER                                20          /*!< dBm */
#define FSK_PREAMBLE_LENGTH                             16          /*!< bits */
#define FSK_DATA_SHAPING                                RADIOLIB_SHAPING_0_5  /*!< GFSK filter BT product */
#define FSK_CURRENT_LIMIT                               140.0       /*!< mA */

// Morse Code
#define NUM_CW_BEEPS                                    3           /*!< number of CW sync beeps in low power mode */
#define MORSE_PREAMBLE_LENGTH                           0           /*!< number of start signal repetitions */
#define MORSE_SPEED                                     20          /*!< words per minute */
#define MORSE_BATTERY_MIN                               3200.0      /*!< minimum voltage value that can be send via Morse (corresponds to 'A'), mV*/
#define MORSE_BATTERY_STEP                              50.0        /*!< voltage step in Morse, mV */
#define MORSE_BEACON_LOOP_FREQ                          2           /*!< how often to transmit full Morse code beacon (e.g. transmit every second main loop when set to 2) */

/*
    Temperature Sensors
*/

// common
#define TMP_100_REG_TEMPERATURE                         0x00
#define TMP_100_REG_CONFIG                              0x01
#define TMP_100_RESOLUTION_9_BITS                       0b00000000  // 0.5 deg. C
#define TMP_100_RESOLUTION_10_BITS                      0b00100000  // 0.25 deg. C
#define TMP_100_RESOLUTION_11_BITS                      0b01000000  // 0.125 deg. C
#define TMP_100_RESOLUTION_12_BITS                      0b01100000  // 0.0625 deg. C (default)
#define TMP_100_LSB_RESOLUTION                          0.0625  // deg. C
#define TMP_100_MODE_CONTINUOUS                         0b00000000
#define TMP_100_MODE_SHUTDOWN                           0b00000001
#define TMP_100_START_ONE_SHOT                          0b10000000

// Y panel temperature sensor
#define TEMP_SENSOR_Y_PANEL_BUS                         Wire
#define TEMP_SENSOR_Y_PANEL_ADDRESS                     0b1001000 // ADD1 low, ADD0 low

// top panel temperature sensor
#define TEMP_SENSOR_TOP_BUS                             Wire2
#define TEMP_SENSOR_TOP_ADDRESS                         0b1001011 // ADD1 float, ADD0 low

// bottom panel temperature sensor
#define TEMP_SENSOR_BOTTOM_BUS                          Wire2
#define TEMP_SENSOR_BOTTOM_ADDRESS                      0b1001000 // ADD1 low, ADD0 low

// battery temperature sensor
#define TEMP_SENSOR_BATTERY_BUS                         Wire2
#define TEMP_SENSOR_BATTERY_ADDRESS                     0b1001001 // ADD1 low, ADD0 float

// second battery temperature sensor
#define TEMP_SENSOR_SEC_BATTERY_BUS                     Wire2
#define TEMP_SENSOR_SEC_BATTERY_ADDRESS                 0b1001010 // ADD1 low, ADD0 high

// MCU temperature sensor
#define TEMP_SENSOR_MCU_BUS                             Wire2
#define TEMP_SENSOR_MCU_ADDRESS                         0b1001010 // ADD1 low, ADD0 high

/*
    ADCS H-bridges
*/

#define ADCS_X_BRIDGE_ADDRESS                           0b1100100 // A1 float, A0 float
#define ADCS_Y_BRIDGE_ADDRESS                           0b1100110 // A1 float, A0 high
#define ADCS_Z_BRIDGE_ADDRESS                           0b1100101 // A1 low, A0 high

/*
   IMU
*/

#define IMU_BUS                                         Wire2
#define IMU_ACCEL_GYRO_ADDRESS                          0b1101011 // SDO_A/G pulled high internally
#define IMU_MAG_ADDRESS                                 0b0011110 // SDO_M pulled high internally
#define IMU_DEG_S_TO_RAD_S                              (M_PI / 180.0)  // gyroscope conversion constant from deg/s to rad/s
#define IMU_G_TO_MS2                                    (9.80665) // accelerometer conversion constant from g units to m/s^2
#define IMU_GAUSS_TO_TESLA                              (0.0001)  // magnetometer conversion constant from gauss to tesla

/*
   Current Sensors
*/

// X axis solar cells
#define CURR_SENSOR_X_A_BUS                             Wire
#define CURR_SENSOR_X_A_ADDRESS                         0b1000001 // A1 low, A0 high
#define CURR_SENSOR_X_B_BUS                             Wire
#define CURR_SENSOR_X_B_ADDRESS                         0b1000010 // A1 low, A0 SDA

// Z axis solar cells
#define CURR_SENSOR_Z_A_BUS                             Wire
#define CURR_SENSOR_Z_A_ADDRESS                         0b1001100 // A1 SCL, A0 low
#define CURR_SENSOR_Z_B_BUS                             Wire
#define CURR_SENSOR_Z_B_ADDRESS                         0b1000100 // A1 high, A0 low

// Y axis solar cells
#define CURR_SENSOR_Y_BUS                               Wire2
#define CURR_SENSOR_Y_ADDRESS                           0b1000000 // A1 low, A0 low

// MPPT output current
#define CURR_SENSOR_MPPT_OUTPUT_BUS                     Wire2
#define CURR_SENSOR_MPPT_OUTPUT_ADDRESS                 0b1000100 // A1 high, A0 low

/*
   Light Sensors
*/

#define LIGHT_SENSOR_GAIN                               VEML7700_GAIN_1_8
#define LIGHT_SENSOR_INTEGRATION_TIME                   VEML7700_IT_25MS
#define LIGHT_SENSOR_Y_PANEL_BUS                        Wire
#define LIGHT_SENSOR_TOP_PANEL_BUS                      Wire2

/*
    ADCS TODO - placeholder values only!!!!
*/

#define ADCS_NUM_AXES                                   3       // number of axes to control
#define ADCS_NUM_PANELS                                 6       // number of solar panels for eclipse decision
#define ADCS_CALC_TYPE                                  double  // numeric type to use in ADCS calculation, float for single precision, double for double precision
#define ADCS_TIME_STEP                                  100     // time step between successive ADCS updates, in ms
#define ADCS_PULSE_MAX_INTENSITY                        1.0    // abs max value in the H bridges
#define ADCS_PULSE_MAX_LENGTH                           (ADCS_TIME_STEP/2.0)  // maximum length of H-bridge pulse
#define ADCS_DETUMB_OMEGA_TOLERANCE                     0.001     // detumbling will be stopped once change in normalized angular velocity drops below this value
#define ADCS_ACTIVE_EULER_TOLERANCE                     0.01     //
#define ADCS_ACTIVE_OMEGA_TOLERANCE                     0.01     //
#define ADCS_MIN_INERTIAL_MOMENT                        0.0003782 //
#define ADCS_PULSE_AMPLITUDE                            1.0       // current pulse amplitude - maximum H-bridge voltage output divided by magnetorquer impednace
#define ADCS_CALCULATION_TOLERANCE                      0.001    //
#define ADCS_ECLIPSE_THRESHOLD                          0.28     // Eclipse condition reaching for a 20% output of the solar panels
#define ADCS_ROTATION_WEIGHT_RATIO                      0.7     //  Weight ratio to average sensor, referred to the Euler integrator scheme (less accurate)
#define ADCS_ROTATION_TRIGGER                           (M_PI/6.0) // Angular difference between sensor to trigger their averaging
#define ADCS_DISTURBANCE_COVARIANCE                     0.001     // Covariance of the dynamical disturbances
#define ADCS_NOISE_COVARIANCE                           0.001     // Covariance of the sensor noise
#define ADCS_COIL_CHARACTERISTICS                       { {893.655,     0,       0}, \
                                                          {     0, 15.833,       0}, \
                                                          {     0,      0, 108.551} }
#define ADCS_PANEL_UNIT_VECTOR                          { {1.0, 0, 0}, \
                                                          {0, 1.0, 0}, \
                                                          {0, 0, 1.0}, \
                                                          {-1.0, 0, 0}, \
                                                          {0, -1.0, 0}, \
                                                          {0,  0,-1.0}  }
#define ADCS_MAX_NUM_CONTROLLERS                        10      //
#define ADCS_DEFAULT_CONTROLLER                         { {1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, \
                                                          {1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, \
                                                          {1.0, 1.0, 1.0, 1.0, 1.0, 1.0} }
#define ADCS_INERTIA_TENSOR                             { {2645.9, 2.2, 4.41}, \
                                                          {2.2, 1167.3, -0.1}, \
                                                          {4.41, -0.1, 1090.9} } // Inertia tensor is introduced in its inversed form
#define ADCS_NUM_CONTROLLERS                            1       //  use only one controller by default
#define ADCS_BRIDGE_TIMER_UPDATE_PERIOD                 (ADCS_TIME_STEP/100)  // time step between successive H bridge output updates, in ms
#define ADCS_BRIDGE_OUTPUT_HIGH                         63      // H bridge drive strength to be used as "high"
#define ADCS_BRIDGE_OUTPUT_LOW                         -63      // H bridge drive strength to be used as "low"
#define ADCS_SOLAR_POWER_XZ_MAX                        (0.9670)  // maximum output power of solar panels X and Z, in W
#define ADCS_SOLAR_POWER_Y_MAX                         (0.2420)  // maximum output power of solar panel Y, in W
#define ADCS_SOLAR_SENSOR_MAX                          (172911.0) // maximum light intensity output of solar sensors, in lux

/*
    Global Variables
*/

// RAM buffer for system info page
extern uint8_t systemInfoBuffer[];

// flag to signal interrupts enabled/disabled
extern volatile bool interruptsEnabled;

// flag to signal data was received from ISR
extern volatile bool dataReceived;

// flag to indicate whether "science mode" is currently active or not
extern bool scienceModeActive;

// flag to abort currently running process
extern volatile bool abortExecution;

// current modem configuration
extern uint8_t currentModem;
extern uint8_t spreadingFactorMode;

// second I2C interface
extern TwoWire Wire2;

// additional SPI interfaces
extern SPIClass RadioSPI;
extern SPIClass FlashSPI;

// additional UART interface
extern HardwareSerial GpsSerial;

// ADCS timer instances
extern HardwareTimer* AdcsTimer;
extern HardwareTimer* HbridgeTimer;

// RadioLib instances
extern SX1268 radio;
extern MorseClient morse;

// camera instance
extern Camera* camera;

// RTC instance
extern STM32RTC& rtc;

// transmission password
extern const char* password;

// encryption key
extern const uint8_t encryptionKey[];

// temperature sensors
extern struct wireSensor_t tempSensorPanelY;
extern struct wireSensor_t tempSensorTop;
extern struct wireSensor_t tempSensorBottom;
extern struct wireSensor_t tempSensorBattery;
extern struct wireSensor_t tempSensorSecBattery;
extern struct wireSensor_t tempSensorMCU;

// ADCS H-bridges
extern MiniMoto bridgeX;
extern MiniMoto bridgeY;
extern MiniMoto bridgeZ;

// IMU
extern LSM9DS1 imu;

// current sensors
extern currentSensor_t currSensorXA;
extern currentSensor_t currSensorXB;
extern currentSensor_t currSensorZA;
extern currentSensor_t currSensorZB;
extern currentSensor_t currSensorY;
extern currentSensor_t currSensorMPPT;

// light sensors
extern lightSensor_t lightSensorPanelY;
extern lightSensor_t lightSensorTop;

// GPS logging variables (global since GPS logging is event-driven)
extern gpsLogState_t gpsLogState;

// ADCS state variables (global since ADCS updates are interrupt-driven)
extern volatile adcsState_t adcsState;

// cached ADCS parameters
extern adcsParams_t adcsParams;

void Configuration_Setup();

#endif
