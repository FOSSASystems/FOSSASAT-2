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

#define OVERRIDE_INA                                    currSensorZB

/*
    Array Length Limits
*/

// string length limit
#define MAX_STRING_LENGTH                               32

// maximum number of image bytes in packet
#define MAX_IMAGE_PACKET_LENGTH                         128

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
#define DEFAULT_NUMBER_OF_SLEEP_INTERVALS               6
#define DEFAULT_SLEEP_INTERVAL_VOLTAGES                 { 4050, 4000, 3900, 3800, 3700,    0 }    // mV
#define DEFAULT_SLEEP_INTERVAL_LENGTHS                  {   20,   35,  100,  160,  180,  240 }    // sec

/*
   Default TLE
*/
#define TLE_LINE_1                                      "1 00000U 20000A   20182.79906828 0.00000000  00000-0 -12345-3 0    10"
#define TLE_LINE_2                                      "2 00000 137.0503 217.9687 0010435 173.7291 322.4297 15.88896416000000"

/*
   Flash Configuration
*/

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
#define RESPONSE_DELAY_SHORT                            100         /*!< Shorter version of RESPONSE_DELAY to be used for certain frames (e.g. Flash downlink) (ms) */
#define WHITENING_INITIAL                               0x1FF       /*!< Whitening LFSR initial value, to ensure SX127x compatibility */

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
#define FSK_DATA_SHAPING                                0.5         /*!< GFSK filter BT product */
#define FSK_CURRENT_LIMIT                               140.0       /*!< mA */

// Morse Code
#define NUM_CW_BEEPS                                    3           /*!< number of CW sync beeps in low power mode */
#define MORSE_PREAMBLE_LENGTH                           0           /*!< number of start signal repetitions */
#define MORSE_SPEED                                     20          /*!< words per minute */
#define MORSE_BATTERY_MIN                               3200.0      /*!< minimum voltage value that can be send via Morse (corresponds to 'A'), mV */
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
    Global Variables
*/

// RAM buffer for system info page
extern uint8_t systemInfoBuffer[];

// flag to signal interrupts enabled/disabled
extern volatile bool interruptsEnabled;

// flag to signal data was received from ISR
extern volatile bool dataReceived;

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
extern Adafruit_INA260 currSensorXA;
extern Adafruit_INA260 currSensorXB;
extern Adafruit_INA260 currSensorZA;
extern Adafruit_INA260 currSensorZB;
extern Adafruit_INA260 currSensorY;
extern Adafruit_INA260 currSensorMPPT;

// light sensors
extern Adafruit_VEML7700 lightSensorPanelY;
extern Adafruit_VEML7700 lightSensorTop;

void Configuration_Setup();

#endif
