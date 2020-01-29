#ifndef _FOSSASAT_CONFIGURATION_H
#define _FOSSASAT_CONFIGURATION_H

#include "FossaSat2.h"

/*
    Configuration Macros - All macros MUST be enabled prior to integration!!!
*/

// uncomment reset system info (callsing, configuration etc.) on start
//#define RESET_SYSTEM_INFO

// comment out to disable deployment sequence
#define ENABLE_DEPLOYMENT_SEQUENCE

// comment out to disable sleep during deployment sequence
#define ENABLE_DEPLOYMENT_SLEEP

// comment out to disable transmission control (transmission disable and no transmissions in low power mode)
#define ENABLE_TRANSMISSION_CONTROL

/*
    Array Length Limits
*/

// string length limit
#define MAX_STRING_LENGTH                               32

// optional data length limit
#define MAX_OPT_DATA_LENGTH                             128

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
#define ANALOG_IN_RANDOM_SEED                           PA12    // used as source for randomSeed(), should be left floating


/*
   Timing Definitions
*/

#define DEPLOYMENT_DEBUG_LENGTH                         60      // s
#define DEPLOYMENT_DEBUG_SAMPLE_PERIOD                  1000    // ms
#define DEPLOYMENT_SLEEP_LENGTH                         1500000 // ms
#define DEPLOYMENT_CHARGE_LIMIT                         12      // h
#define DEPLOYMENT_PULSE_LENGTH                         1200    // ms
#define WATCHDOG_LOOP_HEARTBEAT_PERIOD                  1000    // ms

/*
   Voltage Limits
*/

#define DEPLOYMENT_BATTERY_VOLTAGE_LIMIT                3700.0  // mV
#define HEATER_BATTERY_VOLTAGE_LIMIT                    3800.0  // mV
#define BATTERY_CW_BEEP_VOLTAGE_LIMIT                   3800.0        /*!< Battery voltage limit to switch into morse beep (mV). */
#define LOW_POWER_MODE_VOLTAGE_LIMIT                    3800.0

/*
   Temperature Limits
*/

#define BATTERY_HEATER_TEMP_LIMIT                       5.0     // deg. C
#define MPPT_TEMP_LIMIT                                 0.0     // deg. C
#define BATTERY_HEATER_DUTY_CYCLE                       255     // PWM duty cycle 0 - 255

/*
    Flash Configuration
*/

#define FLASH_PAGE_SIZE                                 0x00000100
#define FLASH_SECTOR_SIZE                               0x00001000
#define FLASH_64K_BLOCK_SIZE                            0x00010000
#define FLASH_IMAGE_NUM_64K_BLOCKS                      8

// Flash address map                                                    LSB           MSB
// 64kB block 0 - system info, stats, image lengths
// sector 0 - system info
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

// sector 1 - stats
// todo stats
#define FLASH_STATS                                     0x00001000  //  0x00001000    0x00001FFF

// sector 2 - image lengths: 4 bytes per length
#define FLASH_IMAGE_LENGTHS                             0x00002000  //  0x00002000    0x00002FFF

// 64kB block 1 - store & forward slots
#define FLASH_STORE_AND_FORWARD_START                   0x00020000  //  0x00020000    0x0002FFFF

// 64kB block 2 - NMEA sentences: null-terminated C-strings, each starts with 4-byte timestamp (offset since recording start)
#define FLASH_NMEA_START                                0x00030000  //  0x00030000    0x0003FFFF

// 64kB blocks 8 - 1023 - image slots: 8 blocks per slot
#define FLASH_IMAGES_START                              0x00070000  //  0x00070000    0x03FFFFFF
#define FLASH_IMAGE_SLOT_SIZE                           (FLASH_IMAGE_NUM_64K_BLOCKS * FLASH_64K_BLOCK_SIZE)

/*
    Radio Configuration
*/

// modem definitions
#define MODEM_FSK                                       'F'
#define MODEM_LORA                                      'L'

// common
#define CALLSIGN_DEFAULT                                "FOSSASAT-2"
#define CARRIER_FREQUENCY                               436.7       /*!< MHz */
#define SYNC_WORD                                       0x12        /*!< Ensure this sync word is compatable with all devices. */
#define TCXO_VOLTAGE                                    1.6         /*!< Sets the radio's TCX0 voltage. (V) */
#define MAX_NUM_OF_BLOCKS                               3           /*!< maximum number of AES128 blocks that will be accepted */
#define LORA_RECEIVE_WINDOW_LENGTH                      40          /*!< How long to listen out for LoRa transmissions for (s) */
#define FSK_RECEIVE_WINDOW_LENGTH                       20          /*!< How long to listen out for FSK transmissions for (s) */
#define RESPONSE_DELAY                                  1000        /*!< How long to wait for before responding/processing a transmission (ms) */

// LoRa
#define LORA_BANDWIDTH                                  125.0       /*!< kHz dual sideband */
#define LORA_SPREADING_FACTOR                           11
#define LORA_SPREADING_FACTOR_ALT                       10
#define LORA_CODING_RATE                                8           /*!< 4/8, Extended Hamming */
#define LORA_OUTPUT_POWER                               20          /*!< dBm */
#define LORA_CURRENT_LIMIT                              140.0       /*!< mA */
#define LORA_PREAMBLE_LENGTH                            8       // symbols

// GFSK
#define FSK_BIT_RATE                                    9.6         /*!< kbps nominal */
#define FSK_FREQUENCY_DEVIATION                         5.0         /*!< kHz single-sideband */
#define FSK_RX_BANDWIDTH                                39.0        /*!< kHz single-sideband */
#define FSK_OUTPUT_POWER                                20          /*!< dBm */
#define FSK_PREAMBLE_LENGTH                             16          /*!< bits */
#define FSK_DATA_SHAPING                                0.5         /*!< GFSK filter BT product */
#define FSK_CURRENT_LIMIT                               140.0       /*!< mA */

// Morse Code
#define NUM_CW_BEEPS                                    3           /*!< number of CW sync beeps in low power mode */
#define MORSE_PREAMBLE_LENGTH                           3           /*!< number of start signal repetitions */
#define MORSE_SPEED                                     20          /*!< words per minute */
#define MORSE_BATTERY_MIN                               3200.0      /*!< minimum voltage value that can be send via Morse (corresponds to 'A'), mV*/
#define MORSE_BATTERY_STEP                              50.0        /*!< voltage step in Morse, mV*/

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

/*
    ADCS H-bridges
*/

#define ADCS_X_BRIDGE_ADDRESS                           0b1100100 // A1 float, A0 float
#define ADCS_Y_BRIDGE_ADDRESS                           0b1100101 // A1 float, A0 high
#define ADCS_Z_BRIDGE_ADDRESS                           0b1100110 // A1 high, A0 low

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

#define LIGHT_SENSOR_GAIN                               VEML7700_GAIN_1
#define LIGHT_SENSOR_INTEGRATION_TIME                   VEML7700_IT_800MS
#define LIGHT_SENSOR_Y_PANEL_BUS                        Wire
#define LIGHT_SENSOR_TOP_PANEL_BUS                      Wire2

/*
    Global Variables
*/

// debug-only stopwatch
FOSSASAT_DEBUG_STOPWATCH_INIT_H

// flag to signal interrupts enabled/disabled
extern volatile bool interruptsEnabled;

// flag to signal data was received from ISR
extern volatile bool dataReceived;

// current modem configuration
extern uint8_t currentModem;

extern uint8_t spreadingFactorMode;

// timestamps
extern uint32_t lastHeartbeat;

// second I2C interface
extern TwoWire Wire2;

// additional SPI interfaces
extern SPIClass RadioSPI;
extern SPIClass FlashSPI;

// additional UART interface
extern HardwareSerial GpsSerial;

// RadioLib instances
extern SX1262 radio;
extern MorseClient morse;

// camera instance
extern ArduCAM camera;

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
