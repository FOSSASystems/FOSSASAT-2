#ifndef _FOSSASAT_CONFIGURATION_H
#define _FOSSASAT_CONFIGURATION_H

#include "FossaSat2.h"

/*
    Configuration Macros - All macros MUST be enabled prior to integration!!!
*/

// uncomment reset system info (callsing, configuration etc.) on start
//#define RESET_SYSTEM_INFO

// comment out to disable deployment sequence
//#define ENABLE_DEPLOYMENT_SEQUENCE

// comment out to disable sleep during deployment sequence
//#define ENABLE_DEPLOYMENT_SLEEP

// comment out to disable transmission control (transmission disable and no transmissions in low power mode)
//#define ENABLE_TRANSMISSION_CONTROL

// comment out to disable reset on radio init error
#define ENABLE_RADIO_ERROR_RESET

/*
    Array Length Limits
*/

// string length limit
#define MAX_STRING_LENGTH                               32

// optional data length limit
#define MAX_OPT_DATA_LENGTH                             64

// radio buffer length limit
#define MAX_RADIO_BUFFER_LENGTH                         256

/*
    Pin Mapping
*/

// I2C
#define I2C1_SDA                                        PA10
#define I2C1_SCL                                        PA9
#define I2C2_SDA                                        PB11
#define I2C2_SCL                                        PB10

// SPI
#define SPI1_SCK                                        PA5
#define SPI1_MISO                                       PA6
#define SPI1_MOSI                                       PA7

// radio
#define RADIO_NSS                                       PC7
#define RADIO_BUSY                                      PC6
#define RADIO_DIO1                                      PB12

// external flash
#define FLASH_CS                                        PC11
#define FLASH_RESET                                     PC12

// control FETs
#define DEPLOYMENT_FET_1                                PB1
#define DEPLOYMENT_FET_2                                PC4
#define CAMERA_POWER_FET                                PC9
#define BATTERY_HEATER_FET                              PC8

// other
#define CAMERA_CS                                       PC10
#define WATCHDOG_IN                                     PB2
#define MPPT_OFF                                        PA8
#define ANALOG_IN_RANDOM_SEED                           PA11    // used as source for randomSeed(), should be left floating

/*
   Low Power Modes
*/

#define LOW_POWER_NONE                                  0
#define LOW_POWER_IDLE                                  1
#define LOW_POWER_SLEEP                                 2
#define LOW_POWER_DEEP_SLEEP                            3
#define LOW_POWER_SHUTDOWN                              4

/*
   Timing Definitions
*/

#define DEPLOYMENT_DEBUG_LENGTH                         60      // s
#define DEPLOYMENT_DEBUG_SAMPLE_PERIOD                  1000    // ms
#define DEPLOYMENT_SLEEP_LENGTH                         1500000 // ms
#define DEPLOYMENT_CHARGE_LIMIT                         12      // h
#define DEPLOYMENT_PULSE_LENGTH                         1200    // ms
#define WATCHDOG_LOOP_HEARTBEAT_PERIOD                  1000    // ms
#define BATTERY_CHECK_PERIOD                            300000  // ms

/*
   Voltage Limits
*/

#define DEPLOYMENT_BATTERY_LEVEL_LIMIT                  3700.0  // mV
#define HEATER_BATTERY_LEVEL_LIMIT                      3800.0  // mV

/*
   Temperature Limits
*/

#define BATTERY_HEATER_TEMP_LIMIT                       5.0     // deg. C
#define MPPT_TEMP_LIMIT                                 0.0     // deg. C

/*
    Flash Configuration
*/

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

/*
    Radio Configuration
*/

// modem definitions
#define MODEM_FSK                                       0
#define MODEM_LORA                                      1

// common
#define CALLSIGN_DEFAULT                                "FOSSASAT-2"
#define CARRIER_FREQUENCY                               436.7   // MHz
#define TCXO_VOLTAGE                                    1.6     // V
#define MAX_NUM_OF_BLOCKS                               3       // maximum number of AES128 blocks that will be accepted
#define MODEM_SWITCHING_PERIOD_FSK                      2000000 // us
#define MODEM_SWITCHING_PERIOD_LORA                     6000000 // us
#define SYSTEM_INFO_TRANSMIT_PERIOD                     60000   // us
#define MORSE_BEACON_PERIOD                             60000   // us

// LoRa
#define LORA_BANDWIDTH                                  125.0   // kHz (dual sideband)
#define LORA_OUTPUT_POWER                               20      // dBm
#define LORA_SPREADING_FACTOR                           11      //
#define LORA_CODING_RATE                                5       // parity only
#define LORA_SYNC_WORD                                  0x12    //
#define LORA_PREAMBLE_LENGTH                            8       // symbols
#define LORA_CURRENT_LIMIT                              120     // mA

// GFSK
#define FSK_FREQUENCY_DEVIATION                         5.0     // kHz (single sideband)
#define FSK_OUTPUT_POWER                                20      // dBm
#define FSK_BIT_RATE                                    9.6     // kbps
#define FSK_DATA_SHAPING                                0.5     // FSK filter BT product
#define FSK_SYNC_WORD                                   {0x12, 0x12}
#define FSK_PREAMBLE_LENGTH                             16      // bits
#define FSK_RX_BANDWIDTH                                19.5    // kHz (single sideband)
#define FSK_CURRENT_LIMIT                               200     // mA

// Morse Code
#define MORSE_SPEED                                     12      // words per minute (assuming PARIS as standard word)
#define MORSE_PREAMBLE_LENGTH                           3       // symbols

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
#define TEMP_SENSOR_SEC_BATTERY_ADDRESS                 0b1001100 // ADD1 float, ADD0 float ?????????

/*
    ADCS H-bridges
*/

#define ADCS_X_BRIDGE_ADDRESS                           0b1100100 // A1 float, A0 float
#define ADCS_Y_BRIDGE_ADDRESS                           0b1100101 // A1 float, A0 high
#define ADCS_Z_BRIDGE_ADDRESS                           0b1100010 // A1 low, A0 high

/*
   IMU
*/

#define IMU_BUS                                         Wire2
#define IMU_ACCEL_GYRO_ADDRESS                          0b1101010 // SDO_A/G low
#define IMU_MAG_ADDRESS                                 0b0011100 // SDO_M low

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

// flag to signal modem should be switched
extern volatile bool switchModem;

// current modem configuration
extern uint8_t currentModem;

// timestamps
extern uint32_t lastTransmit;
extern uint32_t lastMorseTransmit;
extern uint32_t lastHeartbeat;

// hardware timer instance
extern HardwareTimer tmr;

// second I2C instance
extern TwoWire Wire2;

// RadioLib instances
extern SX1268 radio;
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
