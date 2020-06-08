#include "Configuration.h"

// RAM buffer for system info page
uint8_t systemInfoBuffer[FLASH_EXT_PAGE_SIZE];

// flag to signal interrupts enabled/disabled
volatile bool interruptsEnabled = true;

// flag to signal data was received from ISR
volatile bool dataReceived = false;

// flag to indicate whether "science mode" is currently active or not
bool scienceModeActive = false;

// flag to abort currently running process
volatile bool abortExecution = false;

// current modem configuration
uint8_t currentModem = MODEM_FSK;
uint8_t spreadingFactorMode = LORA_SPREADING_FACTOR;

// second I2C instance
TwoWire Wire2;

// additional SPI interfaces
SPIClass RadioSPI(RADIO_MOSI, RADIO_MISO, RADIO_SCK);
SPIClass FlashSPI(FLASH_MOSI, FLASH_MISO, FLASH_SCK);

// additional UART interface
HardwareSerial GpsSerial(GPS_RX, GPS_TX);

// ADCS timer instances
HardwareTimer* AdcsTimer = new HardwareTimer(TIM2);
HardwareTimer* HbridgeTimer = new HardwareTimer(TIM3);

// RadioLib instances
SX1268 radio = new Module(RADIO_NSS, RADIO_DIO1, RADIO_NRST, RADIO_BUSY, RadioSPI, SPISettings(2000000, MSBFIRST, SPI_MODE0));
MorseClient morse(&radio);

// camera instance
Camera* camera = ArduCAM::createCamera(OV2640, CAMERA_CS);

// RTC instance
STM32RTC& rtc = STM32RTC::getInstance();

// transmission password
const char* password = "password";

// encryption key
const uint8_t encryptionKey[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                                 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x00};

// temperature sensors
struct wireSensor_t tempSensorPanelY =      { .bus = TEMP_SENSOR_Y_PANEL_BUS,     .addr = TEMP_SENSOR_Y_PANEL_ADDRESS,      .res = TMP_100_RESOLUTION_12_BITS, .mode = TMP_100_MODE_SHUTDOWN };
struct wireSensor_t tempSensorTop =         { .bus = TEMP_SENSOR_TOP_BUS,         .addr = TEMP_SENSOR_TOP_ADDRESS,          .res = TMP_100_RESOLUTION_12_BITS, .mode = TMP_100_MODE_SHUTDOWN };
struct wireSensor_t tempSensorBottom =      { .bus = TEMP_SENSOR_BOTTOM_BUS,      .addr = TEMP_SENSOR_BOTTOM_ADDRESS,       .res = TMP_100_RESOLUTION_12_BITS, .mode = TMP_100_MODE_SHUTDOWN };
struct wireSensor_t tempSensorBattery =     { .bus = TEMP_SENSOR_BATTERY_BUS,     .addr = TEMP_SENSOR_BATTERY_ADDRESS,      .res = TMP_100_RESOLUTION_12_BITS, .mode = TMP_100_MODE_SHUTDOWN };
struct wireSensor_t tempSensorSecBattery =  { .bus = TEMP_SENSOR_SEC_BATTERY_BUS, .addr = TEMP_SENSOR_SEC_BATTERY_ADDRESS,  .res = TMP_100_RESOLUTION_12_BITS, .mode = TMP_100_MODE_SHUTDOWN };
struct wireSensor_t tempSensorMCU =         { .bus = TEMP_SENSOR_MCU_BUS,         .addr = TEMP_SENSOR_MCU_ADDRESS,          .res = TMP_100_RESOLUTION_12_BITS, .mode = TMP_100_MODE_SHUTDOWN };

// ADCS H-bridges
MiniMoto bridgeX(ADCS_X_BRIDGE_ADDRESS);
MiniMoto bridgeY(ADCS_Y_BRIDGE_ADDRESS);
MiniMoto bridgeZ(ADCS_Z_BRIDGE_ADDRESS);

// IMU
LSM9DS1 imu;

// current sensors
currentSensor_t currSensorXA =    { .driver = new Adafruit_INA260(), .bus = CURR_SENSOR_X_A_BUS,          .addr = CURR_SENSOR_X_A_ADDRESS,          .available = false };
currentSensor_t currSensorXB =    { .driver = new Adafruit_INA260(), .bus = CURR_SENSOR_X_B_BUS,          .addr = CURR_SENSOR_X_B_ADDRESS,          .available = false };
currentSensor_t currSensorZA =    { .driver = new Adafruit_INA260(), .bus = CURR_SENSOR_Z_A_BUS,          .addr = CURR_SENSOR_Z_A_ADDRESS,          .available = false };
currentSensor_t currSensorZB =    { .driver = new Adafruit_INA260(), .bus = CURR_SENSOR_Z_B_BUS,          .addr = CURR_SENSOR_Z_B_ADDRESS,          .available = false };
currentSensor_t currSensorY  =    { .driver = new Adafruit_INA260(), .bus = CURR_SENSOR_Y_BUS,            .addr = CURR_SENSOR_Y_ADDRESS,            .available = false };
currentSensor_t currSensorMPPT =  { .driver = new Adafruit_INA260(), .bus = CURR_SENSOR_MPPT_OUTPUT_BUS,  .addr = CURR_SENSOR_MPPT_OUTPUT_ADDRESS,  .available = false };

// light sensors
lightSensor_t lightSensorPanelY = { .driver = new Adafruit_VEML7700(), .bus = LIGHT_SENSOR_Y_PANEL_BUS,   .available = false };
lightSensor_t lightSensorTop =    { .driver = new Adafruit_VEML7700(), .bus = LIGHT_SENSOR_TOP_PANEL_BUS, .available = false };

// GPS logging variables (global since GPS logging is done from serialEvent)
gpsLogState_t gpsLogState;

// ADCS state variables (global since ADCS updates are interrupt-driven)
volatile adcsState_t adcsState;

// cached ADCS parameters
adcsParams_t adcsParams;

void Configuration_Setup() {
  // initialize pins
  pinMode(FLASH_CS, OUTPUT);
  digitalWrite(FLASH_CS, HIGH);
  pinMode(FLASH_RESET, INPUT);

  pinMode(CAMERA_CS, OUTPUT);
  digitalWrite(CAMERA_CS, HIGH);
  pinMode(CAMERA_POWER_FET, OUTPUT);
  digitalWrite(CAMERA_POWER_FET, POWER_FET_POLARITY_OFF);

  pinMode(DEPLOYMENT_FET_1, OUTPUT);
  pinMode(DEPLOYMENT_FET_2, OUTPUT);
  pinMode(BATTERY_HEATER_FET, OUTPUT);

  pinMode(WATCHDOG_IN, OUTPUT);
  pinMode(MPPT_OFF, OUTPUT);
  pinMode(ANALOG_IN_RANDOM_SEED, INPUT_ANALOG);

  // provide seed for encrpytion PRNG
  randomSeed(analogRead(ANALOG_IN_RANDOM_SEED));

  pinMode(GPS_POWER_FET, OUTPUT);
  digitalWrite(GPS_POWER_FET, POWER_FET_POLARITY_OFF);

  // initialize default I2C interface
  Wire.setSDA(I2C1_SDA);
  Wire.setSCL(I2C1_SCL);
  Wire.begin();

  // initialize second I2C interface
  Wire2.setSDA(I2C2_SDA);
  Wire2.setSCL(I2C2_SCL);
  Wire2.begin();

  // initialize SPI interfaces
  FlashSPI.begin();
  RadioSPI.begin();
  SPI.setMOSI(CAMERA_MOSI);
  SPI.setMISO(CAMERA_MISO);
  SPI.setSCLK(CAMERA_SCK);
  SPI.begin();

  // initialize RTC
  rtc.setClockSource(STM32RTC::LSE_CLOCK);
  rtc.begin();

  // initialize low power library
  LowPower.begin();
}
