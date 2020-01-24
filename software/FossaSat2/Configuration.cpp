#include "Configuration.h"

// debug-only stopwatch
FOSSASAT_DEBUG_STOPWATCH_INIT_CPP

// flag to signal interrupts enabled/disabled
volatile bool interruptsEnabled = true;

// flag to signal data was received from ISR
volatile bool dataReceived = false;

// flag to signal modem should be switched
volatile bool switchModem = false;

// current modem configuration
uint8_t currentModem = MODEM_FSK;

// timestamps
uint32_t lastTransmit = 0;
uint32_t lastMorseTransmit = 0;
uint32_t lastHeartbeat = 0;

// hardware timer instance
HardwareTimer tmr(TIM1);

// second I2C instance
TwoWire Wire2;

// RadioLib instances
SX1268 radio = new Module(RADIO_NSS, RADIO_DIO1, RADIO_BUSY);
MorseClient morse(&radio);

// camera instance
ArduCAM camera(OV2640, CAMERA_CS);

// transmission password
const char* password = "password";

// encryption key
const uint8_t encryptionKey[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
                                };

// temperature sensors
struct wireSensor_t tempSensorPanelY = { .bus = TEMP_SENSOR_Y_PANEL_BUS, .addr = TEMP_SENSOR_Y_PANEL_ADDRESS};
struct wireSensor_t tempSensorTop = { .bus = TEMP_SENSOR_TOP_BUS, .addr = TEMP_SENSOR_TOP_ADDRESS};
struct wireSensor_t tempSensorBottom = { .bus = TEMP_SENSOR_BOTTOM_BUS, .addr = TEMP_SENSOR_BOTTOM_ADDRESS};
struct wireSensor_t tempSensorBattery = { .bus = TEMP_SENSOR_BATTERY_BUS, .addr = TEMP_SENSOR_BATTERY_ADDRESS};
struct wireSensor_t tempSensorSecBattery = { .bus = TEMP_SENSOR_SEC_BATTERY_BUS, .addr = TEMP_SENSOR_SEC_BATTERY_ADDRESS};

// ADCS H-bridges
MiniMoto bridgeX(ADCS_X_BRIDGE_ADDRESS);
MiniMoto bridgeY(ADCS_Y_BRIDGE_ADDRESS);
MiniMoto bridgeZ(ADCS_Z_BRIDGE_ADDRESS);

// IMU
LSM9DS1 imu;

// current sensors
Adafruit_INA260 currSensorXA = Adafruit_INA260();
Adafruit_INA260 currSensorXB = Adafruit_INA260();
Adafruit_INA260 currSensorZA = Adafruit_INA260();
Adafruit_INA260 currSensorZB = Adafruit_INA260();
Adafruit_INA260 currSensorY = Adafruit_INA260();
Adafruit_INA260 currSensorMPPT = Adafruit_INA260();

// light sensors
Adafruit_VEML7700 lightSensorPanelY = Adafruit_VEML7700();
Adafruit_VEML7700 lightSensorTop = Adafruit_VEML7700();

void Configuration_Setup() {
  // initialize pins
  pinMode(FLASH_CS, OUTPUT);
  pinMode(FLASH_RESET, OUTPUT);

  pinMode(DEPLOYMENT_FET_1, OUTPUT);
  pinMode(DEPLOYMENT_FET_2, OUTPUT);
  pinMode(CAMERA_POWER_FET, OUTPUT);
  pinMode(BATTERY_HEATER_FET, OUTPUT);

  pinMode(CAMERA_CS, OUTPUT);
  pinMode(WATCHDOG_IN, OUTPUT);
  pinMode(MPPT_OFF, OUTPUT);
  pinMode(ANALOG_IN_RANDOM_SEED, INPUT);

  digitalWrite(CAMERA_CS, HIGH);

  // initialize default I2C interface
  Wire.setSDA(I2C1_SDA);
  Wire.setSCL(I2C1_SCL);
  Wire.begin();

  // initialize second I2C interface
  Wire2.setSDA(I2C2_SDA);
  Wire2.setSCL(I2C2_SCL);
  Wire2.begin();

  // initialize default SPI interface
  SPI.setSCLK(SPI1_SCK);
  SPI.setMISO(SPI1_MISO);
  SPI.setMOSI(SPI1_MOSI);
  SPI.begin();

  // provide seed for encrpytion PRNG
  randomSeed(analogRead(ANALOG_IN_RANDOM_SEED));

  // initialize low power library
  LowPower.begin();
}
