#include "Configuration.h"

// debug-only stopwatch
FOSSASAT_DEBUG_STOPWATCH_INIT_CPP

// flag to signal interrupts enabled/disabled
volatile bool interruptsEnabled = true;

// flag to signal data was received from ISR
volatile bool dataReceived = false;

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

// RadioLib instances
SX1268 radio = new Module(RADIO_NSS, RADIO_DIO1, RADIO_NRST, RADIO_BUSY, RadioSPI);
MorseClient morse(&radio);

// camera instance
ArduCAM camera(OV2640, CAMERA_CS);

// RTC instance
STM32RTC& rtc = STM32RTC::getInstance();

// transmission password
const char* password = "password";

// encryption key
const uint8_t encryptionKey[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                                 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x00};

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
  digitalWrite(FLASH_CS, HIGH);
  pinMode(FLASH_RESET, INPUT);
  
  pinMode(CAMERA_CS, OUTPUT);
  digitalWrite(CAMERA_CS, HIGH);
  pinMode(CAMERA_POWER_FET, OUTPUT);
  digitalWrite(CAMERA_POWER_FET, LOW);

  pinMode(DEPLOYMENT_FET_1, OUTPUT);
  pinMode(DEPLOYMENT_FET_2, OUTPUT);
  pinMode(BATTERY_HEATER_FET, OUTPUT);
  
  pinMode(WATCHDOG_IN, OUTPUT);
  pinMode(MPPT_OFF, OUTPUT);
  pinMode(ANALOG_IN_RANDOM_SEED, INPUT);

  pinMode(GPS_POWER_FET, OUTPUT);
  digitalWrite(GPS_POWER_FET, LOW);

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
  SPI.setMOSI(CAMERA_MOSI);
  SPI.setMISO(CAMERA_MISO);
  SPI.setSCLK(CAMERA_SCK);
  SPI.begin();

  // initialize UART interfaces
  GpsSerial.begin(9600);

  // initialize RTC
  rtc.setClockSource(STM32RTC::LSE_CLOCK);
  rtc.begin();
  
  // provide seed for encrpytion PRNG
  randomSeed(analogRead(ANALOG_IN_RANDOM_SEED));

  // initialize low power library
  LowPower.begin();
  LowPower.attachInterruptWakeup(RADIO_DIO1, Communication_Receive_Interrupt, RISING);
}
