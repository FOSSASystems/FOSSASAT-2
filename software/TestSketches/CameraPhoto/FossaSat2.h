// C++ libraries
#include <string.h>

// Arduino/STM32 libraries
#include <STM32RTC.h>
#include <STM32LowPower.h>
#include <SPI.h>
#include <Wire.h>

// FOSSA libraries
#include <FOSSA-Comms.h>

// 3rd party libraries
#include <Adafruit_INA260.h>
#include <Adafruit_VEML7700.h>
#include <aes.h>
#include <ArduCAM.h>
#include <GroveMiniMoto.h>
#include <SparkFunLSM9DS1.h>

// RadioLib
#define RADIOLIB_STATIC_ONLY
#include <RadioLib.h>

// files
#include "src/Camera.h"
#include "src/Communication.h"
#include "src/Configuration.h"
#include "src/Debug.h"
#include "src/Navigation.h"
#include "src/PersistentStorage.h"
#include "src/PowerControl.h"
#include "src/Sensors.h"
#include "src/Types.h"

// Reed-Solomon FEC
#include "src/FEC/rs8.h"

// CRC32
#include "src/FEC/crc32.h"

// ADCS
#include "src/ADCS/ADCS/adcs.h"
