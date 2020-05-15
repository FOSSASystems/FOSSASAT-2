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
#include "Camera.h"
#include "Communication.h"
#include "Configuration.h"
#include "Debug.h"
#include "Navigation.h"
#include "PersistentStorage.h"
#include "PowerControl.h"
#include "Sensors.h"
#include "Types.h"
