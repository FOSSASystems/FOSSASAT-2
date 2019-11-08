// C++ libraries
#include <string.h>

// Arduino/STM32 libraries
#include <STM32LowPower.h>
#include <Wire.h>

// FOSSA libraries
#include <FOSSA-Comms.h>

// 3rd party libraries
#include <Adafruit_INA260.h>
#include <Adafruit_VEML7700.h>
#include <aes.hpp>
#include <GroveMiniMoto.h>
#include <RadioLib.h>
#include <SparkFunLSM9DS1.h>

// files
#include "AttitudeControl.h"
#include "Camera.h"
#include "Communication.h"
#include "Configuration.h"
#include "Debug.h"
#include "PersistentStorage.h"
#include "PowerControl.h"
#include "Sensors.h"
#include "Types.h"
