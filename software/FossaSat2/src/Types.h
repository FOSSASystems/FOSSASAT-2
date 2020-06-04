#ifndef _FOSSASAT_TYPES_H
#define _FOSSASAT_TYPES_H

#include "FossaSat2.h"

// structure to save data about I2C sensors
struct wireSensor_t {
  TwoWire& bus;
  uint8_t addr;
  uint8_t res;
  uint8_t mode;
};

struct lightSensor_t {
  Adafruit_VEML7700* driver;
  TwoWire& bus;
  bool available;
};

struct currentSensor_t {
  Adafruit_INA260* driver;
  TwoWire& bus;
  uint8_t addr;
  bool available;
};

// TODO GPS struct
/*struct gpsLogState_t {

};*/

struct adcsState_t {
  float prevIntensity[3];
  float prevOmegaNorm;
  uint32_t start;
  bool active;
};

struct adcsParams_t {
  float maxPulseInt;
  float maxPulseLen;
  float omegaTol;
  uint32_t timeStep;
  float minInertialMoment;
  float orbInclination;
  float orbPeriod;
  float pulseAmplitude;
  uint32_t detumbleLen;
};

#endif
