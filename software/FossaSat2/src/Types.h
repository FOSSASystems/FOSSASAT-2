#ifndef _FOSSASAT_TYPES_H
#define _FOSSASAT_TYPES_H

#include "FossaSat2.h"

#define FLASH_NMEA_LOG_SLOT_SIZE                        (128)
#define ADCS_NUM_AXES                                   3
#define ADCS_CALC_TYPE                                  double

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

struct gpsLogState_t {
  uint8_t buff[FLASH_NMEA_LOG_SLOT_SIZE];
  uint16_t buffPos;
  uint32_t flashPos;
  uint32_t lastFixAddr;
  bool overwrite;
  uint32_t start;
};

struct adcsBridgeState_t {
  bool outputHigh;
  uint32_t lastUpdate;
  uint32_t pulseLen;
};

struct adcsState_t {
  float prevIntensity[ADCS_NUM_AXES];
  float prevOmegaNorm;
  uint32_t start;
  bool active;
  adcsBridgeState_t bridgeStateX;
  adcsBridgeState_t bridgeStateY;
  adcsBridgeState_t bridgeStateZ;
};

struct adcsControlBits_t {
  uint8_t detumbleOnly : 1;       // LSB
  uint8_t overrideDetumbleTol : 1;
  uint8_t overrideFaultX : 1;
  uint8_t overrideFaultY : 1;
  uint8_t overrideFaultZ : 1;
};

struct adcsParams_t {
  ADCS_CALC_TYPE maxPulseInt;
  ADCS_CALC_TYPE maxPulseLen;
  ADCS_CALC_TYPE omegaTol;
  ADCS_CALC_TYPE angularTol;
  uint32_t timeStep;
  ADCS_CALC_TYPE minInertialMoment;
  ADCS_CALC_TYPE orbInclination;
  ADCS_CALC_TYPE meanOrbitalMotion;
  ADCS_CALC_TYPE pulseAmplitude;
  uint32_t detumbleLen;
  uint32_t activeLen;
  ADCS_CALC_TYPE calcTol;
  ADCS_CALC_TYPE coilChar[ADCS_NUM_AXES][ADCS_NUM_AXES];
  uint8_t position[2*ADCS_NUM_AXES];
  ADCS_CALC_TYPE eclipseThreshold;
  ADCS_CALC_TYPE rotationWeightRatio;
  ADCS_CALC_TYPE rotationTrigger;
  uint32_t bridgeTimerUpdatePeriod;
  int8_t bridgeOutputHigh;
  int8_t bridgeOutputLow;
  union {
    adcsControlBits_t bits;
    uint8_t val;
  } control;
};

#endif
