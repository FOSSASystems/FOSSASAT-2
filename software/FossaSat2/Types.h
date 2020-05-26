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

#endif
