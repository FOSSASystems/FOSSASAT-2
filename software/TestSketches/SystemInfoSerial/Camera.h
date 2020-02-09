#ifndef _FOSSASAT_CAMERA_H
#define _FOSSASAT_CAMERA_H

#include "FossaSat2.h"

uint8_t Camera_Init(uint8_t pictureSize, uint8_t lightMode, uint8_t saturation, uint8_t brightness, uint8_t contrast, uint8_t special);
uint32_t Camera_Capture(uint8_t slot);

#endif
