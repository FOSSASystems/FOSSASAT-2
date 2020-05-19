#ifndef _FOSSASAT_CAMERA_H
#define _FOSSASAT_CAMERA_H

#include "FossaSat2.h"

uint8_t Camera_Init(JPEG_Size pictureSize, Light_Mode lightMode, Color_Saturation saturation, Brightness brightness, Contrast contrast, Special_Effects special);
uint32_t Camera_Capture(uint8_t slot);

#endif
