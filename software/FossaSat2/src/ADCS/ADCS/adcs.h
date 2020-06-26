/* Project: FossaSat-2 Flight Software
   Author: Team Steiner
   Module: adcs
   File: adcs.h
   04/30/20

   This file drives the function declaration file
*/

#ifndef ADCS_H_INCLUDED
#define ADCS_H_INCLUDED

/***************** Headers ****************/
#include <math.h>

#include "../../../FossaSat2.h"

#define ADCS_CALC_TYPE                                  double
#define ADCS_NUM_AXES                                   3

#define ADCS_RES_DONE_TOL_REACHED                       0x00
#define ADCS_RES_DONE_TIME_LIMIT                        0x01
#define ADCS_RES_STOPPED_LOW_POWER                      0x02
#define ADCS_RES_STOPPED_FAULT_X                        0x03
#define ADCS_RES_STOPPED_FAULT_Y                        0x04
#define ADCS_RES_STOPPED_FAULT_Z                        0x05
#define ADCS_RES_STOPPED_ABORT                          0x06
#define ADCS_RUNNING                                    0x07

#include "../ACS/acs.h"
#include "../ADS/ads.h"

// TODO no global vars inside aux functions
// TODO extract common code
// TODO specify array dimensions in declarations

/*********** Functions declaration ************/
// Main structure
void ADCS_Main(const uint8_t controlFlags, const uint32_t detumbleDuration, const uint32_t activeDuration,
               const ADCS_CALC_TYPE orbitalInclination, const ADCS_CALC_TYPE meanOrbitalMotion);

ADCS_CALC_TYPE ADCS_VectorNorm(const ADCS_CALC_TYPE dim[]);

void ADCS_Detumble_Init(const uint32_t detumbleDuration, const ADCS_CALC_TYPE orbitalInclination, const ADCS_CALC_TYPE meanOrbitalMotion);
void ADCS_Detumble_Update();
void ADCS_Detumble_Finish(uint8_t result, bool startActiveControl);

void ADCS_ActiveControl_Init(const uint32_t activeDuration);
void ADCS_ActiveControl_Update();

uint8_t ADCS_Load_Ephemerides(uint32_t row, ADCS_CALC_TYPE solarEph[], ADCS_CALC_TYPE magEph[]);
bool ADCS_Check();
void ADCS_Finish(uint8_t result);
void ADCS_Update_Bridges();

#endif // ADCS_H_INCLUDED
