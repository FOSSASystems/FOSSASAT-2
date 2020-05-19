#ifndef _FOSSASAT_NAVIGATION_H
#define _FOSSASAT_NAVIGATION_H

#include "FossaSat2.h"

uint8_t Navigation_Get_EpochYear(const char* tleLine);
double Navigation_Get_EpochDay(const char* tleLine);
double Navigation_Get_BallisticCoeff(const char* tleLine);
double Navigation_Get_MeanMotion2nd(const char* tleLine);
double Navigation_Get_DragTerm(const char* tleLine);
double Navigation_Get_Inclination(const char* tleLine);
double Navigation_Get_RightAscension(const char* tleLine);
double Navigation_Get_Eccentricity(const char* tleLine);
double Navigation_Get_PerigeeArgument(const char* tleLine);
double Navigation_Get_MeanAnomaly(const char* tleLine);
double Navigation_Get_MeanMotion(const char* tleLine);
double Navigation_Get_RevolutionNumber(const char* tleLine);

#endif
