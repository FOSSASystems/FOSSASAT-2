#ifndef _FOSSASAT_NAVIGATION_H
#define _FOSSASAT_NAVIGATION_H

#include "FossaSat2.h"

#define SKYTRAQ_START_OF_SEQ_1          0xA0
#define SKYTRAQ_START_OF_SEQ_2          0xA1
#define SKYTRAQ_END_OF_SEQ_1            0x0D
#define SKYTRAQ_END_OF_SEQ_2            0x0A
#define SKYTRAQ_ACK                     0x83
#define SKYTRAQ_NACK                    0x84

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

uint16_t Navigation_GNSS_Run_Cmd(uint8_t* payload, uint16_t payloadLen, uint8_t* resp, uint32_t timeout = 3000);
bool Navigation_GNSS_Send_Cmd(uint8_t* payload, uint16_t payloadLen);
uint16_t Navigation_GNSS_Get_Resp(uint8_t* resp, uint32_t timeout = 3000);

void Navigation_GNSS_Wipe_Log();
void Navigation_GNSS_Setup_Logging();
void Navigation_GNSS_SerialEvent();
uint32_t Navigation_GNSS_Finish_Logging();

#endif
