#ifndef _FOSSASAT_COMMUNICATION_H
#define _FOSSASAT_COMMUNICATION_H

#include "FossaSat2.h"

// interrupt functions
void Communication_Receive_Interrupt();

// modem configuration
int16_t Communication_Set_SpreadingFactor(uint8_t sfMode);
int16_t Communication_Set_LoRa_Configuration(float bw, uint8_t sf, uint8_t cr, uint16_t preambleLen, bool crc, int8_t power);
int16_t Communication_Set_Modem(uint8_t modem);

// CW functions
void Communication_Send_Morse_Beacon(float battVoltage);
void Communication_CW_Beep(uint32_t len);

// system info functions
void Communication_Send_Basic_System_Info();
void Communication_Send_Full_System_Info();
void Communication_Send_Statistics(uint8_t flags);
template <typename T>
void Communication_Frame_Add(uint8_t** buffPtr, T val, const char* name, uint32_t mult, const char* unit);

void Communication_Set_ADCS_Param(uint8_t** optDataPtr, uint8_t* adcsPage, uint32_t addr);
void Communication_Transfer_Picture(uint32_t imgAddress, uint32_t imgLen, uint16_t packetId);

// FOSSA Communication Protocol frame handling
void Communication_Check_New_Packet();
void Communication_Acknowledge(uint8_t functionId, uint8_t result);
void Communication_Process_Packet();
void Comunication_Parse_Frame(uint8_t* frame, uint8_t len);
void Communication_Execute_Function(uint8_t functionId, uint8_t* optData = nullptr, size_t optDataLen = 0);
int16_t Communication_Send_Response(uint8_t respId, uint8_t* optData = nullptr, size_t optDataLen = 0, bool overrideModem = false);
bool Communication_Check_OptDataLen(uint8_t expected, uint8_t actual);

// radio handling
int16_t Communication_Transmit(uint8_t* data, uint8_t len, bool overrideModem = false);

#endif
