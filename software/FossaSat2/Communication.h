#ifndef _FOSSASAT_COMMUNICATION_H
#define _FOSSASAT_COMMUNICATION_H

#include "FossaSat2.h"

// interrupt functions
void Communication_Receive_Interrupt();
void Communication_Change_Modem(HardwareTimer* tmr);

// modem configuration
void Communication_Set_Modem(uint8_t modem);
int16_t Communication_Set_LoRa_Configuration(float bw, uint8_t sf, uint8_t cr, uint16_t preambleLen, bool crc, int8_t power);

// system info functions
void Communication_System_Info_Add(uint8_t** buffPtr, uint8_t val, const char* name, uint32_t mult, const char* unit);
void Communication_Send_System_Info();
void Communication_Send_Morse_Beacon();

// FOSSA Communication Protocol frame handling
void Comunication_Parse_Frame(uint8_t* frame, uint8_t len);
void Communication_Execute_Function(uint8_t functionId, uint8_t* optData = nullptr, size_t optDataLen = 0);
int16_t Communication_Send_Response(uint8_t respId, uint8_t* optData = nullptr, size_t optDataLen = 0, bool encrypt = false, bool overrideModem = false);
bool Communication_Check_OptDataLen(uint8_t expected, uint8_t actual);

// radio handling
int16_t Communication_Transmit(uint8_t* data, uint8_t len, bool overrideModem = false);

#endif
