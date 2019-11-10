#include "PersistentStorage.h"

// EEPROM only included once to supress unused variable warning
#include <EEPROM.h>

// internal EEPROM reading template
template <class T>
T PersistentStorage_Read_Internal(uint16_t addr) {
  T val;
  EEPROM.get(addr, val);
  return (val);
}

// external Flash reading template
template <class T>
T PersistentStorage_Read_External(uint16_t addr) {

}

// internal EEPROM writing template
template <class T>
void PersistentStorage_Write_Internal(uint16_t addr, T val) {
  EEPROM.put(addr, val);
}

// external Flash writing template
template <class T>
void PersistentStorage_Write_External(uint16_t addr, T val) {

}

void PersistentStorage_Wipe_Internal() {
  // wipe data, use buffered access
  FOSSASAT_DEBUG_PRINT(F("Internal wipe, "));
  FOSSASAT_DEBUG_PRINT(EEPROM_USED_SIZE);
  FOSSASAT_DEBUG_PRINT(F(" bytes ... "));
  for (uint16_t i = 0; i < EEPROM_USED_SIZE; i++) {
    eeprom_buffered_write_byte(i, EEPROM_RESET_VALUE);
  }
  eeprom_buffer_flush();
  FOSSASAT_DEBUG_PRINTLN(F("done!"));
  FOSSASAT_DEBUG_PRINT(F("Setting default variables ... "));


  // set reset counter to 0
  PersistentStorage_Write_Internal<uint16_t>(EEPROM_RESTART_COUNTER_ADDR, 0);

  // set deployment counter to 0
  PersistentStorage_Write_Internal<uint8_t>(EEPROM_DEPLOYMENT_COUNTER_ADDR, 0);

  // set default callsign
  PersistentStorage_Set_Callsign((char*)CALLSIGN_DEFAULT);

  // set default power configuration
  PersistentStorage_Write_Internal<uint8_t>(EEPROM_TRANSMISSIONS_ENABLED, 1);

  FOSSASAT_DEBUG_PRINTLN(F("done!"));
}

void PersistentStorage_Wipe_External() {

}

void PersistentStorage_Set_Callsign(char* newCallsign) {
  // get length of the new callsign (callsign is saved to EEPROM including terminating NULL)
  uint8_t newCallsignLen = (uint8_t)strlen(newCallsign) + 1;

  // check new callsign length
  if (newCallsignLen > MAX_STRING_LENGTH) {
    FOSSASAT_DEBUG_PRINTLN(F("New callsign too long!"));
    return;
  }

  // write new callsign length
  PersistentStorage_Write_Internal<uint8_t>(EEPROM_CALLSIGN_LEN_ADDR, newCallsignLen);

  // write new callsign (including terminating NULL)
  for (uint8_t i = 0; i < newCallsignLen; i++) {
    PersistentStorage_Write_Internal<uint8_t>(EEPROM_CALLSIGN_ADDR + i, (uint8_t)newCallsign[i]);
  }
}

void PersistentStorage_Get_Callsign(char* buff, uint8_t len) {
  // get callsign from EEPROM (terminating NULL is stored as well)
  for (uint8_t i = 0; i < len; i++) {
    buff[i] = (char)PersistentStorage_Read_Internal<uint8_t>(EEPROM_CALLSIGN_ADDR + i);
  }
}

// explicitly instantiate templates
template uint8_t PersistentStorage_Read_Internal<uint8_t>(uint16_t);
template uint8_t PersistentStorage_Read_External<uint8_t>(uint16_t);
template uint16_t PersistentStorage_Read_Internal<uint16_t>(uint16_t);
template uint16_t PersistentStorage_Read_External<uint16_t>(uint16_t);
template void PersistentStorage_Write_Internal<uint8_t>(uint16_t, uint8_t);
template void PersistentStorage_Write_External<uint8_t>(uint16_t, uint8_t);
template void PersistentStorage_Write_Internal<uint16_t>(uint16_t, uint16_t);
template void PersistentStorage_Write_External<uint16_t>(uint16_t, uint16_t);
