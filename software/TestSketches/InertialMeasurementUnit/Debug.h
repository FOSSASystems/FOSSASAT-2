#ifndef _FOSSASAT_DEBUG_H
#define _FOSSASAT_DEBUG_H

HardwareSerial debugSerial(PA3, PA2);

// uncomment to enable debug output
// RadioLib debug can be enabled in RadioLib/src/TypeDef.h
#define FOSSASAT_DEBUG

#define FOSSASAT_DEBUG_PORT   debugSerial
#define FOSSASAT_DEBUG_SPEED  9600

#ifdef FOSSASAT_DEBUG
#define FOSSASAT_DEBUG_BEGIN(...) { FOSSASAT_DEBUG_PORT.begin(__VA_ARGS__); delay(500); while(!FOSSASAT_DEBUG_PORT); }
#define FOSSASAT_DEBUG_PRINT(...) { FOSSASAT_DEBUG_PORT.print(__VA_ARGS__); }
#define FOSSASAT_DEBUG_PRINTLN(...) { FOSSASAT_DEBUG_PORT.println(__VA_ARGS__); }
#define FOSSASAT_DEBUG_WRITE(...) { FOSSASAT_DEBUG_PORT.write(__VA_ARGS__); }
#define FOSSASAT_DEBUG_PRINT_BUFF(BUFF, LEN) { \
    for(size_t i = 0; i < LEN; i++) { \
      FOSSASAT_DEBUG_PORT.print(F("0x")); \
      FOSSASAT_DEBUG_PORT.print(BUFF[i], HEX); \
      FOSSASAT_DEBUG_PORT.print('\t'); \
      FOSSASAT_DEBUG_PORT.write(BUFF[i]); \
      FOSSASAT_DEBUG_PORT.println(); \
    } }
#define FOSSASAT_DEBUG_PRINT_FLASH(ADDR, LEN) { \
    uint8_t readBuff[FLASH_EXT_PAGE_SIZE]; \
    PersistentStorage_Read(ADDR, readBuff, LEN); \
    char buff[16]; \
    if(LEN < 16) { \
      for(uint8_t i = 0; i < LEN; i++) { \
        sprintf(buff, "%02x ", readBuff[i]); \
        FOSSASAT_DEBUG_PORT.print(buff); \
      } \
      FOSSASAT_DEBUG_PORT.println(); \
    } else { \
      for(size_t i = 0; i < LEN/16; i++) { \
        for(uint8_t j = 0; j < 16; j++) { \
          sprintf(buff, "%02x ", readBuff[i*16 + j]); \
          FOSSASAT_DEBUG_PORT.print(buff); \
        } \
        FOSSASAT_DEBUG_PORT.println(); \
      } \
    } }
#define FOSSASAT_DEBUG_DELAY(MS) { delay(MS); }
#define FOSSASAT_DEBUG_STOPWATCH_INIT_H extern uint32_t fsdbgStart;
#define FOSSASAT_DEBUG_STOPWATCH_INIT_CPP uint32_t fsdbgStart = 0;
#define FOSSASAT_DEBUG_STOPWATCH_START() { fsdbgStart = millis(); }
#define FOSSASAT_DEBUG_STOPWATCH_STOP() { \
    FOSSASAT_DEBUG_PORT.print(F("Elapsed: ")); \
    FOSSASAT_DEBUG_PORT.print(millis() - fsdbgStart); \
    FOSSASAT_DEBUG_PORT.println(F(" ms")); \
  }
#else
#define FOSSASAT_DEBUG_BEGIN(...) {}
#define FOSSASAT_DEBUG_PRINT(...) {}
#define FOSSASAT_DEBUG_PRINTLN(...) {}
#define FOSSASAT_DEBUG_WRITE(...) {}
#define FOSSASAT_DEBUG_PRINT_BUFF(BUFF, LEN) {}
#define FOSSASAT_DEBUG_PRINT_FLASH(ADDR, LEN) {}
#define FOSSASAT_DEBUG_STOPWATCH_INIT_H
#define FOSSASAT_DEBUG_STOPWATCH_INIT_CPP
#define FOSSASAT_DEBUG_STOPWATCH_START(...) {}
#define FOSSASAT_DEBUG_STOPWATCH_STOP(...) {}
#define FOSSASAT_DEBUG_DELAY(MS) {}
#endif

#endif
