#include "Debug.h"

#define GPS_TX                                          PA0
#define GPS_RX                                          PA1
#define GPS_POWER_FET                                   PB8

HardwareSerial GpsSerial(GPS_RX, GPS_TX);

void setup() {
  pinMode(GPS_POWER_FET, OUTPUT);
  digitalWrite(GPS_POWER_FET, LOW);
  
  // initialize debug port
  FOSSASAT_DEBUG_PORT.begin(FOSSASAT_DEBUG_SPEED);
  FOSSASAT_DEBUG_PORT.println();

  // initialize UART interfaces
  GpsSerial.begin(9600);
}

void loop() {
  while(GpsSerial.available()) {
    FOSSASAT_DEBUG_PORT.write(GpsSerial.read());
  }

}
