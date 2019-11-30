#include <SoftwareSerial.h>

#define STMSERIAL_RX    9
#define STMSERIAL_TX    8
#define WATCHDOG_IN     7
#define PORT_SPEED      9600

SoftwareSerial stmSerial(STMSERIAL_RX, STMSERIAL_TX);

uint32_t lastHeartbeat = 0;

void setup() {
  Serial.begin(PORT_SPEED);
  stmSerial.begin(PORT_SPEED);
  pinMode(WATCHDOG_IN, OUTPUT);
}

void loop() {
  if(stmSerial.available()) {
    Serial.write(stmSerial.read());
  }

  if(Serial.available()) {
    stmSerial.write(Serial.read());
  }

  if(millis() - lastHeartbeat > 1000) {
    lastHeartbeat = millis();
    digitalWrite(WATCHDOG_IN, !digitalRead(WATCHDOG_IN));
  }
}
