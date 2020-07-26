/*
    FossaSat2.h
*/

#include <Wire.h>
#include <GroveMiniMoto.h>
#include "Debug.h"

/*
    Configuration.h
*/

// I2C
#define I2C1_SDA                                        PA10
#define I2C1_SCL                                        PA9

// ADCS H-bridges
#define ADCS_X_BRIDGE_ADDRESS                           0b1100100 // A1 float, A0 float
#define ADCS_Y_BRIDGE_ADDRESS                           0b1100110 // A1 float, A0 high
#define ADCS_Z_BRIDGE_ADDRESS                           0b1100101 // A1 low, A0 high

/*
    Configuration.cpp
*/

MiniMoto bridgeX(ADCS_X_BRIDGE_ADDRESS);
MiniMoto bridgeY(ADCS_Y_BRIDGE_ADDRESS);
MiniMoto bridgeZ(ADCS_Z_BRIDGE_ADDRESS);

const int speedVal = 50;

void setup() {
  FOSSASAT_DEBUG_PORT.begin(FOSSASAT_DEBUG_SPEED);
  while(!FOSSASAT_DEBUG_PORT);
  FOSSASAT_DEBUG_PORT.println();

  Wire.setSDA(I2C1_SDA);
  Wire.setSCL(I2C1_SCL);
  Wire.begin();

  FOSSASAT_DEBUG_PRINTLN();
  FOSSASAT_DEBUG_PORT.println(F("Action\t\tDevice\tFault #"));
  FOSSASAT_DEBUG_PORT.println(F("-------------------------------------------------------------"));
  
  FOSSASAT_DEBUG_PORT.print(F("Init\t"));
  FOSSASAT_DEBUG_PORT.print(F("\tX axis \t"));
  FOSSASAT_DEBUG_PORT.println(bridgeX.getFault());
  FOSSASAT_DEBUG_PORT.print(F("\t\tY axis \t"));
  FOSSASAT_DEBUG_PORT.println(bridgeY.getFault());
  FOSSASAT_DEBUG_PORT.print(F("\t\tZ axis \t"));
  FOSSASAT_DEBUG_PORT.println(bridgeZ.getFault());
  delay(1000);
}

void loop() {
  bridgeX.drive(speedVal);
  bridgeY.drive(speedVal);
  bridgeZ.drive(speedVal);
  
  FOSSASAT_DEBUG_PORT.print(F("Drive +"));
  FOSSASAT_DEBUG_PORT.print(speedVal);
  FOSSASAT_DEBUG_PORT.print(F("\t\tX axis \t"));
  FOSSASAT_DEBUG_PORT.println(bridgeX.getFault());
  FOSSASAT_DEBUG_PORT.print(F("\t\tY axis \t"));
  FOSSASAT_DEBUG_PORT.println(bridgeY.getFault());
  FOSSASAT_DEBUG_PORT.print(F("\t\tZ axis \t"));
  FOSSASAT_DEBUG_PORT.println(bridgeZ.getFault());
  delay(5000);
  
  bridgeX.stop();
  bridgeY.stop();
  bridgeZ.stop();
  
  FOSSASAT_DEBUG_PORT.print(F("Stop\t"));
  FOSSASAT_DEBUG_PORT.print(F("\tX axis \t"));
  FOSSASAT_DEBUG_PORT.println(bridgeX.getFault());
  FOSSASAT_DEBUG_PORT.print(F("\t\tY axis \t"));
  FOSSASAT_DEBUG_PORT.println(bridgeY.getFault());
  FOSSASAT_DEBUG_PORT.print(F("\t\tZ axis \t"));
  FOSSASAT_DEBUG_PORT.println(bridgeZ.getFault());
  delay(5000);

  bridgeX.drive(-1 * speedVal);
  bridgeY.drive(-1 * speedVal);
  bridgeZ.drive(-1 * speedVal);
  
  FOSSASAT_DEBUG_PORT.print(F("Drive -"));
  FOSSASAT_DEBUG_PORT.print(speedVal);
  FOSSASAT_DEBUG_PORT.print(F("\tX axis \t"));
  FOSSASAT_DEBUG_PORT.println(bridgeX.getFault());
  FOSSASAT_DEBUG_PORT.print(F("\t\tY axis \t"));
  FOSSASAT_DEBUG_PORT.println(bridgeY.getFault());
  FOSSASAT_DEBUG_PORT.print(F("\t\tZ axis \t"));
  FOSSASAT_DEBUG_PORT.println(bridgeZ.getFault());
  delay(1000);
  
  bridgeX.stop();
  bridgeY.stop();
  bridgeZ.stop();
  
  FOSSASAT_DEBUG_PORT.print(F("Stop\t"));
  FOSSASAT_DEBUG_PORT.print(F("\tX axis \t"));
  FOSSASAT_DEBUG_PORT.println(bridgeX.getFault());
  FOSSASAT_DEBUG_PORT.print(F("\t\tY axis \t"));
  FOSSASAT_DEBUG_PORT.println(bridgeY.getFault());
  FOSSASAT_DEBUG_PORT.print(F("\t\tZ axis \t"));
  FOSSASAT_DEBUG_PORT.println(bridgeZ.getFault());
  delay(5000);
}
