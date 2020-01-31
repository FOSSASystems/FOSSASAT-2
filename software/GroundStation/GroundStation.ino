/*
   FOSSA Ground Station Example

   Tested on Arduino Uno and SX1268, can be used with any LoRa radio
   from the SX127x or SX126x series. Make sure radio type (line 23)
   and pin mapping (lines 26 - 29) match your hardware!

   References:

   RadioLib error codes:
   https://jgromes.github.io/RadioLib/group__status__codes.html

   FOSSASAT-1B Communication Guide:

*/

// include all libraries
#include <RadioLib.h>
#include <FOSSA-Comms.h>

//#define USE_GFSK                    // uncomment to use GFSK
#define USE_SX126X                    // uncomment to use SX126x

// pin definitions
#define CS                    10      // SPI chip select
#define DIO                   2       // DIO0 for SX127x, DIO1 for SX126x
#define NRST                  NC      // NRST pin (optional)
#define BUSY                  9       // BUSY pin (SX126x-only)

// modem configuration
#define FREQUENCY             436.7   // MHz
#define BANDWIDTH             125.0   // kHz
#define SPREADING_FACTOR      11      // -
#define CODING_RATE           8       // 4/8
#define SYNC_WORD             0x12    // used as LoRa "sync word", or twice repeated as FSK sync word (0x1212)
#define OUTPUT_POWER          20      // dBm
#define CURRENT_LIMIT         140     // mA
#define LORA_PREAMBLE_LEN     8       // symbols
#define BIT_RATE              9.6     // kbps
#define FREQ_DEV              5.0     // kHz SSB
#define RX_BANDWIDTH          39.0    // kHz SSB
#define FSK_PREAMBLE_LEN      16      // bits
#define DATA_SHAPING          0.5     // BT product
#define TCXO_VOLTAGE          1.6

// camera configuration macros, from ArduCAM.h

// size
#define OV2640_160x120        0
#define OV2640_176x144        1
#define OV2640_320x240        2
#define OV2640_352x288        3
#define OV2640_640x480        4
#define OV2640_800x600        5
#define OV2640_1024x768       6
#define OV2640_1280x1024      7
#define OV2640_1600x1200      8

// light mode
#define Auto                  0
#define Sunny                 1
#define Cloudy                2
#define Office                3
#define Home                  4

// saturation
#define Saturation2           2
#define Saturation1           3
#define Saturation0           4
#define Saturation_1          5
#define Saturation_2          6

// brightness
#define Brightness2           2
#define Brightness1           3
#define Brightness0           4
#define Brightness_1          5
#define Brightness_2          6

// contrast
#define Contrast2             2
#define Contrast1             3
#define Contrast0             4
#define Contrast_1            5
#define Contrast_2            6

// special effects
#define Antique               0
#define Bluish                1
#define Greenish              2
#define Reddish               3
#define BW                    4
#define Negative              5
#define BWnegative            6
#define Normal                7

// set up radio module
#ifdef USE_SX126X
SX1268 radio = new Module(CS, DIO, NRST, BUSY);
#else
SX1278 radio = new Module(CS, DIO, NRST, NC);
#endif

// flags
volatile bool interruptEnabled = true;
volatile bool transmissionReceived = false;

// satellite callsign
char callsign[] = "FOSSASAT-2";

// transmission password
const char* password = "password";

// encryption key
const uint8_t encryptionKey[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                                 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x00};

// radio ISR
void onInterrupt() {
  if (!interruptEnabled) {
    return;
  }

  transmissionReceived = true;
}

void sendFrame(uint8_t functionId, uint8_t optDataLen = 0, uint8_t* optData = NULL) {
  // build frame
  uint8_t len = FCP_Get_Frame_Length(callsign, optDataLen);
  uint8_t* frame = new uint8_t[len];
  FCP_Encode(frame, callsign, functionId, optDataLen, optData);

  // send data
  int state = radio.transmit(frame, len);
  delete[] frame;

  // check transmission success
  if (state == ERR_NONE) {
    Serial.println(F("sent successfully!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
  }
}

void sendFrameEncrypted(uint8_t functionId, uint8_t optDataLen = 0, uint8_t* optData = NULL) {
  // build frame
  uint8_t len = FCP_Get_Frame_Length(callsign, optDataLen, password);
  uint8_t* frame = new uint8_t[len];
  FCP_Encode(frame, callsign, functionId, optDataLen, optData, encryptionKey, password);

  // send data
  int state = radio.transmit(frame, len);
  delete[] frame;
  
  // check transmission success
  if (state == ERR_NONE) {
    Serial.println(F("sent successfully!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
  }
}

// function to print controls
void printControls() {
  Serial.println(F("------------- Controls -------------"));
  Serial.println(F("p - send ping frame"));
  Serial.println(F("i - request satellite info"));
  Serial.println(F("l - request last packet info"));
  Serial.println(F("r - send message to be retransmitted"));
  Serial.println(F("d - deploy"));
  Serial.println(F("w - disable low power mode"));
  Serial.println(F("W - enable low power mode"));
  Serial.println(F("m - disable MPPT keep alive"));
  Serial.println(F("M - enable MPPT keep alive"));
  Serial.println(F("t - restart"));
  Serial.println(F("f - wipe flash"));
  Serial.println(F("L - set Rx window lengths"));
  Serial.println(F("R - retransmit custom"));
  Serial.println(F("o - get rotation data"));
  Serial.println(F("u - send packet with unknown function ID"));
  Serial.println(F("s - get stats"));
  Serial.println(F("c - capture photo"));
  Serial.println(F("e - set power limits"));
  Serial.println(F("T - set RTC"));
  Serial.println(F("------------------------------------"));
}

void decode(uint8_t* respFrame, uint8_t respLen) {
  // print raw data
  Serial.print(F("Received "));
  Serial.print(respLen);
  Serial.println(F(" bytes:"));
  PRINT_BUFF(respFrame, respLen);

  // print packet info
  Serial.print(F("RSSI: "));
  Serial.print(radio.getRSSI());
  Serial.println(F(" dBm"));
  Serial.print(F("SNR: "));
  Serial.print(radio.getSNR());
  Serial.println(F(" dB"));

  // get function ID
  uint8_t functionId = FCP_Get_FunctionID(callsign, respFrame, respLen);
  Serial.print(F("Function ID: 0x"));
  Serial.println(functionId, HEX);

  // check optional data
  uint8_t* respOptData = nullptr;
  uint8_t respOptDataLen = 0;
  if (functionId < PRIVATE_OFFSET) {
    // public frame
    respOptDataLen = FCP_Get_OptData_Length(callsign, respFrame, respLen);
  } else {
    // private frame
    respOptDataLen = FCP_Get_OptData_Length(callsign, respFrame, respLen, encryptionKey, password);
  }
  Serial.print(F("Optional data ("));
  Serial.print(respOptDataLen);
  Serial.println(F(" bytes):"));
  if (respOptDataLen > 0) {
    // read optional data
    respOptData = new uint8_t[respOptDataLen];
    if (functionId < PRIVATE_OFFSET) {
      // public frame
      FCP_Get_OptData(callsign, respFrame, respLen, respOptData);
    } else {
      // private frame
      FCP_Get_OptData(callsign, respFrame, respLen, respOptData, encryptionKey, password);
    }
    PRINT_BUFF(respOptData, respOptDataLen);
  }

  // process received frame
  switch (functionId) {
    case RESP_PONG:
      Serial.println(F("Pong!"));
      break;

    case RESP_SYSTEM_INFO:
      Serial.println(F("System info:"));

      Serial.print(F("batteryVoltage = "));
      Serial.print(FCP_Get_Battery_Voltage(respOptData));
      Serial.println(" V");

      Serial.print(F("batteryChargingCurrent = "));
      Serial.print(FCP_Get_Battery_Charging_Current(respOptData), 4);
      Serial.println(" mA");

      uint32_t onboardTime = 0;
      memcpy(&onboardTime, respOptData + 3, sizeof(uint32_t));
      Serial.print(F("onboardTime = "));
      Serial.println(onboardTime);

      uint8_t powerConfig = 0;
      memcpy(&powerConfig, respOptData + 7, sizeof(uint8_t));
      Serial.print(F("powerConfig = 0b"));
      Serial.println(powerConfig, BIN);

      uint16_t resetCounter = 0;
      memcpy(&resetCounter, respOptData + 8, sizeof(uint16_t));
      Serial.print(F("resetCounter = "));
      Serial.println(resetCounter);

      Serial.print(F("voltageXA = "));
      Serial.print(FCP_System_Info_Get_Voltage(respOptData, 10));
      Serial.println(" V");

      Serial.print(F("voltageXB = "));
      Serial.print(FCP_System_Info_Get_Voltage(respOptData, 11));
      Serial.println(" V");

      Serial.print(F("voltageZA = "));
      Serial.print(FCP_System_Info_Get_Voltage(respOptData, 12));
      Serial.println(" V");

      Serial.print(F("voltageZB = "));
      Serial.print(FCP_System_Info_Get_Voltage(respOptData, 13));
      Serial.println(" V");

      Serial.print(F("voltageY = "));
      Serial.print(FCP_System_Info_Get_Voltage(respOptData, 14));
      Serial.println(" V");

      Serial.print(F("batteryTemp = "));
      Serial.print(FCP_System_Info_Get_Temperature(respOptData, 15));
      Serial.println(" deg C");

      Serial.print(F("boardTemp = "));
      Serial.print(FCP_System_Info_Get_Temperature(respOptData, 17));
      Serial.println(" deg C");
      
      break;

    case RESP_PACKET_INFO: {
      Serial.println(F("Packet info:"));

      Serial.print(F("SNR = "));
      Serial.print(respOptData[0] / 4.0);
      Serial.println(F(" dB"));

      Serial.print(F("RSSI = "));
      Serial.print(respOptData[1] / -2.0);
      Serial.println(F(" dBm"));

      uint16_t counter = 0;
      Serial.print(F("valid LoRa frames = "));
      memcpy(&counter, respOptData + 2, sizeof(uint16_t));
      Serial.println(counter);
      
      Serial.print(F("invalid LoRa frames = "));
      memcpy(&counter, respOptData + 4, sizeof(uint16_t));
      Serial.println(counter);
      
      Serial.print(F("valid FSK frames = "));
      memcpy(&counter, respOptData + 6, sizeof(uint16_t));
      Serial.println(counter);
      
      Serial.print(F("invalid FSK frames = "));
      memcpy(&counter, respOptData + 8, sizeof(uint16_t));
      Serial.println(counter);
    } break;

    case RESP_REPEATED_MESSAGE:
      Serial.println(F("Got repeated message:"));
      for (uint8_t i = 0; i < respOptDataLen; i++) {
        Serial.write(respOptData[i]);
      }
      Serial.println();
      break;

    case RESP_DEPLOYMENT_STATE:
      Serial.println(F("Got deployment counter:"));
      Serial.println(respOptData[0]);
      break;

    case RESP_STATISTICS: {
      Serial.println(F("Got stats:\t\tunit\tmin\tavg\tmax"));
      uint8_t flags = respOptData[0];
      uint8_t pos = 1;
      if(flags & 0x01) {
        // charging voltage
        Serial.print(F("batteryChargingVoltage\t[V]"));
        Serial.print('\t');
        Serial.print(FCP_System_Info_Get_Voltage(respOptData, pos));
        Serial.print('\t');
        Serial.print(FCP_System_Info_Get_Voltage(respOptData, pos + 1));
        Serial.print('\t');
        Serial.println(FCP_System_Info_Get_Voltage(respOptData, pos + 2));
        pos += 3;
      }

      if(flags& 0x02) {
        // charging current
        Serial.print(F("batteryChargingCurrent\t[mA]"));
        Serial.print('\t');
        Serial.print(FCP_System_Info_Get_Current(respOptData, pos));
        Serial.print('\t');
        Serial.print(FCP_System_Info_Get_Current(respOptData, pos + 2));
        Serial.print('\t');
        Serial.println(FCP_System_Info_Get_Current(respOptData, pos + 4));
        pos += 6;
      }

      if(flags & 0x04) {
        // battery voltage
        Serial.print(F("batteryVoltage\t\t[V]"));
        Serial.print('\t');
        Serial.print(FCP_System_Info_Get_Voltage(respOptData, pos));
        Serial.print('\t');
        Serial.print(FCP_System_Info_Get_Voltage(respOptData, pos + 1));
        Serial.print('\t');
        Serial.println(FCP_System_Info_Get_Voltage(respOptData, pos + 2));
        pos += 3;
      }

      if(flags & 0x08) {
        // cell A voltage
        Serial.print(F("solarCellAVoltage\t[V]"));
        Serial.print('\t');
        Serial.print(FCP_System_Info_Get_Voltage(respOptData, pos));
        Serial.print('\t');
        Serial.print(FCP_System_Info_Get_Voltage(respOptData, pos + 1));
        Serial.print('\t');
        Serial.println(FCP_System_Info_Get_Voltage(respOptData, pos + 2));
        pos += 3;
      }

      if(flags & 0x10) {
        // cell B voltage
        Serial.print(F("solarCellBVoltage\t[V]"));
        Serial.print('\t');
        Serial.print(FCP_System_Info_Get_Voltage(respOptData, pos));
        Serial.print('\t');
        Serial.print(FCP_System_Info_Get_Voltage(respOptData, pos + 1));
        Serial.print('\t');
        Serial.println(FCP_System_Info_Get_Voltage(respOptData, pos + 2));
        pos += 3;
      }

      if(flags & 0x20) {
        // cell C voltage
        Serial.print(F("solarCellCVoltage\t[V]"));
        Serial.print('\t');
        Serial.print(FCP_System_Info_Get_Voltage(respOptData, pos));
        Serial.print('\t');
        Serial.print(FCP_System_Info_Get_Voltage(respOptData, pos + 1));
        Serial.print('\t');
        Serial.println(FCP_System_Info_Get_Voltage(respOptData, pos + 2));
        pos += 3;
      }

      if(flags & 0x40) {
        // battery temperature
        Serial.print(F("batteryTemperature\t[deg C]"));
        Serial.print('\t');
        Serial.print(FCP_System_Info_Get_Temperature(respOptData, pos));
        Serial.print('\t');
        Serial.print(FCP_System_Info_Get_Temperature(respOptData, pos + 2));
        Serial.print('\t');
        Serial.println(FCP_System_Info_Get_Temperature(respOptData, pos + 4));
        pos += 6;
      }

      if(flags & 0x80) {
        // board temperature
        Serial.print(F("boardTemperature\t[deg C]"));
        Serial.print('\t');
        Serial.print(FCP_System_Info_Get_Temperature(respOptData, pos));
        Serial.print('\t');
        Serial.print(FCP_System_Info_Get_Temperature(respOptData, pos + 2));
        Serial.print('\t');
        Serial.println(FCP_System_Info_Get_Temperature(respOptData, pos + 4));
        pos += 6;
      }
    } break;

    case RESP_RECORDED_SOLAR_CELLS:
      Serial.println(F("Got recorded cells:"));
      Serial.println(F("A\tB\tC"));
      for(uint8_t i = 0; i < respOptDataLen; i += 3) {
        Serial.print(respOptData[i]);
        Serial.print('\t');
        Serial.print(respOptData[i+1]);
        Serial.print('\t');
        Serial.println(respOptData[i+2]);
      }
      break;

    default:
      Serial.println(F("Unknown function ID!"));
      break;
  }

  printControls();
  if (respOptDataLen > 0) {
    delete[] respOptData;
  }
}

void getResponse(uint32_t timeout) {
  uint32_t start = millis();
  while (millis() - start <= timeout) {
    if (transmissionReceived) {
      // disable reception interrupt
      interruptEnabled = false;
      transmissionReceived = false;

      // read received data
      size_t respLen = radio.getPacketLength();
      uint8_t* respFrame = new uint8_t[respLen];
      int state = radio.readData(respFrame, respLen);

      if (state == ERR_NONE) {
        decode(respFrame, respLen);
      } else {
        Serial.print(F("Error, code "));
        Serial.println(state);
      }

      delete[] respFrame;

      // enable reception interrupt
      radio.startReceive();
      interruptEnabled = true;
    }
  }
}

void restart() {
  Serial.print(F("Sending restart request ... "));

  // send the frame
  sendFrameEncrypted(CMD_RESTART);
}

void wipe(uint8_t flags) {
  Serial.print(F("Sending wipe request ... "));

  // send the frame
  uint8_t optData[] = {flags};
  sendFrameEncrypted(CMD_WIPE_EEPROM, 1, optData);
}

void setLowPowerMode(uint8_t en) {
  Serial.print(F("Sending low power mode change request ... "));

  // send the frame
  uint8_t optData[] = {en};
  sendFrameEncrypted(CMD_SET_LOW_POWER_ENABLE, 1, optData);
}

void setMPPTKeepAlive(uint8_t en) {
  Serial.print(F("Sending MPPT mode change request ... "));

  // send the frame
  uint8_t optData[] = {0x01, en};
  sendFrameEncrypted(CMD_SET_MPPT_MODE, 2, optData);
}

void deploy() {
  Serial.print(F("Sending deployment request ... "));

  // send the frame
  sendFrameEncrypted(CMD_DEPLOY);
}

void sendPing() {
  Serial.print(F("Sending ping frame ... "));

  // send the frame
  sendFrame(CMD_PING);
}

void requestInfo() {
  Serial.print(F("Requesting system info ... "));

  // send the frame
  sendFrame(CMD_TRANSMIT_SYSTEM_INFO);
}

void requestPacketInfo() {
  Serial.print(F("Requesting last packet info ... "));

  // send the frame
  sendFrame(CMD_GET_PACKET_INFO);
}

void requestRetransmit() {
  Serial.println(F("Enter message to be sent:"));
  Serial.println(F("(max 32 characters, end with LF or CR+LF)"));

  // get data to be retransmited
  char optData[32];
  uint8_t bufferPos = 0;
  while (bufferPos < 32) {
    while (!Serial.available());
    char c = Serial.read();
    Serial.print(c);
    if ((c != '\r') && (c != '\n')) {
      optData[bufferPos] = c;
      bufferPos++;
    } else {
      break;
    }
  }

  // wait for a bit to receive any trailing characters
  delay(100);

  // dump the serial buffer
  while (Serial.available()) {
    Serial.read();
  }

  Serial.println();
  Serial.print(F("Requesting retransmission ... "));

  // send the frame
  optData[bufferPos] = '\0';
  uint8_t optDataLen = strlen(optData);
  sendFrame(CMD_RETRANSMIT, optDataLen, (uint8_t*)optData);
}

void requestRetransmitCustom() {
  Serial.println(F("Enter message to be sent:"));
  Serial.println(F("(max 32 characters, end with LF or CR+LF)"));

  // get data to be retransmited
  uint8_t optData[32 + 7];
  optData[0] = 0x07;
  optData[1] = 0x06;
  optData[2] = 0x08;
  optData[3] = 0x08;
  optData[4] = 0x00;
  optData[5] = 0x01;
  optData[6] = 20;
  uint8_t bufferPos = 7;
  while (bufferPos < 32 + 7) {
    while (!Serial.available());
    char c = Serial.read();
    Serial.print(c);
    if ((c != '\r') && (c != '\n')) {
      optData[bufferPos] = (uint8_t)c;
      bufferPos++;
    } else {
      break;
    }
  }

  // wait for a bit to receive any trailing characters
  delay(100);

  // dump the serial buffer
  while (Serial.available()) {
    Serial.read();
  }

  Serial.println();
  Serial.print(F("Requesting retransmission ... "));

  // send the frame
  uint8_t optDataLen = bufferPos - 1;
  sendFrame(CMD_RETRANSMIT_CUSTOM, optDataLen, optData);
}

int16_t setLoRa() {
  int state = radio.begin(FREQUENCY,
                          BANDWIDTH,
                          SPREADING_FACTOR,
                          CODING_RATE,
                          SYNC_WORD,
                          OUTPUT_POWER,
                          CURRENT_LIMIT,
                          LORA_PREAMBLE_LEN,
                          TCXO_VOLTAGE);
  radio.setCRC(true);
  return(state);
}

int16_t setGFSK() {
  int state = radio.beginFSK(FREQUENCY,
                             BIT_RATE,
                             FREQ_DEV,
                             RX_BANDWIDTH,
                             OUTPUT_POWER,
                             CURRENT_LIMIT,
                             FSK_PREAMBLE_LEN,
                             DATA_SHAPING,
                             TCXO_VOLTAGE);
  uint8_t syncWordFSK[2] = {SYNC_WORD, SYNC_WORD};
  radio.setSyncWord(syncWordFSK, 2);
  #ifdef USE_SX126X
    radio.setCRC(2);
  #else
    radio.setCRC(true);
  #endif
  return (state);
}

void setRxWindows(uint8_t fsk, uint8_t lora) {
  Serial.print(F("Sending RX window change request ... "));

  // send the frame
  uint8_t optData[] = {fsk, lora};
  sendFrameEncrypted(CMD_SET_RECEIVE_WINDOWS, 2, optData);
}

void sendUnknownFrame() {
  radio.implicitHeader(strlen(callsign) + 1);
  sendPing();
  radio.explicitHeader();
}

void getStats(uint8_t mask) {
  Serial.print(F("Sending stats request ... "));
  sendFrame(CMD_GET_STATISTICS, 1, &mask);
}

void recordIMU(uint8_t samples, uint16_t period) {
  Serial.print(F("Sending record IMU request ... "));
  uint8_t optData[3];
  optData[0] = samples;
  memcpy(optData + 1, &period, 2);
  sendFrameEncrypted(CMD_RECORD_IMU, 3, optData);
}

void cameraCapture(uint8_t slot, uint8_t pictureSize, uint8_t lightMode, uint8_t saturation, uint8_t brightness, uint8_t contrast, uint8_t special) {
  Serial.print(F("Sending capture request ... "));
  uint8_t optData[4] = {slot, ((pictureSize << 4) & 0xF0) | (lightMode & 0x0F),
                         ((saturation << 4) & 0xF0) | (brightness & 0x0F),
                         ((contrast << 4) & 0xF0) | (special & 0x0F)};
  sendFrameEncrypted(CMD_CAMERA_CAPTURE, 4, optData);
}

void setPowerLimits(int16_t deploymentVoltageLimit, int16_t heaterBatteryLimit, int16_t cwBeepLimit, int16_t lowPowerLimit, float heaterTempLimit, float mpptTempLimit, uint8_t heaterDutyCycle) {
  Serial.print(F("Sending power limits change request ... "));
  uint8_t optData[17];
  memcpy(optData, &deploymentVoltageLimit, sizeof(int16_t));
  memcpy(optData + sizeof(int16_t), &heaterBatteryLimit, sizeof(int16_t));
  memcpy(optData + 2*sizeof(int16_t), &cwBeepLimit, sizeof(int16_t));
  memcpy(optData + 3*sizeof(int16_t), &lowPowerLimit, sizeof(int16_t));
  memcpy(optData + 4*sizeof(int16_t), &heaterTempLimit, sizeof(float));
  memcpy(optData + 4*sizeof(int16_t) + sizeof(float), &mpptTempLimit, sizeof(float));
  memcpy(optData + 4*sizeof(int16_t) + 2*sizeof(float), &heaterDutyCycle, sizeof(uint8_t));
  sendFrameEncrypted(CMD_SET_POWER_LIMITS, 17, optData);
}

void setRTC(uint8_t year, uint8_t month, uint8_t day, uint8_t weekDay, uint8_t hours, uint8_t minutes, uint8_t seconds) {
  Serial.print(F("Sending RTC change request ... "));
  uint8_t optData[7] = {year, month, day, weekDay, hours, minutes, seconds};
  sendFrameEncrypted(CMD_SET_RTC, 7, optData);
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("FOSSASAT-2 Ground Station Demo Code"));

  // initialize the radio
  #ifdef USE_GFSK
    int state = setGFSK();
  #else
    int state = setLoRa();
  #endif

  if (state == ERR_NONE) {
    Serial.println(F("Radio initialization successful!"));
  } else {
    Serial.print(F("Failed to initialize radio, code: "));
    Serial.println(state);
    while (true);
  }

  #ifdef USE_SX126X
    radio.setDio1Action(onInterrupt);
  #else
    radio.setDio0Action(onInterrupt);
  #endif
  
  // begin listening for packets
  radio.startReceive();

  // provide seed for PRNG
  randomSeed(analogRead(A6));

  printControls();
}

void loop() {
  // check serial data
  if (Serial.available()) {
    // disable reception interrupt
    interruptEnabled = false;
    #ifdef USE_SX126X
      radio.clearDio1Action();
    #else
      radio.clearDio0Action();
    #endif

    // get the first character
    char serialCmd = Serial.read();

    // wait for a bit to receive any trailing characters
    delay(50);

    // dump the serial buffer
    while (Serial.available()) {
      Serial.read();
    }

    // process serial command
    switch (serialCmd) {
      case 'p':
        sendPing();
        break;
      case 'i':
        requestInfo();
        break;
      case 'l':
        requestPacketInfo();
        break;
      case 'r':
        requestRetransmit();
        break;
      case 'd':
        deploy();
        break;
      case 'w':
        setLowPowerMode(0x00);
        break;
      case 'W':
        setLowPowerMode(0x01);
        break;
      case 'm':
        setMPPTKeepAlive(0x00);
        break;
      case 'M':
        setMPPTKeepAlive(0x01);
        break;
      case 't':
        restart();
        break;
      case 'f':
        wipe(0xFF);
        break;
      case 'L':
        setRxWindows(20, 20);
        break;
      case 'R':
        requestRetransmitCustom();
        break;
      case 'o':
        recordIMU(40, 1000);
        break;
      case 'u':
        Serial.print(F("Sending unknown frame ... "));
        sendFrame(0xFF);
        break;
      case 's':
        getStats(0b00001111);
        break;
      case 'c':
        cameraCapture(0, OV2640_320x240, Auto, Saturation0, Brightness0, Contrast0, Normal);
        break;
      case 'e':
        setPowerLimits(3600, 3700, 3750, 3900, 4.2, -1.5, 126);
        break;
      case 'T':
        setRTC(21, 4, 2, 5, 20, 10, 50);
        break;
      default:
        Serial.print(F("Unknown command: "));
        Serial.println(serialCmd);
        break;
    }

    // for some reason, when using SX126x GFSK and listening after transmission, 
    // the next packet received will have bad CRC,
    // and the data will be the transmitted packet
    // the only workaround seems to be resetting the module
    #if defined(USE_GFSK) && defined(USE_SX126X)
      radio.sleep(false);
      delay(10);
      setGFSK();
    #endif

    // set radio mode to reception
    #ifdef USE_SX126X
      radio.setDio1Action(onInterrupt);
    #else
      radio.setDio0Action(onInterrupt);
    #endif
    radio.startReceive();
    interruptEnabled = true;
  }

  // check if new data were received
  if (transmissionReceived) {
    // disable reception interrupt
    interruptEnabled = false;
    transmissionReceived = false;

    // read received data
    size_t respLen = radio.getPacketLength();
    uint8_t* respFrame = new uint8_t[respLen];
    int state = radio.readData(respFrame, respLen);

    // check reception success
    if (state == ERR_NONE) {
      decode(respFrame, respLen);

    } else if (state == ERR_CRC_MISMATCH) {
      Serial.println(F("Got CRC error!"));
      Serial.print(F("Received "));
      Serial.print(respLen);
      Serial.println(F(" bytes:"));
      PRINT_BUFF(respFrame, respLen);

    } else {
      Serial.println(F("Reception failed, code "));
      Serial.println(state);

    }

    // enable reception interrupt
    delete[] respFrame;
    radio.startReceive();
    interruptEnabled = true;
  }
}
