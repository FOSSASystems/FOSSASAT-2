// include all libraries
#include <RadioLib.h>
#include <FOSSA-Comms.h>

#define USE_GFSK                    // uncomment to use GFSK
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
#define FREQ_DEV              10//5.0     // kHz SSB
#define RX_BANDWIDTH          93.8//46.9//39.0    // kHz SSB
#define FSK_PREAMBLE_LEN      16      // bits
#define DATA_SHAPING          0.5     // BT product
#define TCXO_VOLTAGE          1.6

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
  radio.transmit(frame, len);
  delete[] frame;
}

void sendFrameEncrypted(uint8_t functionId, uint8_t optDataLen = 0, uint8_t* optData = NULL) {
  // build frame
  uint8_t len = FCP_Get_Frame_Length(callsign, optDataLen, password);
  uint8_t* frame = new uint8_t[len];
  FCP_Encode(frame, callsign, functionId, optDataLen, optData, encryptionKey, password);

  // send data
  radio.transmit(frame, len);
  delete[] frame;
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

  // provide seed for PRNG
  randomSeed(analogRead(A6));

  while(!Serial.available());

  uint8_t slot = 0;
  sendFrameEncrypted(CMD_GET_PICTURE_BURST, 1, &slot);

  #if defined(USE_GFSK) && defined(USE_SX126X)
    radio.sleep(false);
    delay(10);
    setGFSK();
  #endif

  #ifdef USE_SX126X
    radio.setDio1Action(onInterrupt);
  #else
    radio.setDio0Action(onInterrupt);
  #endif
  
  // begin listening for packets
  radio.startReceive();
}

void loop() {
  // check if new data were received
  if (transmissionReceived) {
    // disable reception interrupt
    interruptEnabled = false;
    transmissionReceived = false;

    // read received data
    size_t respLen = radio.getPacketLength();
    uint8_t* respFrame = new uint8_t[respLen];
    int state = radio.readData(respFrame, respLen);

    if (state == ERR_NONE) {
      uint8_t functionId = FCP_Get_FunctionID(callsign, respFrame, respLen);
      if(functionId == RESP_CAMERA_PICTURE) {
        uint8_t respOptDataLen = FCP_Get_OptData_Length(callsign, respFrame, respLen);
        uint8_t* respOptData = new uint8_t[respOptDataLen];
        FCP_Get_OptData(callsign, respFrame, respLen, respOptData);

        uint16_t packetId = 0;
        memcpy(&packetId, respOptData, sizeof(uint16_t));
        //Serial.print(F("Packet ID: "));
        //Serial.println(packetId);
        
        char buff[16];
        if(respOptDataLen < 16) {
          for(uint8_t i = 0; i < respOptDataLen; i++) {
            sprintf(buff, "%02x ", respOptData[2 + i]);
            Serial.print(buff);
          }
          Serial.println();
        } else {
          for(uint8_t i = 0; i < respOptDataLen/16; i++) {
            for(uint8_t j = 0; j < 16; j++) {
              sprintf(buff, "%02x ", respOptData[2 + i*16 + j]);
              Serial.print(buff);
            }
            Serial.println();
          }
        }

        delete[] respOptData;
      }
    }

    // enable reception interrupt
    delete[] respFrame;
    radio.startReceive();
    interruptEnabled = true;
  }

}
