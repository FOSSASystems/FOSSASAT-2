/*
    FossaSat2.h
*/

#include <RadioLib.h>
#include "Debug.h"

/*
    Configuration.h
*/

#define RADIO_MOSI                                      PB15
#define RADIO_MISO                                      PB14
#define RADIO_SCK                                       PB13
#define RADIO_NSS                                       PC7
#define RADIO_DIO1                                      PC9
#define RADIO_NRST                                      PC8
#define RADIO_BUSY                                      PC6
#define WATCHDOG_IN                                     PC13

// modem definitions
#define MODEM_FSK                                       'F'
#define MODEM_LORA                                      'L'

// common
#define CALLSIGN_DEFAULT                                "FOSSASAT-2"
#define SYNC_WORD                                       0x12        /*!< Ensure this sync word is compatable with all devices. */
#define TCXO_VOLTAGE                                    1.6         /*!< Sets the radio's TCX0 voltage. (V) */
#define MAX_NUM_OF_BLOCKS                               3           /*!< maximum number of AES128 blocks that will be accepted */
#define LORA_RECEIVE_WINDOW_LENGTH                      40          /*!< How long to listen out for LoRa transmissions for (s) */
#define FSK_RECEIVE_WINDOW_LENGTH                       20          /*!< How long to listen out for FSK transmissions for (s) */
#define RESPONSE_DELAY                                  600         /*!< How long to wait for before responding to a transmission (ms) */
#define WHITENING_INITIAL                               0x1FF       /*!< Whitening LFSR initial value, to ensure SX127x compatibility */

// LoRa
#define LORA_FREQUENCY                                  436.7       /*!< MHz */
#define LORA_BANDWIDTH                                  125.0       /*!< kHz dual sideband */
#define LORA_SPREADING_FACTOR                           11
#define LORA_SPREADING_FACTOR_ALT                       10
#define LORA_CODING_RATE                                8           /*!< 4/8, Extended Hamming */
#define LORA_OUTPUT_POWER                               20          /*!< dBm */
#define LORA_CURRENT_LIMIT                              140.0       /*!< mA */
#define LORA_PREAMBLE_LENGTH                            8       // symbols

// GFSK
#define FSK_FREQUENCY                                   436.9       /*!< MHz */
#define FSK_BIT_RATE                                    9.6         /*!< kbps nominal */
#define FSK_FREQUENCY_DEVIATION                         5.0         /*!< kHz single-sideband */
#define FSK_RX_BANDWIDTH                                39.0        /*!< kHz single-sideband */
#define FSK_OUTPUT_POWER                                20          /*!< dBm */
#define FSK_PREAMBLE_LENGTH                             16          /*!< bits */
#define FSK_DATA_SHAPING                                RADIOLIB_SHAPING_0_5  /*!< GFSK filter BT product */
#define FSK_CURRENT_LIMIT                               140.0       /*!< mA */

// Morse Code
#define NUM_CW_BEEPS                                    3           /*!< number of CW sync beeps in low power mode */
#define MORSE_PREAMBLE_LENGTH                           3           /*!< number of start signal repetitions */
#define MORSE_SPEED                                     20          /*!< words per minute */
#define MORSE_BATTERY_MIN                               3200.0      /*!< minimum voltage value that can be send via Morse (corresponds to 'A'), mV*/
#define MORSE_BATTERY_STEP                              50.0        /*!< voltage step in Morse, mV*/

/*
    Configuration.cpp
*/
uint8_t spreadingFactorMode = LORA_SPREADING_FACTOR;
SPIClass RadioSPI(RADIO_MOSI, RADIO_MISO, RADIO_SCK);
SX1262 radio = new Module(RADIO_NSS, RADIO_DIO1, RADIO_NRST, RADIO_BUSY, RadioSPI, SPISettings(2000000, MSBFIRST, SPI_MODE0));
MorseClient morse(&radio);

/*
    Communication.cpp
*/
int16_t Communication_Set_Modem(uint8_t modem) {
  int16_t state = ERR_NONE;
  FOSSASAT_DEBUG_PRINT(F("Set modem "));
  FOSSASAT_DEBUG_WRITE(modem);
  FOSSASAT_DEBUG_PRINTLN();

  // initialize requested modem
  switch (modem) {
    case MODEM_LORA:
        state = radio.begin(LORA_FREQUENCY,
                            LORA_BANDWIDTH,
                            LORA_SPREADING_FACTOR,
                            LORA_CODING_RATE,
                            SYNC_WORD,
                            LORA_OUTPUT_POWER,
                            LORA_PREAMBLE_LENGTH,
                            TCXO_VOLTAGE);
        radio.setCRC(true);
        radio.setCurrentLimit(LORA_CURRENT_LIMIT);
      break;
    case MODEM_FSK: {
        state = radio.beginFSK(FSK_FREQUENCY,
                               FSK_BIT_RATE,
                               FSK_FREQUENCY_DEVIATION,
                               FSK_RX_BANDWIDTH,
                               FSK_OUTPUT_POWER,
                               FSK_PREAMBLE_LENGTH,
                               TCXO_VOLTAGE);
        uint8_t syncWordFSK[2] = {SYNC_WORD, SYNC_WORD};
        radio.setSyncWord(syncWordFSK, 2);
        radio.setCRC(2);
        radio.setDataShaping(FSK_DATA_SHAPING);
        radio.setCurrentLimit(FSK_CURRENT_LIMIT);
      } break;
    default:
      FOSSASAT_DEBUG_PRINT(F("Unkown modem "));
      FOSSASAT_DEBUG_PRINTLN(modem);
      return(ERR_UNKNOWN);
  }

  radio.setWhitening(true, WHITENING_INITIAL);

  // handle possible error codes
  FOSSASAT_DEBUG_PRINT(F("Radio init "));
  FOSSASAT_DEBUG_PRINTLN(state);
  FOSSASAT_DEBUG_DELAY(10);
  if (state != ERR_NONE) {
    // radio chip failed, restart
    //PowerControl_Watchdog_Restart();
  }

  // set spreading factor
  //Communication_Set_SpreadingFactor(spreadingFactorMode);

  // save current modem
  //currentModem = modem;
  return(state);
}

static uint8_t dummyPacket[255];

void setup() {
  FOSSASAT_DEBUG_PORT.begin(FOSSASAT_DEBUG_SPEED);
  while(!FOSSASAT_DEBUG_PORT);
  FOSSASAT_DEBUG_PORT.println();

  RadioSPI.begin();

  memset(dummyPacket, 0xAA, sizeof(dummyPacket));
}

void loop() {
  // CW beep
  Communication_Set_Modem(MODEM_FSK);
  radio.transmitDirect();
  delay(5000);
  radio.standby();
  digitalWrite(WATCHDOG_IN, !digitalRead(WATCHDOG_IN));
  delay(1000);

  // LoRa Tx
  Communication_Set_Modem(MODEM_LORA);
  radio.transmit(dummyPacket, sizeof(dummyPacket));
  digitalWrite(WATCHDOG_IN, !digitalRead(WATCHDOG_IN));
  delay(1000);

  // FSK Tx
  Communication_Set_Modem(MODEM_FSK);
  for(uint8_t i = 0; i < 10; i++) {
    radio.transmit(dummyPacket, sizeof(dummyPacket));
  }
  digitalWrite(WATCHDOG_IN, !digitalRead(WATCHDOG_IN));
  delay(1000);
}
