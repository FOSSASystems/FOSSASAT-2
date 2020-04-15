# FOSSASAT-2 Software Dependencies
The following is a list of all libraries and other dependencies required to build software for FOSSASAT-2.

## Platform & Core Libraries
* STM32 Arduino core: https://github.com/stm32duino/Arduino_Core_STM32
* STM32 RTC library: https://github.com/stm32duino/STM32RTC
* STM32 Low Power library: https://github.com/stm32duino/STM32LowPower

## Libraries
* FOSSA-Comms: https://github.com/FOSSASystems/FOSSA-Comms
* RadioLib: https://github.com/jgromes/RadioLib
  * compile with static-only memory management enabled (src/BuildOpt.h - uncomment #define STATIC_ONLY)
* tiny-AES-c: https://github.com/FOSSASystems/tiny-AES-c
* Adafruit Bus IO: https://github.com/adafruit/Adafruit_BusIO
* Adafruit INA260: https://github.com/adafruit/Adafruit_INA260
* Adafruit VEML7700 Light Sensor Driver: https://github.com/adafruit/Adafruit_VEML7700
* Grove Mini I2C Motor Driver: https://github.com/kkoiwai/Grove_Mini_I2C_Motor_Driver
* SparkFun LSM9DS1: https://github.com/sparkfun/SparkFun_LSM9DS1_Arduino_Library
* FOSSA Systems ArduCAM fork: https://github.com/FOSSASystems/ArduCAM
