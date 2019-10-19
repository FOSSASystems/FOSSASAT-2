# FOSSASAT-2 I2C Address Map

The purpose of this document is to keep track of I2C adddresses of all devices on all I2C buses on FOSSASAT-2.

**IMPORTANT**: When adding new devices, care MUST be taken so that no two devices on the same I2C bus share the same address! In addition, all used I2C buses SHOULD have similar number of devices.

## I2C1

| #  | Device    | Address |
| -- | --------- | ------- |
| 1  | TSL2651_1 | 0101001 |
| 2  | TSL2651_2 | 0111001 |
| 3  | TSL2651_3 | 1001001 |
| 4  | DRV8830_1 | 1100000 |
| 5  | DRV8830_2 | 1100001 |
| 6  | DRV8830_3 | 1100010 |
| 7  | INA260_1  | 1000000 |
| 8  | INA260_2  | 1000001 |
| 9  | INA260_3  | 1000010 |
| 10 | INA260_4  | 1000011 |
| 11 | INA260_5  | 1000100 |

## I2C2

| #  | Device      | Address |
| -- | ----------- | ------- |
| 1  | TSL2651_4   | 0101001 |
| 2  | TSL2651_5   | 0111001 |
| 3  | TSL2651_6   | 1001001 |
| 4  | TMP100_1    | 1001000 |
| 5  | TMP100_2    | 1001001 |
| 6  | TMP100_3    | 1001010 |
| 7  | TMP100_4    | 1001011 |
| 8  | TMP100_5    | 1001100 |
| 9  | LSM9DS1 ACC | 1101010 |
| 10 | LSM9DS1 MAG | 0011100 |

## Address space

Used adress are crossed out.

| Device      | Available addresses          |
| ----------- | ---------------------------- |
| TSL2651     | ~~0101001~~ (I2C1 1, I2C2 1) |
|             | ~~0111001~~ (I2C1 2, I2C2 2) |
|             | ~~1001001~~ (I2C1 3, I2C2 3) |
| INA260      | ~~1000000~~ (I2C1 7)         |
|             | ~~1000001~~ (I2C1 8)         |
|             | ~~1000010~~ (I2C1 9)         |
|             | ~~1000011~~ (I2C1 10)        |
|             | ~~1000100~~ (I2C1 11)        |
|             | 1000101                      |
|             | 1000110                      |
|             | 1000111                      |
|             | 1001000                      |
|             | 1001001                      |
|             | 1001010                      |
|             | 1001011                      |
|             | 1001100                      |
|             | 1001101                      |
|             | 1001110                      |
|             | 1001111                      |
| DRV8830     | ~~1100000~~ (I2C1 4)         |
|             | ~~1100001~~ (I2C1 5)         |
|             | ~~1100010~~ (I2C1 6)         |
|             | 1100011                      |
|             | 1100100                      |
|             | 1100101                      |
|             | 1100110                      |
|             | 1100111                      |
|             | 1101000                      |
| TMP100      | ~~1001000~~ (I2C2 4)         |
|             | ~~1001001~~ (I2C2 5)         |
|             | ~~1001010~~ (I2C2 6)         |
|             | ~~1001011~~ (I2C2 7)         |
|             | ~~1001100~~ (I2C2 8)         |
|             | 1001101                      |
|             | 1001110                      |
|             | 1001111                      |
| LSM9DS1 ACC | ~~1101010~~ (I2C2 9)         |
|             | 1101011                      |
| LSM9DS1 MAG | ~~0011100~~ (I2C2 10)        |
|             | 0011110                      |
