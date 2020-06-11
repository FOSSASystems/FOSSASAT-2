#include "crc32.h"

uint32_t CRC32_Get(uint8_t* buff, size_t len, uint32_t initial) {
  uint32_t crc = initial;
  while (len--) {
    crc = (crc << 8) ^ crc32_table[((crc >> 24) ^ *buff) & 255];
    buff++;
  }

  return crc;
}
