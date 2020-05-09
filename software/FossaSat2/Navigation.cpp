#include "Navigation.h"

static uint32_t Navigation_Parse_Int(const char* tleLine, uint8_t pos, uint8_t len) {
  char str[32];
  memcpy(str, tleLine + pos, len);
  str[len] = '\0';
  return(strtol(str, NULL, 10));
}

static double Navigation_Parse_Double(const char* tleLine, uint8_t pos, uint8_t len) {
  // copy TLE number
  char str[32];
  memcpy(str, tleLine + pos, len);
  str[len] = '\0';

  // check sign
  double sign = 1;
  if(str[0] == '-') {
    str[0] = '0';
    sign = -1;
  }

  // parse
  return(sign * strtod(str, NULL));
}

static double Navigation_Parse_Decimal(const char* tleLine, uint8_t pos, uint8_t len) {
  // copy TLE number
  char str[32];
  memcpy(str, tleLine + pos, len);
  str[len] = '\0';
  uint8_t mantLen = len;
  char* strPtr = str;

  // check sign
  double sign = 1;
  if(str[0] == '-') {
    sign = -1;
    mantLen -= 1;
    strPtr++;
    str[0] = ' ';
  }

  // get exponent
  double exponent = 0;
  if((str[len - 2] == '-') || (str[len - 2] == '+')) {
    exponent = strtod(str + len - 2, NULL);
    mantLen -= 2;
    str[len - 2] = ' ';
  }

  // get base
  double mant = strtod(strPtr, NULL);
  mant = mant / pow(10, mantLen - 1);

  // get the result
  return(sign * mant * pow(10, exponent));
}

uint8_t Navigation_Get_EpochYear(const char* tleLine) {
  return((uint8_t)Navigation_Parse_Int(tleLine, 18, 2));
}

double Navigation_Get_EpochDay(const char* tleLine) {
  return(Navigation_Parse_Double(tleLine, 20, 12));
}

double Navigation_Get_BallisticCoeff(const char* tleLine) {
  return(Navigation_Parse_Double(tleLine, 33, 10));
}

double Navigation_Get_MeanMotion2nd(const char* tleLine) {
  return(Navigation_Parse_Decimal(tleLine, 44, 8));
}

double Navigation_Get_DragTerm(const char* tleLine) {
  return(Navigation_Parse_Decimal(tleLine, 53, 8));
}

double Navigation_Get_Inclination(const char* tleLine) {
  return(Navigation_Parse_Double(tleLine, 8, 8));
}

double Navigation_Get_RightAscension(const char* tleLine) {
  return(Navigation_Parse_Double(tleLine, 17, 8));
}

double Navigation_Get_Eccentricity(const char* tleLine) {
  return(Navigation_Parse_Decimal(tleLine, 26, 7));
}

double Navigation_Get_PerigeeArgument(const char* tleLine) {
  return(Navigation_Parse_Double(tleLine, 34, 8));
}

double Navigation_Get_MeanAnomaly(const char* tleLine) {
  return(Navigation_Parse_Double(tleLine, 43, 8));
}

double Navigation_Get_MeanMotion(const char* tleLine) {
  return(Navigation_Parse_Double(tleLine, 52, 11));
}

double Navigation_Get_RevolutionNumber(const char* tleLine) {
  return(Navigation_Parse_Int(tleLine, 63, 5));
}
