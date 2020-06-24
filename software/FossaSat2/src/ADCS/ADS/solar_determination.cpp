/* Project: FossaSat-2 Flight Software
   Author: Team Steiner
   Module: ads_determination
   File: solar_determination.c
   04/30/20

   This file drives the calculation of the solar ephemeris from lux and current data
*/

/***************** Headers ********************/
#include "../ADCS/adcs.h"

/*************** Main function *****************/
void ADS_Solar_Determination(ADCS_CALC_TYPE luxData[], ADCS_CALC_TYPE solarEph[], ADCS_CALC_TYPE redundantSolarEph[]) {
  // Constants definitions
  const ADCS_CALC_TYPE maxCurrent = 1.0;                      // Maximum obtainable current
  ADCS_CALC_TYPE panelUnitVector[ADCS_NUM_PANELS][ADCS_NUM_AXES] = ADCS_PANEL_UNIT_VECTOR; // Unitary vector pointing to the solar panels in inverse form

  // Main calculation
  for(uint8_t i = 0; i < ADCS_NUM_AXES; i++) {
    solarEph[i] = 0;
    for(uint8_t j = 0; j < ADCS_NUM_AXES; j++) {
      solarEph[i] += panelUnitVector[i][j]*luxData[j]*(1.0/maxCurrent);
    }
  }

  for(uint8_t i = 0; i < ADCS_NUM_AXES; i++) {
    redundantSolarEph[i] = 0;
    for(uint8_t j = 0; j < ADCS_NUM_AXES; j++) {
      redundantSolarEph[i] += panelUnitVector[ADCS_NUM_AXES + i][j]*luxData[ADCS_NUM_AXES + j]*(1.0/maxCurrent);
    }
  }
}
