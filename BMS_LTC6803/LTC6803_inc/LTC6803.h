#ifndef LTC6803_H
#define LTC6803_H

#include "stdint.h"

#define PEC_POLY 7

//! Function to start Cell Voltage measurement
//! @return void
void LTC6803_stcvad(void);

//! Function to start Temp channel voltage measurement
//! @return void
void LTC6803_sttmpad(void);

//! Function that reads Cell Voltage registers
//! @returns  This function will return a 0 if there is no PEC error and will return -1 if there is a PEC error
uint8_t LTC6803_rdcv(uint8_t total_ic, 			//!< total_ic number of LTC6803 ICs in stack
					 uint16_t cell_codes[][12]	//!< The Function will put the parsed measured cell voltages into this array
					 );
					 
//! Function that reads Temp Voltage registers	
//! @returns  This function will return a 0 if there is no PEC error and will return -1 if there is a PEC error
int8_t LTC6803_rdtmp(uint8_t total_ic,			//!< total_ic number of LTC6803 ICs in stack
					 uint16_t temp_codes[][3]	//!< The Function will put the parsed measured Temp voltages into this array
					 );
					 
//! Function that writes configuration of LTC6803-2/-4					 
//! @return void
void LTC6803_wrcfg(uint8_t total_ic,			//!< total_ic number of LTC6803 ICs in stack
				   uint8_t config[][6]			//!< The function will write the 6803 CFGR register with data in the config array
				   );

//! Function that reads configuration of LTC6803-2/-4			   
//! @returns  This function will return a 0 if there is no PEC error and will return -1 if there is a PEC error
int8_t LTC6803_rdcfg(uint8_t total_ic, 			//!< total_ic number of LTC6803 ICs in stack
					 uint8_t r_config[][7]		//!< The Function will put the read config register data into this array
					 );
//! Function that calculates PEC byte
//! @returns The calculated CRC8
uint8_t pec8_calc(uint8_t len,					//!< the length of the data array
				  uint8_t *data					//!< data array
				  );
#endif
