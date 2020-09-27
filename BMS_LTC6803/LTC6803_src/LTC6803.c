#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "LT_SPI.h"
#include "LTC6803.h"

// Function that writes configuration of LTC6803-2/-3
void LTC6803_wrcfg(uint8_t total_ic,uint8_t config[][6])
{
  uint8_t BYTES_IN_REG = 6;
  uint8_t CMD_LEN = 4+7;
  uint8_t *cmd;
  uint16_t cfg_pec;
  uint8_t cmd_index; //command counter

  cmd = (uint8_t *)malloc(CMD_LEN*sizeof(uint8_t));
  for (uint8_t current_ic = 0; current_ic < total_ic; current_ic++)
  {
    cmd[0] = 0x80 + current_ic;
    cmd[1] = pec8_calc(1,cmd);

    cmd[2] = 0x01;
    cmd[3] = 0xc7;

    cmd_index = 4;


    for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++)
    {
      cmd[cmd_index] = config[current_ic][current_byte];
      cmd_index = cmd_index + 1;
    }

    cfg_pec = pec8_calc(BYTES_IN_REG, &config[current_ic][0]);    // calculating the PEC for each ICs configuration register data
    cmd[cmd_index ] = (uint8_t)cfg_pec;
    cmd_index = cmd_index + 1;

    cs_low();
    spi_write_array(CMD_LEN, cmd);
    cs_high();
  }
  free(cmd);
}

//!Function that reads configuration of LTC6803-2/-3
int8_t LTC6803_rdcfg(uint8_t total_ic, //Number of ICs in the system
                     uint8_t r_config[][7] //A two dimensional array that the function stores the read configuration data.
                    )
{
  uint8_t BYTES_IN_REG = 7;

  uint8_t cmd[4];
  uint8_t *rx_data;
  int8_t pec_error = 0;
  uint8_t data_pec;
  uint8_t received_pec;

  rx_data = (uint8_t *) malloc((BYTES_IN_REG*total_ic)*sizeof(uint8_t));

  for (uint8_t current_ic = 0; current_ic < total_ic; current_ic++)       //executes for each LTC6803 in the daisy chain and packs the data
  {
    //into the r_config array as well as check the received Config data
    //for any bit errors

    cmd[0] = 0x80 + current_ic;
    cmd[1] = pec8_calc(1,cmd);
    cmd[2] = 0x02;
    cmd[3] = 0xCE;

    cs_low();
    spi_write_read(cmd, 4, rx_data, (BYTES_IN_REG*total_ic));
    cs_high();

    for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++)
    {
      r_config[current_ic][current_byte] = rx_data[current_byte];
    }

    received_pec =  r_config[current_ic][6];
    data_pec = pec8_calc(6, &r_config[current_ic][0]);
    if (received_pec != data_pec)
    {
      pec_error = -1;
    }
  }

  free(rx_data);
  return(pec_error);
}

//!Function that starts Cell Voltage measurement
void LTC6803_stcvad()
{
  cs_low();
  spi_write(0x10);
  spi_write(0xB0);
  cs_high();
}

//! Function that Temp channel voltage measurement
void LTC6803_sttmpad()
{
  cs_low();
  spi_write(0x30);
  spi_write(0x50);
  cs_high();
}

//!Function that reads Temp Voltage registers
int8_t LTC6803_rdtmp(uint8_t total_ic, uint16_t temp_codes[][3])
{
  int data_counter = 0;
  int pec_error = 0;
  uint8_t data_pec = 0;
  uint8_t received_pec = 0;
  uint8_t cmd[4];
  uint8_t *rx_data;
  rx_data = (uint8_t *) malloc((7)*sizeof(uint8_t));

  for (int i=0; i<total_ic; i++)
  {
    cmd[0] = 0x80 + i;
    cmd[1] = pec8_calc(1,cmd);
    cmd[2] = 0x0E;
    cmd[3] = 0xEA;
		
    cs_low();
    spi_write_read(cmd, 4,rx_data,6);
    cs_high();

    received_pec =  rx_data[5];
    data_pec = pec8_calc(5, &rx_data[0]);
    if (received_pec != data_pec)
    {
      pec_error = -1;
    }

    //int cell_counter = 0;
    data_counter = 0;
    int temp,temp2;

    temp = rx_data[data_counter++];
    temp2 = (rx_data[data_counter]& 0x0F)<<8;
    temp_codes[i][0] = temp + temp2 -512;
    temp2 = (rx_data[data_counter++])>>4;
    temp =  (rx_data[data_counter++])<<4;
    temp_codes[i][1] = temp+temp2 -512;
    temp2 = (rx_data[data_counter++]);
    temp =  (rx_data[data_counter++]& 0x0F)<<8;
    temp_codes[i][2] = temp+temp2 -512;
  }
  free(rx_data);
  return(pec_error);
}

//! Function that reads Cell Voltage registers
uint8_t LTC6803_rdcv( uint8_t total_ic, uint16_t cell_codes[][12])
{
  int data_counter =0;
  int pec_error = 0;
  uint8_t data_pec = 0;
  uint8_t received_pec = 0;
  uint8_t *rx_data;
  uint8_t cmd[4];
  rx_data = (uint8_t *) malloc((19)*sizeof(uint8_t));

  for (int i=0; i<total_ic; i++)
  {
    cmd[0] = 0x80 + i;
    cmd[1] = pec8_calc(1,cmd);
    cmd[2] = 0x04;
    cmd[3] = 0xDC;
		
    cs_low();
    spi_write_read(cmd, 4,rx_data,19);
    cs_high();

    received_pec =  rx_data[18];
    data_pec = pec8_calc(18, &rx_data[0]);
    if (received_pec != data_pec)
    {
      pec_error = -1;
    }

    //int cell_counter = 0;
    data_counter = 0;
    uint16_t temp,temp2;

    for (int k = 0; k<12; k=k+2)
    {
      temp = rx_data[data_counter++];
      temp2 = (uint16_t)(rx_data[data_counter]&0x0F)<<8;
      cell_codes[i][k] = temp + temp2 - 512;
      temp2 = (rx_data[data_counter++])>>4;
      temp =  (rx_data[data_counter++])<<4;
      cell_codes[i][k+1] = temp+temp2 - 512;
    }
  }
  free(rx_data);
  return(pec_error);
}

//!Function that calculates PEC byte
uint8_t pec8_calc(uint8_t len, uint8_t *data)
{
  uint8_t  remainder = 0x41;//PEC_SEED;
  /*
   * Perform modulo-2 division, a byte at a time.
   */
  for (int byte = 0; byte < len; ++byte)
  {
    /*
     * Bring the next byte into the remainder.
     */
    remainder ^= data[byte];
    /*
     * Perform modulo-2 division, a bit at a time.
     */
    for (uint8_t bit = 8; bit > 0; --bit)
    {
      /*
       * Try to divide the current data bit.
       */
      if (remainder & 128)
      {
        remainder = (remainder << 1) ^ PEC_POLY;
      }
      else
      {
        remainder = (remainder << 1);
      }
    }
  }
  /*
   * The final remainder is the CRC result.
   */
  return (remainder);
}
