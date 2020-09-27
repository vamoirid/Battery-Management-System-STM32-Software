#ifndef LT_SPI_H
#define LT_SPI_H

#include <stdint.h>

#define LTC6811_CHIP_SELECT_GPIO_PIN            GPIO_PIN_13     // D10
#define LTC6811_CHIP_SELECT_GPIO_PORT           GPIOC

//Initialize the SPI Peripheral

void LTC6811_Initialize(void);

// cs_low

void cs_low(void);

// cs_high

void cs_high(void);

// spi_write

void spi_write(uint8_t data); //Bytes to be written on the SPI port

// spi_write_array

void spi_write_array(uint8_t len, // Option: Number of bytes to be written on the SPI port
                     uint8_t data[] //Array of bytes to be written on the SPI port
                    );

/*
 Writes and read a set number of bytes using the SPI port.
*/

void spi_write_read(uint8_t tx_Data[],//array of data to be written on SPI port
                    uint8_t tx_len, //length of the tx data arry
                    uint8_t *rx_data,//Input: array that will store the data read by the SPI port
                    uint8_t rx_len //Option: number of bytes to be read from the SPI port
                   );

uint8_t spi_read_byte(uint8_t tx_dat);//name conflicts with linduino also needs to take a byte as a parameter
#endif
