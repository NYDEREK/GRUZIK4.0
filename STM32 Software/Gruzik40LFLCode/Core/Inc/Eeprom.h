/*
 * Eeprom.h
 *
 *  Created on: Oct 4, 2025
 *      Author: SNYDE
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#define SST26_CMD_READ_SR1          0x05
#define SST26_CMD_WRITE_SR1         0x01
#define SST26_CMD_READ_SR2          0x35
#define SST26_CMD_WRITE_SR2         0x31
#define SST26_CMD_READ_JEDEC_ID     0x9F
#define SST26_CMD_READ              0x03
#define SST26_CMD_FAST_READ         0x0B
#define SST26_CMD_PAGE_PROGRAM      0x02
#define SST26_CMD_SECTOR_ERASE      0x20
#define SST26_CMD_BLOCK_ERASE_64K   0xD8
#define SST26_CMD_CHIP_ERASE        0xC7
#define SST26_CMD_WRITE_ENABLE      0x06
#define SST26_CMD_WRITE_DISABLE     0x04
#define SST26_CMD_READ_SREG         0x2B
#define SST26_CMD_WRITE_SREG        0x2F
#define SST26_CMD_READ_BAR          0x16
#define SST26_CMD_WRITE_BAR         0x17
#define SST26_CMD_READ_VCR          0x85
#define SST26_CMD_WRITE_VCR         0x81
#define SST26_CMD_READ_NVCR         0xB5
#define SST26_CMD_WRITE_NVCR        0xB1

#include "main.h"
#include "string.h" // memcpy

typedef struct
{
	SPI_HandleTypeDef *spi;


} Eeprom_t;

uint8_t Eeprom_init(Eeprom_t *eeprom, SPI_HandleTypeDef *spi);
#endif /* INC_EEPROM_H_ */
