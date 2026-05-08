/*
 * Eeprom.c
 *
 *  Created on: Oct 4, 2025
 *      Author: SNYDE
 */


#include "main.h"
#include "Eeprom.h"

uint8_t Eeprom_init(Eeprom_t * eeprom, SPI_HandleTypeDef *spi)
{
	eeprom->spi = spi;

	uint8_t cmd = 0x9F;    // JEDEC ID command
	uint8_t id[3] = {0};

	HAL_GPIO_WritePin(SPI2_CS2_GPIO_Port, SPI2_CS2_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(spi, &cmd, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(spi, id, 3, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(SPI2_CS2_GPIO_Port, SPI2_CS2_Pin, GPIO_PIN_SET);


	if(id[0] != 0xBFu)
	{
		return 0u;
	}

	return 1u;
}
