/*
 * ADXL.c
 *
 *  Created on: 14 nov. 2018
 *      Author: Max Pettersson
 */

#include "ADXL.h"



uint8_t ADXL_Init(SPI_HandleTypeDef *hspi)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
	ADXL_WriteRegister(hspi, 0x31, 0x0B); //Data format set full res and 16g
	ADXL_WriteRegister(hspi, 0x24, 0x15); //THRESH_ACT
	ADXL_WriteRegister(hspi, 0x27, 0x70); //Enable activity x, y, z
	ADXL_WriteRegister(hspi, 0x2E, 0x10); //Enable interrupt activity
	ADXL_WriteRegister(hspi, 0x2F, 0x10); //Set interrupt for activity on Int2
	ADXL_WriteRegister(hspi, 0x2C, 0x0F); //Set data rate to 3200Hz
	ADXL_WriteRegister(hspi, 0x38, 0xDF); //Set FIFO mode on trigger
	ADXL_WriteRegister(hspi, 0x2D, 0x08); //Enable measure mode
	ADXL_Sleep(hspi);


	ADXL_resetInt(hspi);

	return ADXL_ReadRegister(hspi, 0x00);
}

uint8_t ADXL_ReadRegister(SPI_HandleTypeDef*hspi, uint8_t reg)
{
	uint8_t inData;
	reg |= 0x80;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
	HAL_SPI_Transmit(hspi, &reg, 1, 20);
	HAL_SPI_Receive(hspi, &inData, 1, 20);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
	return inData;
}

uint8_t ADXL_ReadRegister_IT(SPI_HandleTypeDef*hspi, uint8_t reg)
{
	uint8_t inData;
	reg |= 0x80;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
	HAL_SPI_Transmit_IT(hspi, &reg, 1);
	HAL_SPI_Receive_IT(hspi, &inData, 1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
	return inData;
}

void ADXL_WriteRegister(SPI_HandleTypeDef*hspi, uint8_t reg, uint8_t val)
{
	uint8_t outData[2];
	outData[0] = reg|0x40;
	outData[1] = val;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
	HAL_SPI_Transmit(hspi, outData, 2, 200);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
}

void ADXL_ReadRegisters(SPI_HandleTypeDef*hspi, uint8_t reg, uint8_t*data, uint8_t len)
{
	uint8_t outData = reg|0x80|0x40;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
	HAL_SPI_Transmit(hspi, &outData, 1, 200);
	HAL_SPI_Receive(hspi, data, len, 200);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
}

void ADXL_ReadRegisters_IT(SPI_HandleTypeDef*hspi, uint8_t reg, uint8_t*data, uint8_t len)
{
	uint8_t outData = reg|0x80|0x40;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
	HAL_SPI_Transmit_IT(hspi, &outData, 1);
	HAL_SPI_Receive_IT(hspi, data, len);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
}

uint8_t ADXL_resetInt(SPI_HandleTypeDef*hspi)
{
	ADXL_ReadRegister(hspi, 0x30);
}

void ADXL_Sleep(SPI_HandleTypeDef *hspi)
{
	ADXL_WriteRegister(hspi, 0x2D, 0x0D);
}
void ADXL_wakeUp(SPI_HandleTypeDef *hspi)
{
	ADXL_WriteRegister(hspi, 0x2D, 0x00);
	ADXL_WriteRegister(hspi, 0x2D, 0x08);
}
