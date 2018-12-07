/*
 * ADXL.h
 *
 *  Created on: 14 nov. 2018
 *      Author: Max Pettersson
 */

#include "stm32f4xx_hal.h"

#ifndef ADXL_H_
#define ADXL_H_


uint8_t ADXL_Init(SPI_HandleTypeDef *hspi);
void ADXL_WriteRegister(SPI_HandleTypeDef*hspi, uint8_t reg, uint8_t val);
uint8_t ADXL_ReadRegister(SPI_HandleTypeDef*hspi, uint8_t reg);
uint8_t ADXL_ReadRegister_IT(SPI_HandleTypeDef*hspi, uint8_t reg);
void ADXL_ReadRegisters(SPI_HandleTypeDef*hspi, uint8_t reg, uint8_t* data, uint8_t len);
void ADXL_ReadRegisters_IT(SPI_HandleTypeDef*hspi, uint8_t reg, uint8_t* data, uint8_t len);
uint8_t ADXL_resetInt(SPI_HandleTypeDef*hspi);
void ADXL_Sleep(SPI_HandleTypeDef *hspi);
void ADXL_wakeUp(SPI_HandleTypeDef *hspi);

#endif /* ADXL_H_ */
