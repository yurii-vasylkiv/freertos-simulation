/*
 * IMU.h
 *
 *  Created on: Jan. 12, 2021
 *      Author: Benjo
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "stm32f4xx_hal.h"

#define SWO_Pin GPIO_PIN_4
#define SWO_GPIO_Port GPIOB

double SENSSCALEGYRO = 16.4;
double SENSSCALEACCEL = 2048.0;

int readIMU(SPI_HandleTypeDef spi,uint8_t regAddress, int numBytes, uint8_t dReturned[]);
int writeIMU(SPI_HandleTypeDef spi, uint8_t regAddress, int numBytes, uint8_t dBuffer[]);
int selectBank(SPI_HandleTypeDef spi,int bank);
int configAccel(SPI_HandleTypeDef spi);
int configGyro(SPI_HandleTypeDef spi);
int configIMU(SPI_HandleTypeDef spi);
int readAccel(SPI_HandleTypeDef spi,float dBuffer[3]);
int readGyro(SPI_HandleTypeDef spi,float dBuffer[3]);
void readMag(SPI_HandleTypeDef spi, float dBuffer [3]);

#endif /* INC_IMU_H_ */
