/*
 * IMU.h
 *
 *  Created on: Jan. 12, 2021
 *      Author: Ben Martin
 *  This is a simple driver for the ICM-20948
 *  data sheet can be found here: https://www.mouser.ca/datasheet/2/400/DS-000189-ICM-20948-v1.3-1385562.pdf
 *  As of Jan 12 2021 this only supports the accelerometer and gyroscope
 *
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "stm32f4xx_hal.h"

#define SWO_Pin GPIO_PIN_4 //this is for the chip select pin
#define SWO_GPIO_Port GPIOB //if this value is changed make sure it is changed in the IMU.c file (readIMU and writeIMU)
//if theres a way of automating that change I didnt know it
double SENSSCALEGYRO = 16.4;//the IMU is programmed to have +/- 2000dps range so this is the sensitivity scale at that range
double SENSSCALEACCEL = 2048.0;//the IMU is programmed to have +/- 16g range so this is the sensitivity scale at that range

// reads x number of bytes starting from the regAddress and counts up (INCLUSIVE).  The data is stored in the array
int readIMU(SPI_HandleTypeDef spi,uint8_t regAddress, int numBytes, uint8_t dReturned[]);
// writes x number of bytes starting from the regAddress and counts up (INCLUSIVE).  The data to write should be stored in the array
int writeIMU(SPI_HandleTypeDef spi, uint8_t regAddress, int numBytes, uint8_t dBuffer[]);
//the IMU has 4 register banks, 0-3
int selectBank(SPI_HandleTypeDef spi,int bank);
//sets the accelerometer
int configAccel(SPI_HandleTypeDef spi);
//sets the gyro
int configGyro(SPI_HandleTypeDef spi);
//sets the power settings and calls configGyro and configAccel
int configIMU(SPI_HandleTypeDef spi);
//reads the accelerometer and returns 3 floats for x,y,z in gs
int readAccel(SPI_HandleTypeDef spi,float dBuffer[3]);
//reads the gyroscope and returns 3 floats for rotation in x,y,z measured in degrees/s
int readGyro(SPI_HandleTypeDef spi,float dBuffer[3]);

#endif /* INC_IMU_H_ */