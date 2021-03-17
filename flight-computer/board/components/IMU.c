/*
 * IMU.c
 *
 *  Created on: Jan. 12, 2021
 *      Author: Ben Martin
 */

#include "IMU.h"


int readIMU(SPI_HandleTypeDef spi,uint8_t regAddress, int numBytes, uint8_t dReturned[]){
	int i;
	for(i=0;i<numBytes;i++){ //get one byte at a time
		//Every register on IMU is only 8 bits wide
		//So for this function we will always only send 1 byte and have 1 byte returned
		//Because SPI, 2 bytes needs to be sent with the first being a dummy
		//The second byte read will be real data
		uint8_t dataOut[2];
		uint8_t dataBack[2];
		dataOut[0] = regAddress | 0x80; //make the 7th bit high to show its a read op
		dataOut[1] = 0x80; //setting dummy data
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);//CS goes low to activate device
		HAL_SPI_TransmitReceive(&spi, dataOut, dataBack, 2, 100);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);//CS goes high
		dReturned[i] = dataBack[1];
		regAddress++;
	}
}
int writeIMU(SPI_HandleTypeDef spi, uint8_t regAddress, int numBytes, uint8_t dBuffer[]){
	int i;
	for(i=0; i<numBytes;i++){//one byte at a time again
		uint8_t dataOut[2];
		dataOut[0] = regAddress;
		dataOut[1] = dBuffer[i];
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);//CS goes low to activate device
		HAL_SPI_Transmit(&spi, dataOut, 2, 100);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);//CS goes high
		regAddress++;
	}
}

int selectBank(SPI_HandleTypeDef spi,int bank){
	uint8_t dataOut[1];
	dataOut[0] = bank<<4;
	writeIMU(spi, 127, 1, dataOut);
}
int configAccel(SPI_HandleTypeDef spi){
	selectBank(spi,2);
	uint8_t prevSettings[2];
	readIMU(spi,20, 2, prevSettings);
	prevSettings[0] |= 6;
	writeIMU(spi,20, 2, prevSettings);
	selectBank(spi,0);

}
int configGyro(SPI_HandleTypeDef spi){
	selectBank(spi,2);
	uint8_t prevSettings[2];
	readIMU(spi,1, 2, prevSettings);
	prevSettings[0] |= 6;
	writeIMU(spi,1, 2, prevSettings);
	selectBank(spi,0);
}
int configIMU(SPI_HandleTypeDef spi){
	//PWRMGMT register at 0x06 each bits function is below
	//7: reset device if 1
	//6: sleep if 1
	//5: low power mode if 1
	//4: reserved
	//3: disables temp if 1
	//2-0: sets clock, val 1 is the best for us
	selectBank(spi, 0);
	uint8_t pwrMGMT[1] = {1};
	writeIMU(spi,0x06, 1, pwrMGMT);
	configAccel(spi);
	configGyro(spi);
}

int readAccel(SPI_HandleTypeDef spi,float dBuffer[3]){
	//data is in x,y,z order
	uint8_t rawData[6];
	readIMU(spi,45, 6, rawData);
	int i;
	for(i = 0;i<6;i=i+2){
		int16_t combinedVal = (int16_t)((((uint16_t)rawData[i]) << 8) | rawData[i+1]);
		dBuffer[i/2] = combinedVal/SENSSCALEACCEL; //datasheet says to do this
	}
	return 1;
}

int readGyro(SPI_HandleTypeDef spi,float dBuffer[3]){
	uint8_t rawData[6];
		readIMU(spi,51, 6, rawData);
		int i;
		for(i = 0;i<6;i=i+2){
			int16_t combinedVal = (int16_t)((((uint16_t)rawData[i]) << 8) | rawData[i+1]);
			dBuffer[i/2] = combinedVal/SENSSCALEGYRO;
		}
		return 1;
}