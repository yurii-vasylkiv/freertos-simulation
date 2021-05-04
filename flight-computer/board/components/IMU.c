/*
 * IMU.c
 *
 *  Created on: Jan. 12, 2021
 *      Author: Benjo
 */

#include "IMU.h"


int readIMU(SPI_HandleTypeDef spi,uint8_t regAddress, int numBytes, uint8_t dReturned[]){
	int i;
	for(i=0;i<numBytes;i++){
		//Every register on IMU is only 8 bits wide
		//So for this function we will always only send 1 byte and have 1 byte returned
		//Because SPI, 2 bytes needs to be sent with the first being a dummy
		//The second byte read will be real data
		uint8_t dataOut[2];
		uint8_t dataBack[2];
		dataOut[0] = regAddress | 0x80;
		dataOut[1] = 0x80;
		HAL_GPIO_WritePin(SWO_GPIO_Port, SWO_Pin, GPIO_PIN_RESET);//CS goes low to activate device
		HAL_SPI_TransmitReceive(&spi, dataOut, dataBack, 2, 100);
		HAL_GPIO_WritePin(SWO_GPIO_Port, SWO_Pin, GPIO_PIN_SET);//CS goes high
		dReturned[i] = dataBack[1];
		regAddress++;
	}
	return 1;
}
int writeIMU(SPI_HandleTypeDef spi, uint8_t regAddress, int numBytes, uint8_t dBuffer[]){
	int i;
	for(i=0; i<numBytes;i++){
		uint8_t dataOut[2];
		dataOut[0] = regAddress;
		dataOut[1] = dBuffer[i];
		HAL_GPIO_WritePin(SWO_GPIO_Port, SWO_Pin, GPIO_PIN_RESET);//CS goes low to activate device
		HAL_SPI_Transmit(&spi, dataOut, 2, 100);
		HAL_GPIO_WritePin(SWO_GPIO_Port, SWO_Pin, GPIO_PIN_SET);//CS goes high
		regAddress++;
	}
	return 1;
}

int selectBank(SPI_HandleTypeDef spi,int bank){
	uint8_t dataOut[1];
	dataOut[0] = bank<<4;
	writeIMU(spi, 127, 1, dataOut);
	return 1;
}

//reads one value from the Mag at the given register
static uint8_t ICM_Mag_Read(SPI_HandleTypeDef spi, uint8_t reg){
  	selectBank(spi, 3);
  	uint8_t Data;
  	uint8_t buffer[2];
  	buffer[0] = 0x0C|0x80;
  	buffer[1] = reg;
  	writeIMU(spi, 0x03, 2, buffer);// 3 and 4 is now set
  	HAL_Delay(1);
  	buffer[0] = 0xff;
  	writeIMU(spi, 0x06, 1, buffer);
  	HAL_Delay(1);
  	selectBank(spi, 0);
  	HAL_Delay(1);
  	readIMU(spi, 0x3B, 1, buffer);
  	HAL_Delay(1);
  	Data = buffer[0];
	selectBank(spi, 0);

  	return Data;

}
//writes one value from the IMU to the MAG
static void ICM_Mag_Write(SPI_HandleTypeDef spi, uint8_t reg, uint8_t val){
	selectBank(spi, 3);
	uint8_t buffer[2];
	buffer[0] = 0x0C;
	buffer[1] = reg;
	writeIMU(spi, 0x03, 2, buffer);// 3 and 4 is now set
	buffer[0] = val;
	writeIMU(spi, 0x06, 1, buffer);
	selectBank(spi, 0);

}

int configAccel(SPI_HandleTypeDef spi){
	selectBank(spi,2);
	uint8_t prevSettings[2];
	readIMU(spi,20, 2, prevSettings);
	prevSettings[0] |= 6;
	writeIMU(spi,20, 2, prevSettings);
	selectBank(spi,0);
	return 1;

}
int configGyro(SPI_HandleTypeDef spi){
	selectBank(spi,2);
	uint8_t prevSettings[2];
	readIMU(spi,1, 2, prevSettings);
	prevSettings[0] |= 6;
	writeIMU(spi,1, 2, prevSettings);
	selectBank(spi,0);
	return 1;
}

int configMAG(SPI_HandleTypeDef spi){
	selectBank(spi, 0);
	uint8_t dBuffer[1];
	dBuffer[0] = 0x30;
	writeIMU(spi, 0x0F, 1, dBuffer);// INT Pin / Bypass Enable Configuration
	dBuffer[0] = 0x20;
	writeIMU(spi, 0x03, 1, dBuffer);// I2C_MST_EN
	selectBank(spi, 3);
	dBuffer[0] = 0x4D;
	writeIMU(spi, 0x01, 1, dBuffer);// I2C Master mode and Speed 400 kHz
	dBuffer[0] = 0x01;
	writeIMU(spi, 0x02, 1, dBuffer);// I2C_SLV0 _DLY_ enable
	dBuffer[0] = 0x81;
	writeIMU(spi, 0x05, 1, dBuffer); // enable IIC	and EXT_SENS_DATA==1 Byte
	selectBank(spi, 0);

	//init magnetometer
	ICM_Mag_Write(spi,0x31,0x01);
	HAL_Delay(1000);
	ICM_Mag_Write(spi,0x31,0x02);
	selectBank(spi, 0);




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
	configMAG(spi);
	return 1;
}



void readMag(SPI_HandleTypeDef spi, float dBuffer [3]){
	uint8_t mag_buffer[10];

	mag_buffer[0] =ICM_Mag_Read(spi,0x01);

	mag_buffer[1] =ICM_Mag_Read(spi,0x11);
	mag_buffer[2] =ICM_Mag_Read(spi,0x12);
	mag_buffer[3] =ICM_Mag_Read(spi,0x13);
	mag_buffer[4] =ICM_Mag_Read(spi,0x14);
	mag_buffer[5] =ICM_Mag_Read(spi,0x15);
	mag_buffer[6] =ICM_Mag_Read(spi,0x16);
	int i;
		for(i = 0;i<6;i=i+2){
			int16_t combinedVal = (int16_t)((((uint16_t)mag_buffer[i+1]) << 8) | mag_buffer[i]);
			dBuffer[i/2] = combinedVal/4912.0;//4912 is constant fs range of mag
		}
	ICM_Mag_Write(spi, 0x31,0x01);

}

int readAccel(SPI_HandleTypeDef spi,float dBuffer[3]){
	//data is in x,y,z order
	uint8_t rawData[6];
	readIMU(spi,45, 6, rawData);
	int i;
	for(i = 0;i<6;i=i+2){
		int16_t combinedVal = (int16_t)((((uint16_t)rawData[i]) << 8) | rawData[i+1]);
		dBuffer[i/2] = combinedVal/SENSSCALEACCEL;
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
