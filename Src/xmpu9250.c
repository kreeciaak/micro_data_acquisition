/*
 * mpu9250.c
 *
 *  Created on: 14 kwi 2018
 *      Author: User
 */


#include <xmpu9250.h>
#include "stm32f4xx_hal.h"


extern I2C_HandleTypeDef hi2c3;


// LED Indication
void MPU_Error(void)
{
	HAL_GPIO_WritePin(LED_blue_GPIO_Port, LED_blue_Pin, GPIO_PIN_SET);
}

void MPU_Success(void)
{
	HAL_GPIO_WritePin(LED_blue_GPIO_Port, LED_blue_Pin, GPIO_PIN_RESET);
}

//I2C Operations

static uint8_t MPU_I2C_Read(uint16_t Addr, uint8_t Reg)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t value = 0;
	status = HAL_I2C_Mem_Read(&hi2c3, Addr, Reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 0x10000);
	if (status != HAL_OK) MPU_Error();
	else MPU_Success();
	return value;
}

static void MPU_I2C_Write(uint16_t Addr, uint8_t Reg, uint8_t Value)
{
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Write(&hi2c3, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, 0x10000);
	if (status != HAL_OK) MPU_Error();
	else MPU_Success();
}

uint8_t MPU_I2C_ReadID(uint16_t Addr)
{
	uint8_t ctrl = 0x00;
	ctrl = MPU_I2C_Read(Addr, MPU9250_RA_WHO_AM_I);
	return ctrl;
}

void MPU_Accel_Ini(void)
{
	uint8_t ctrl;
	if (MPU_I2C_ReadID(MPU9250_DEFAULT_ADDRESS)==0x71)
	{
		ctrl = LSM303DLHC_ODR_RATE_200 | LSM303DLHC_LPEN_NORMAL_MODE | LSM303DLHC_3AXIS_ENABLE;
		MPU_I2C_Write(LSM303DLHC_ADDRESS_A,LSM303DLHC_RA_CTRL_REG1_A,ctrl);
		ctrl = LSM303DLHC_I1_DRDY1_ENABLE;
		MPU_I2C_Write(LSM303DLHC_ADDRESS_A,LSM303DLHC_RA_CTRL_REG3_A,ctrl);
		ctrl = LSM303DLHC_CONTINUOS_UPDATE | LSM303DLHC_LITTLE_ENDIAN | LSM303DLHC_FS_2 | LSM303DLHC_HIGH_RES;
		MPU_I2C_Write(LSM303DLHC_ADDRESS_A,LSM303DLHC_RA_CTRL_REG4_A,ctrl);
	}
	HAL_Delay(500);
}


extern SPI_HandleTypeDef hspi2;

void MPU_Error(void)
{
	HAL_GPIO_WritePin(LED_blue_GPIO_Port, LED_blue_Pin, GPIO_PIN_SET);
}

void MPU_Success(void)
{
	HAL_GPIO_WritePin(LED_blue_GPIO_Port, LED_blue_Pin, GPIO_PIN_RESET);
}


uint8_t MPU_SPIx_WriteRead(uint8_t Byte)
{
	uint8_t receivedbyte = 0;
	if(HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) &Byte,(uint8_t*) &receivedbyte,1,0x10000)!=HAL_OK)
	{
		MPU_Error();
	}else{
		MPU_Success();
	}
	return receivedbyte;
}

void MPU_Gyro_IO_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
	if(NumByteToRead>0x01)
	{
		ReadAddr |= (uint8_t) (READWRITE_CMD | MULTIPLEBYTE_CMD);
	}else{
		ReadAddr |= (uint8_t) READWRITE_CMD;
	}
	CS2_ON;
	MPU_SPIx_WriteRead(ReadAddr);
	while(NumByteToRead>0x00)
	{
		*pBuffer = MPU_SPIx_WriteRead(DUMMY_BYTE);
		NumByteToRead--;
		pBuffer++;
	}
	CS2_OFF;
}

/*void L3G_Gyro_IO_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToRead)
{
	CS_OFF;
	if(NumByteToRead>0x01)
	{
		WriteAddr |= (uint8_t)  MULTIPLEBYTE_CMD;
	}
	CS_ON;
	L3G_SPIx_WriteRead(WriteAddr);
	while(NumByteToRead>=0x01)
	{
		L3G_SPIx_WriteRead(*pBuffer);
		NumByteToRead--;
		pBuffer++;
	}
	CS_OFF;
}*/

uint8_t MPU_Gyro_ReadID(void)
{
	uint8_t ctrl = 0;
	MPU_Gyro_IO_Read(&ctrl,0x75,1);
	return ctrl;
}

void MPU_Gyro_Ini(void)
{
	uint8_t ctrl = 0x00;

	if(MPU_Gyro_ReadID()==0x71)
	{
		ctrl = L3GD20_DR_190HZ | L3GD20_BW_VHIGH | L3GD20_NORM_MODE_EN | L3GD20_XYZ_EN;
		L3G_Gyro_IO_Write(&ctrl,L3GD20_CTRL_REG1,1);

		ctrl = L3GD20_HPM_NOR_M_R | L3GD20_HPCF_10;
		L3G_Gyro_IO_Write(&ctrl,L3GD20_CTRL_REG2,1);

		ctrl = L3GD20_I2_DRDY_EN;
		L3G_Gyro_IO_Write(&ctrl,L3GD20_CTRL_REG3,1);

		ctrl = L3GD20_FS_250DPS;
		L3G_Gyro_IO_Write(&ctrl,L3GD20_CTRL_REG4,1);

	}else{
		L3G_Error();
	}
	HAL_Delay(500);
}

/*void L3G_Gyro_GetXYZ(int16_t* pData)
{
	uint8_t buffer[6];
	uint8_t i = 0;

	//L3G_Gyro_IO_Read(buffer,L3GD20_OUT_X_L,6);

	L3G_Gyro_IO_Read((uint8_t*) &buffer[0],L3GD20_OUT_X_L,1);
	L3G_Gyro_IO_Read((uint8_t*) &buffer[1],L3GD20_OUT_X_H,1);
	L3G_Gyro_IO_Read((uint8_t*) &buffer[2],L3GD20_OUT_Y_L,1);
	L3G_Gyro_IO_Read((uint8_t*) &buffer[3],L3GD20_OUT_Y_H,1);
	L3G_Gyro_IO_Read((uint8_t*) &buffer[4],L3GD20_OUT_Z_L,1);
	L3G_Gyro_IO_Read((uint8_t*) &buffer[5],L3GD20_OUT_Z_H,1);


	for (i=0; i<3; i++)
	{
		pData[i] = ((int16_t)((uint16_t)buffer[2*i+1]<<8)+buffer[2*i]);
	}
}*/

