/*
 * l3gd20.c
 *
 *  Created on: 14 kwi 2018
 *      Author: User
 */
#include "stm32f4xx_hal.h"
#include "l3gd20.h"


extern I2C_HandleTypeDef hspi1;

void L3G_Error(void)
{
	HAL_GPIO_WritePin(LED_red_GPIO_Port, LED_red_Pin, GPIO_PIN_SET);
}

void L3G_Success(void)
{
	HAL_GPIO_WritePin(LED_red_GPIO_Port, LED_red_Pin, GPIO_PIN_RESET);
}


uint8_t L3G_SPIx_WriteRead(uint8_t Byte)
{
	uint8_t receivedbyte = 0;
	if(HAL_SPI_TransmitReceive(&hspi1,(uint8_t*) &Byte,(uint8_t*) &receivedbyte,1,0x10000)!=HAL_OK)
	{
		L3G_Error();
	}else{
		L3G_Success();
	}
	return receivedbyte;
}

void L3G_Gyro_IO_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
	if(NumByteToRead>0x01)
	{
		ReadAddr |= (uint8_t) (READWRITE_CMD | MULTIPLEBYTE_CMD);
	}else{
		ReadAddr |= (uint8_t) READWRITE_CMD;
	}
	CS_ON;
	L3G_SPIx_WriteRead(ReadAddr);
	while(NumByteToRead>0x00)
	{
		*pBuffer = L3G_SPIx_WriteRead(DUMMY_BYTE);
		NumByteToRead--;
		pBuffer++;
	}
	CS_OFF;
}

void L3G_Gyro_IO_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToRead)
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
}

uint8_t L3G_Gyro_ReadID(void)
{
	uint8_t ctrl = 0;
	L3G_Gyro_IO_Read(&ctrl,0x0F,1);
	return ctrl;
}

void L3G_Gyro_Ini(void)
{
	uint8_t ctrl = 0x00;

	if(L3G_Gyro_ReadID()==0xEA)
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

void L3G_Gyro_GetXYZ(int16_t* pData)
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
}
