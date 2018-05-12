/*
 * lsm303dlhc.c
 *
 *  Created on: 14 kwi 2018
 *      Author: User
*/

#include "stm32f4xx_hal.h"
#include "lsm303dlhc.h"


extern I2C_HandleTypeDef hi2c1;


// LED Indication
void LSM_Error(void)
{
	HAL_GPIO_WritePin(LED_red_GPIO_Port, LED_red_Pin, GPIO_PIN_SET);
}

void LSM_Success(void)
{
	HAL_GPIO_WritePin(LED_red_GPIO_Port, LED_red_Pin, GPIO_PIN_RESET);
}

//I2C Operations

static uint8_t LSM_I2C_Read(uint16_t Addr, uint8_t Reg)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t value = 0;
	status = HAL_I2C_Mem_Read(&hi2c1, Addr, Reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 0x10000);
	if (status != HAL_OK) LSM_Error();
	else LSM_Success();
	return value;
}

static void LSM_I2C_Write(uint16_t Addr, uint8_t Reg, uint8_t Value)
{
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Write(&hi2c1, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, 0x10000);
	if (status != HAL_OK) LSM_Error();
	else LSM_Success();
}

uint8_t LSM_I2C_ReadID(uint16_t Addr)
{
	uint8_t ctrl = 0x00;
	ctrl = LSM_I2C_Read(Addr, 0x0F);
	return ctrl;
}

void LSM_Accel_Ini(void)
{
	uint8_t ctrl, adress;
	adress = LSM_I2C_ReadID(LSM303DLHC_ADDRESS_A);
	if (adress ==0x32 || adress == 0x33)
	{
		ctrl = LSM303DLHC_ODR_RATE_200 | LSM303DLHC_LPEN_NORMAL_MODE | LSM303DLHC_3AXIS_ENABLE;
		LSM_I2C_Write(LSM303DLHC_ADDRESS_A,LSM303DLHC_RA_CTRL_REG1_A,ctrl);
		ctrl = LSM303DLHC_I1_DRDY1_ENABLE;
		LSM_I2C_Write(LSM303DLHC_ADDRESS_A,LSM303DLHC_RA_CTRL_REG3_A,ctrl);
		ctrl = LSM303DLHC_CONTINUOS_UPDATE | LSM303DLHC_LITTLE_ENDIAN | LSM303DLHC_FS_2 | LSM303DLHC_HIGH_RES;
		LSM_I2C_Write(LSM303DLHC_ADDRESS_A,LSM303DLHC_RA_CTRL_REG4_A,ctrl);
	}
	HAL_Delay(500);
}

void LSM_Accel_GetXYZ(int16_t* pData)
{
	uint8_t buffer[6];
	uint8_t i = 0;

	/*buffer[0] = LSM_I2C_Read(LSM303DLHC_ADDRESS_A,LSM303DLHC_RA_OUT_X_L_A);
	buffer[1] = LSM_I2C_Read(LSM303DLHC_ADDRESS_A,LSM303DLHC_RA_OUT_X_H_A);
	buffer[2] = LSM_I2C_Read(LSM303DLHC_ADDRESS_A,LSM303DLHC_RA_OUT_Y_L_A);
	buffer[3] = LSM_I2C_Read(LSM303DLHC_ADDRESS_A,LSM303DLHC_RA_OUT_Y_H_A);
	buffer[4] = LSM_I2C_Read(LSM303DLHC_ADDRESS_A,LSM303DLHC_RA_OUT_Z_L_A);
	buffer[5] = LSM_I2C_Read(LSM303DLHC_ADDRESS_A,LSM303DLHC_RA_OUT_Z_H_A);*/

	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Read(&hi2c1, LSM303DLHC_ADDRESS_A, LSM303DLHC_RA_OUT_X_L_A | LSM303DLHC_OUT_MULTI_READ, I2C_MEMADD_SIZE_8BIT, buffer, 6, 0x10000);
	if (status != HAL_OK) LSM_Error();
	else LSM_Success();

	for (i=0; i<3; i++)
	{
		pData[i] = ((int16_t)((uint16_t)buffer[2*i+1]<<8)+buffer[2*i]);
	}

}

void LSM_Mag_Ini(void)
{
	uint8_t ctrl;
	if (LSM_I2C_ReadID(LSM303DLHC_ADDRESS_M)==0x3C)
	{
		ctrl = LSM303DLHC_TEMP_DISABLE | LSM303DLHC_DO_RATE_220;
		LSM_I2C_Write(LSM303DLHC_ADDRESS_M,LSM303DLHC_RA_CRA_REG_M,ctrl);
		ctrl = LSM303DLHC_GN_1100;
		LSM_I2C_Write(LSM303DLHC_ADDRESS_M,LSM303DLHC_RA_CRB_REG_M,ctrl);
		ctrl = LSM303DLHC_MD_CONTINUOUS;
		LSM_I2C_Write(LSM303DLHC_ADDRESS_M,LSM303DLHC_RA_MR_REG_M,ctrl);
	}
	HAL_Delay(500);
}

void LSM_Mag_GetXYZ(int16_t* pData)
{
	uint8_t buffer[6];
	uint8_t i = 0;

	/*buffer[0] = LSM_I2C_Read(LSM303DLHC_ADDRESS_M,LSM303DLHC_RA_OUT_X_H_M);
	buffer[1] = LSM_I2C_Read(LSM303DLHC_ADDRESS_M,LSM303DLHC_RA_OUT_X_L_M);
	buffer[2] = LSM_I2C_Read(LSM303DLHC_ADDRESS_M,LSM303DLHC_RA_OUT_Y_H_M);
	buffer[3] = LSM_I2C_Read(LSM303DLHC_ADDRESS_M,LSM303DLHC_RA_OUT_Y_L_M);
	buffer[4] = LSM_I2C_Read(LSM303DLHC_ADDRESS_M,LSM303DLHC_RA_OUT_Z_H_M);
	buffer[5] = LSM_I2C_Read(LSM303DLHC_ADDRESS_M,LSM303DLHC_RA_OUT_Z_L_M);*/

	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Read(&hi2c1, LSM303DLHC_ADDRESS_M, LSM303DLHC_RA_OUT_X_H_M | LSM303DLHC_OUT_MULTI_READ, I2C_MEMADD_SIZE_8BIT, buffer, 6, 0x10000);
	if (status != HAL_OK) LSM_Error();
	else LSM_Success();

	for (i=0; i<3; i++)
	{
		pData[i] = ((int16_t)((uint16_t)buffer[2*i]<<8)+buffer[2*i+1]);
	}

}
