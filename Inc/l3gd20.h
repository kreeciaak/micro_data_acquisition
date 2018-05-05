/*
 * l3gd20.h
 *
 *  Created on: 14 kwi 2018
 *      Author: User
 */


#ifndef L3GD20_H_
#define L3GD20_H_

#define READWRITE_CMD		 0x80
#define MULTIPLEBYTE_CMD	 0x40
#define CS_ON				 HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET)
#define CS_OFF				 HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET)
#define DUMMY_BYTE			 0x00


#define L3GD20_WHO_AM_I      0x0F

#define L3GD20_CTRL_REG1     0x20
#define L3GD20_CTRL_REG2     0x21
#define L3GD20_CTRL_REG3     0x22
#define L3GD20_CTRL_REG4     0x23
#define L3GD20_CTRL_REG5     0x24
#define L3GD20_REFERENCE     0x25
#define L3GD20_OUT_TEMP      0x26
#define L3GD20_STATUS_REG    0x27

#define L3GD20_OUT_X_L       0x28
#define L3GD20_OUT_X_H       0x29
#define L3GD20_OUT_Y_L       0x2A
#define L3GD20_OUT_Y_H       0x2B
#define L3GD20_OUT_Z_L       0x2C
#define L3GD20_OUT_Z_H       0x2D

#define L3GD20_FIFO_CTRL_REG 0x2E
#define L3GD20_FIFO_SRC_REG  0x2F

#define L3GD20_INT1_CFG      0x30
#define L3GD20_INT1_SRC      0x31
#define L3GD20_INT1_THS_XH   0x32
#define L3GD20_INT1_THS_XL   0x33
#define L3GD20_INT1_THS_YH   0x34
#define L3GD20_INT1_THS_YL   0x35
#define L3GD20_INT1_THS_ZH   0x36
#define L3GD20_INT1_THS_ZL   0x37
#define L3GD20_INT1_DURATION 0x38

//CTRL_REG1
#define L3GD20_DR_95HZ		 0x00
#define L3GD20_DR_190HZ		 0x40
#define L3GD20_DR_380HZ		 0x80
#define L3GD20_DR_760HZ		 0xC0

#define L3GD20_BW_LOW		 0x00
#define L3GD20_BW_MED		 0x10
#define L3GD20_BW_HIGH		 0x20
#define L3GD20_BW_VHIGH		 0x30

#define L3GD20_NORM_MODE_EN	 0x08
#define L3GD20_XYZ_EN		 0x07

//CTRL_REG2
#define L3GD20_HPM_NOR_M_R	 0x00
#define L3GD20_HPM_REF_SIGN	 0x10
#define L3GD20_HPM_NOR_M	 0x20
#define L3GD20_HPM_AUTORES	 0x30

#define L3GD20_HPCF_10		 0x00
#define L3GD20_HPCF_9		 0x01
#define L3GD20_HPCF_8		 0x02
#define L3GD20_HPCF_7		 0x03
#define L3GD20_HPCF_6		 0x04
#define L3GD20_HPCF_5		 0x05
#define L3GD20_HPCF_4		 0x06
#define L3GD20_HPCF_3		 0x07
#define L3GD20_HPCF_2		 0x08
#define L3GD20_HPCF_1		 0x09

//CTRL_REG3
#define L3GD20_I2_DRDY_EN	 0x08

//CTRL_REG4
#define L3GD20_FS_250DPS	 0x00
#define L3GD20_FS_500DPS	 0x10
#define L3GD20_FS_2000DPS	 0x20
#define L3GD20_FS_2000DPS	 0x30




#endif /* L3GD20_H_ */

void L3G_Error(void);
void L3G_Success(void);
uint8_t L3G_SPIx_WriteRead(uint8_t Byte);
void L3G_Gyro_IO_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);
void L3G_Gyro_IO_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToRead);
uint8_t L3G_Gyro_ReadID(void);
void L3G_Gyro_Ini(void);
void L3G_Gyro_GetXYZ(int16_t* pData);
