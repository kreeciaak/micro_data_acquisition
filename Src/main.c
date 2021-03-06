
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h" //biblioteka do transmisji
#include "lsm303dlhc.h"
#include "l3gd20.h"
#include "vector.h"
#include "calculations.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim10;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
char str1[350] = {0};

int16_t AccelData[3], GyroData[3], MagData[3];
Result result;
Vector3f fAccelData, fGyroData, fMagData, AccelOffset, GyroOffset, MagOffset, RawAnglesDeg;
float z_or_corr = 0, acc_mag = 0;
int flag = -1;
int Procedureflag = 4; //1 - XYorientationCorrection, 3 - ZOrientationCorrection, 4 - Main program
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM10_Init(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM10_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  LSM_Accel_Ini();
  LSM_Mag_Ini();
  L3G_Gyro_Ini();

  LSM_Accel_GetXYZ(AccelData);
  L3G_Gyro_GetXYZ(GyroData);
  LSM_Mag_GetXYZ(MagData);

//  if (Procedureflag == 2)
//  {
//	  AccelOffsetCalculation(AccelOffset);
//	  acc_mag = Norm(VectorTo2PowSum(AccelOffset));
//	  HAL_GPIO_WritePin(LED_green_GPIO_Port, LED_green_Pin, 0);
//	  HAL_Delay(200);
//
//	  GyroOffsetCalculation(GyroOffset);
//	  HAL_GPIO_WritePin(LED_green_GPIO_Port, LED_green_Pin, 0);
//	  HAL_Delay(200);
//
//	  MagOffsetCalculation(MagOffset);
//	  HAL_GPIO_WritePin(LED_green_GPIO_Port, LED_green_Pin, 0);
//	  HAL_Delay(200);
//  }

  HAL_TIM_Base_Start_IT(&htim10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  intToVector3f(AccelData, fAccelData);
	  intToVector3f(GyroData, fGyroData);
	  intToVector3f(MagData, fMagData);

	  if (Procedureflag == 1)
	  {
		  XYorietationCalibration(RawAnglesDeg);
		  sprintf(str1, "%06.4f;%06.4f;%06.4f;\n\r", RawAnglesDeg[0], RawAnglesDeg[1], RawAnglesDeg[2]);
		  CDC_Transmit_FS((uint8_t*)str1, strlen(str1));
		  HAL_Delay(20);
	  }


  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 384;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM1_UP_TIM10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM10 init function */
static void MX_TIM10_Init(void)
{

  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 9999;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 23;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_Pin|CS2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_green_Pin|LED_orange_Pin|LED_red_Pin|LED_blue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_MAG_Pin DRDY_ACC_Pin DRDY_MPU_Pin DRDY_GYRO_Pin */
  GPIO_InitStruct.Pin = DRDY_MAG_Pin|DRDY_ACC_Pin|DRDY_MPU_Pin|DRDY_GYRO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_Pin CS2_Pin */
  GPIO_InitStruct.Pin = CS_Pin|CS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_EXT_Pin */
  GPIO_InitStruct.Pin = BUTTON_EXT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_EXT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_green_Pin LED_orange_Pin LED_red_Pin LED_blue_Pin */
  GPIO_InitStruct.Pin = LED_green_Pin|LED_orange_Pin|LED_red_Pin|LED_blue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if ((htim->Instance == TIM10) && (flag == 1))
	{
		if (Procedureflag == 3)
		{
			z_or_corr = ZorientationCorrection(fAccelData, fGyroData, fMagData);
			sprintf(str1, "%06.4f;\n\r", z_or_corr);
			CDC_Transmit_FS((uint8_t*)str1, strlen(str1));
		}


		if (Procedureflag == 4)
		{
			RawToResult(fAccelData, fGyroData, fMagData, result);
			if (result[0][2] != 0)
			{
				//sprintf(str1, "%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;\n\r", result[0][0],result[0][1],result[0][2],result[1][0],result[1][1],result[1][2],result[2][0],result[2][1],result[2][2],result[3][0],result[3][1],result[3][2],result[4][0],result[4][1],result[4][2],result[5][0],result[5][1],result[5][2],result[6][0],result[6][1],result[6][2],result[7][0],result[7][1],result[7][2],result[8][0],result[8][1],result[8][2],result[9][0],result[9][1],result[9][2]);
				//sprintf(str1, "%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;\n\r", result[0][0],result[0][1],result[0][2],result[1][0],result[1][1],result[1][2],result[2][0],result[2][1],result[2][2],result[3][0],result[3][1],result[3][2],result[4][0],result[4][1],result[4][2],result[5][0],result[5][1],result[5][2],result[6][0],result[6][1],result[6][2],result[7][0],result[7][1],result[7][2],result[8][0],result[8][1],result[8][2],result[9][0],result[9][1],result[9][2],result[10][0],result[10][1],result[10][2],result[11][0],result[11][1],result[11][2],result[12][0],result[12][1],result[12][2],result[13][0],result[13][1],result[13][2],result[14][0],result[14][1],result[14][2],result[15][0],result[15][1],result[15][2],result[16][0],result[16][1],result[16][2],result[17][0],result[17][1],result[17][2],result[18][0],result[18][1],result[18][2],result[19][0],result[19][1],result[19][2],result[20][0],result[20][1],result[20][2],result[21][0],result[21][1],result[21][2],result[22][0],result[22][1],result[22][2],result[23][0],result[23][1],result[23][2],result[24][0],result[24][1],result[24][2],result[25][0],result[25][1],result[25][2],result[26][0],result[26][1],result[26][2],result[27][0],result[27][1],result[27][2],result[28][0],result[28][1],result[28][2],result[29][0],result[29][1],result[29][2],result[30][0],result[30][1],result[30][2],result[31][0],result[31][1],result[31][2],result[32][0],result[32][1],result[32][2],result[33][0],result[33][1],result[33][2],result[34][0],result[34][1],result[34][2],result[35][0],result[35][1],result[35][2]);
				//sprintf(str1, "%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;\n\r", result[0][0],result[0][1],result[0][2],result[1][0],result[1][1],result[1][2],result[2][0],result[2][1],result[2][2],result[3][0],result[3][1],result[3][2],result[4][0],result[4][1],result[4][2]);
				//sprintf(str1, "%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;\n\r", fGyroData[0],fGyroData[1],fGyroData[2],result[0][0],result[0][1],result[0][2]);
				//sprintf(str1, "%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;%06.4f;\n\r", result[0][0],result[0][1],result[0][2],result[1][0],result[1][1],result[1][2]);
				sprintf(str1, "%06.4f;%06.4f;%06.4f;\n\r",result[0][0],result[0][1],result[0][2]);
				//sprintf(str1, "%06.4f;%06.4f;%06.4f;\n\r", AccelOffset[0], AccelOffset[1], AccelOffset[2]);
				//sprintf(str1, "%06.4f\n\r", result[0][0]);
				CDC_Transmit_FS((uint8_t*)str1, strlen(str1));

				//sprintf(str2, "%06.4f;%06.4f;%06.4f;\n\r",result[10][0],result[10][1],result[10][2]);
				//CDC_Transmit_FS((uint8_t*)str2, strlen(str2));

			}
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
  if ((Procedureflag != 1) && (Procedureflag !=2))
  {
	  if (GPIO_Pin == DRDY_ACC_Pin)
	  {
		  LSM_Accel_GetXYZ(AccelData);
		  //sprintf(str1, "%06.4f\n\r", fAccelData[0]);
		  //CDC_Transmit_FS((uint8_t*)str1, strlen(str1));
	  }

	  if (GPIO_Pin == DRDY_MAG_Pin)
	  {
		  LSM_Mag_GetXYZ(MagData);
	  }

	  if (GPIO_Pin == DRDY_GYRO_Pin)
	  {
		  L3G_Gyro_GetXYZ(GyroData);
	  }
  }

  if (GPIO_Pin == BUTTON_EXT_Pin)
  {
	  flag = -flag;
	  HAL_GPIO_TogglePin(LED_green_GPIO_Port, LED_green_Pin);
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
