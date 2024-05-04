/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include "stdlib.h"

//#include "ssd1306_conf.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
	uint8_t acc_set_pmu_normal = 0x11;	// cmd for CMD reg
	uint8_t reg_pmu_status = 0x03;
	uint8_t reg_err = 0x02;
	uint8_t reg_status = 0x1B;
	uint8_t reg_data_acc_z_LSB = 0x16;
	uint8_t reg_data_acc_z_MSB = 0x17;


	//READ BUFFs
	uint8_t pmu_status0 = 99;
	uint8_t errors = 101;
	uint8_t status = 102;
	uint8_t acc_z_LSB = 103;
	uint8_t acc_z_MSB = 104;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);


HAL_StatusTypeDef DDS_ACC_init(void);


/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  /* USER CODE BEGIN 2 */

  ssd1306_Init();
  ssd1306_Fill(White);
  ssd1306_UpdateScreen();


  char char_v[] = {72, 105};
  ssd1306_SetCursor(5, 5);
  ssd1306_WriteString(&char_v[0], Font_11x18, Black);
  ssd1306_UpdateScreen();
  HAL_Delay(300);




  HAL_StatusTypeDef acc_init_result = DDS_ACC_init();









  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);	//TURN LED OFF

  //uint8_t sent_buff[] = {0x6D};
 // uint8_t *pntr_sent_buff = &sent_buff[0];
 // HAL_Delay(1000);
/*  HAL_StatusTypeDef func_result1 = HAL_I2C_Master_Transmit(&hi2c1, 0b11010001, (uint8_t*)0x6D, 1, HAL_MAX_DELAY);

  uint8_t gotten_buff[] = {13};
  uint8_t *pntr_gotten_buff = &gotten_buff[0];

  HAL_StatusTypeDef func_result2 = HAL_I2C_Master_Receive(&hi2c1, 0b11010000, pntr_gotten_buff, 1, 1000);*/
 /* if( HAL_I2C_IsDeviceReady(&hi2c1, 0b1101000, 3, HAL_MAX_DELAY) == (HAL_OK|HAL_BUSY|HAL_ERROR)){
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);	//TURN LED OFF
  }*/




  //uint16_t acc_z = 0b101;
  char acc_z_string[10];
  uint8_t temp = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		HAL_StatusTypeDef result2 =  HAL_I2C_Mem_Read(&hi2c1, 0b11010001, reg_status, 1, &status, 1, HAL_MAX_DELAY);
		if((status==144 )| (status==128)){
			//HAL_StatusTypeDef result3 =  HAL_I2C_Mem_Read(&hi2c1, 0b11010001, reg_data_acc_z_LSB, 1, &acc_z_LSB, 1, HAL_MAX_DELAY);
			HAL_StatusTypeDef result4 =  HAL_I2C_Mem_Read(&hi2c1, 0b11010001, reg_data_acc_z_MSB, 1, &acc_z_MSB, 1, HAL_MAX_DELAY);
			//acc_z = (acc_z_MSB << 8) | acc_z_LSB;
			//sprintf(acc_z_string, "%d", acc_z);
			if(acc_z_MSB < 192){
				acc_z_string[0] = '-';
				acc_z_MSB = 192 - acc_z_MSB;
			}
			else{
				acc_z_MSB = acc_z_MSB - 192;
				acc_z_string[0] = '+';
			}
			sprintf(&acc_z_string[1], "%d", acc_z_MSB);
			  ssd1306_SetCursor(5, 5);
			  ssd1306_WriteString(&acc_z_string[0], Font_11x18, Black);
			  ssd1306_UpdateScreen();
		}


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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
HAL_StatusTypeDef DDS_ACC_init(void){
	HAL_StatusTypeDef result0 =  HAL_I2C_Mem_Read(&hi2c1, 0b11010001, reg_pmu_status, 1, &pmu_status0, 1, HAL_MAX_DELAY);
	if(pmu_status0==0){
			HAL_StatusTypeDef result1 =  HAL_I2C_Mem_Write(&hi2c1, 0b11010000, 0x7E, 1, &acc_set_pmu_normal, 1, HAL_MAX_DELAY);
			HAL_StatusTypeDef result2 =  HAL_I2C_Mem_Read(&hi2c1, 0b11010001, reg_pmu_status, 1, &pmu_status0, 1, HAL_MAX_DELAY);
	}
	else if((pmu_status0==144) | (pmu_status0==16))
		return HAL_OK;
	else
		return HAL_ERROR;

	HAL_StatusTypeDef result3 =  HAL_I2C_Mem_Read(&hi2c1, 0b11010001, reg_err, 1, &errors, 1, HAL_MAX_DELAY);
	if(errors==0)
		return HAL_OK;
	else
		return HAL_ERROR;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
