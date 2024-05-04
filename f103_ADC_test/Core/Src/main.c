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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t nums_arr[4];
uint8_t ascii_arr[4];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void DDS_VAL_TO_NUMS(uint32_t val);
void DDS_NUM_TO_ASCII(void);
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
	uint32_t ADC_val1 = 0;
	uint32_t ADC_val2 = 0;
	uint32_t ADC_val3 = 0;
	HAL_StatusTypeDef adc_pol_ok;
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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_StatusTypeDef adc_start_ok;
  HAL_StatusTypeDef adc_stop_ok;
  HAL_StatusTypeDef adc_poll_ok;
  HAL_StatusTypeDef uart_rx_ok;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint8_t uart_tx_data[] = {0,0};
  uint8_t uart_question_sign_3d3v[] = {0x33, 0x2E, 0x33, 0x20, 0x56};
  uint8_t uart_question_sign_0v[] = {0x30, 0x56};
  uint8_t uart_CR_LF_sign[] = {0x0D, 0x0A};
  uint32_t adc_arr[] = {0,0,0};
  uint8_t i = 0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  adc_start_ok = HAL_ADC_Start(&hadc1);
	  //adc_poll_ok = HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  //ADC_val1 = HAL_ADC_GetValue(&hadc1);
		  adc_arr[0] = HAL_ADC_GetValue(&hadc1);
		  adc_arr[1] = HAL_ADC_GetValue(&hadc1);
		  adc_arr[2] = HAL_ADC_GetValue(&hadc1);
		  HAL_ADC_Stop(&hadc1);
	  /*for(i = 0; i<3; i=i+1){
		  if(adc_arr[i]>3000){
				HAL_UART_Transmit(&huart1, &uart_question_sign_3d3v[0], 5, HAL_MAX_DELAY);
				uart_rx_ok = HAL_UART_Transmit(&huart1, &uart_CR_LF_sign[0], 2, HAL_MAX_DELAY);
		  }else{
				HAL_UART_Transmit(&huart1, &uart_question_sign_0v[0], 2, HAL_MAX_DELAY);
				uart_rx_ok = HAL_UART_Transmit(&huart1, &uart_CR_LF_sign[0], 2, HAL_MAX_DELAY);
		  }
	  }*/
//HAL_UART_Transmit(&huart1, &uart_CR_LF_sign[0], 2, HAL_MAX_DELAY);









DDS_VAL_TO_NUMS(adc_arr[0]);
DDS_NUM_TO_ASCII();

HAL_UART_Transmit(&huart1, &ascii_arr[0], 4, HAL_MAX_DELAY);
HAL_UART_Transmit(&huart1, &uart_CR_LF_sign[0], 2, HAL_MAX_DELAY);

DDS_VAL_TO_NUMS(adc_arr[1]);
DDS_NUM_TO_ASCII();

HAL_UART_Transmit(&huart1, &ascii_arr[0], 4, HAL_MAX_DELAY);
HAL_UART_Transmit(&huart1, &uart_CR_LF_sign[0], 2, HAL_MAX_DELAY);

DDS_VAL_TO_NUMS(adc_arr[2]);
DDS_NUM_TO_ASCII();

HAL_UART_Transmit(&huart1, &ascii_arr[0], 4, HAL_MAX_DELAY);
HAL_UART_Transmit(&huart1, &uart_CR_LF_sign[0], 2, HAL_MAX_DELAY);

HAL_UART_Transmit(&huart1, &uart_CR_LF_sign[0], 2, HAL_MAX_DELAY);

HAL_Delay(1000);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	/*HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	ADC_val1 = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	ADC_val2 = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	ADC_val3 = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);*/
	//HAL_UART_Transmit_DMA(&huart1, const uint8_t *pData, uint16_t Size)
}


void DDS_num_to_ASCII(uint32_t number, uint8_t *result){

}
void DDS_VAL_TO_NUMS(uint32_t val){
	nums_arr[0] = val/1000;
	nums_arr[1] = (val%1000)/100;
	nums_arr[2] = ((val%1000)%100)/10;
	nums_arr[3] = ((val%1000)%100)%10;
}

void DDS_NUM_TO_ASCII(void){
	for(int i = 0; i<4; i=i+1){
		if(nums_arr[i] == 0){
			ascii_arr[i] = 0x30;
			continue;
		}
		if(nums_arr[i] == 1){
			ascii_arr[i] = 0x31;
			continue;
		}
		if(nums_arr[i] == 2){
			ascii_arr[i] = 0x32;
			continue;
		}
		if(nums_arr[i] == 3){
			ascii_arr[i] = 0x33;
			continue;
		}
		if(nums_arr[i] == 4){
			ascii_arr[i] = 0x34;
			continue;
		}
		if(nums_arr[i] == 5){
			ascii_arr[i] = 0x35;
			continue;
		}
		if(nums_arr[i] == 6){
			ascii_arr[i] = 0x36;
			continue;
		}
		if(nums_arr[i] == 7){
			ascii_arr[i] = 0x37;
			continue;
		}
				if(nums_arr[i] == 8){
			ascii_arr[i] = 0x38;
			continue;
		}
		if(nums_arr[i] == 9){
			ascii_arr[i] = 0x39;
			continue;
		}
	}

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
