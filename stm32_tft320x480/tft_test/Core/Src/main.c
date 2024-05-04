/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void init_tft(void);
void coord_to_MSByte_LSByte(uint16_t *dec, uint8_t *MSB, uint8_t *LSB);
void send_coordinates(uint16_t XS_dec, uint16_t XE_dec, uint16_t YS_dec, uint16_t YE_dec);
void color_to_MSByte_LSByte(uint16_t two_bytes, uint8_t *MSByte, uint8_t *LSByte);
void fill_by_color(uint16_t color, uint16_t length);

void fill_by_data(uint8_t image_data[], uint16_t length/*max65535*/);




//void fill_by_color(uint16_t color){

//}
//void fill_by_data();
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
  MX_DMA_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

//  uint16_t XS_dec = 0;
//  uint16_t XE_dec = 320;
//  uint16_t YS_dec = 0;
//  uint16_t YE_dec = 480;
//
//
//
//  uint8_t image_data[25600];
//
//  uint16_t i;
//
//  for(i = 0; i < 25600; i = i+1){
//	image_data[i] = i;
//  }
HAL_Delay(10);
  init_tft();

  //HAL_Delay(100);

  send_coordinates(0, 320, 0, 100);
  fill_by_color(0xf800, 320*100);	//102 ok but 103 too much for second value



  send_coordinates(0, 320, 100, 200);
  fill_by_color(0xef01, 320*100);	//102 ok but 103 too much for second value



  send_coordinates(0, 320, 200, 300);
  fill_by_color(0xff80, 320*100);	//102 ok but 103 too much for second value



  send_coordinates(0, 320, 300, 400);
  fill_by_color(0x6789, 320*100);	//102 ok but 103 too much for second value


  send_coordinates(0, 320, 400, 480);
  fill_by_color(0x001f, 320*80);	//102 ok but 103 too much for second value




  send_coordinates(120, 220, 190, 290);
  uint8_t test_data[20000];
  uint16_t i;
  for(i = 0; i < 10000; i = i+1){
	  test_data[i] = i;
  }
  for(i = 10000; i < 20000; i = i+1){
	  test_data[i] = i*20;
  }
  fill_by_data(&test_data[0], 20000);






  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DC_RS_Pin|RESET_Pin|CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : DC_RS_Pin RESET_Pin CS_Pin */
  GPIO_InitStruct.Pin = DC_RS_Pin|RESET_Pin|CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//функция делает первую настройку
void init_tft(void){
	  uint8_t slpout_cmd = 0x11;
	  uint8_t noron_cmd = 0x13;
	  uint8_t dispon_cmd = 0x29;
	  uint8_t interf_pix_format_cmd = 0x3a;
	  uint8_t interf_pix_format_prm = 0b01010101;


	  HAL_GPIO_WritePin(GPIOA, RESET_Pin, 0);
	  HAL_GPIO_WritePin(GPIOA, RESET_Pin, 1);
	  HAL_GPIO_WritePin(GPIOA, DC_RS_Pin, 0);

	  //вышел из сна
	  HAL_GPIO_WritePin(GPIOA, CS_Pin, 0);
	  HAL_SPI_Transmit_DMA(&hspi1, &slpout_cmd, 1);
	  HAL_GPIO_WritePin(GPIOA, CS_Pin, 1);
	  //

	  //вкл нормальный режим
	  HAL_GPIO_WritePin(GPIOA, CS_Pin, 0);
	  HAL_SPI_Transmit_DMA(&hspi1, &noron_cmd, 1);
	  HAL_GPIO_WritePin(GPIOA, CS_Pin, 1);
	  //

//	  //вкл дисплей
//	  HAL_GPIO_WritePin(GPIOA, CS_Pin, 0);
//	  HAL_SPI_Transmit_DMA(&hspi1, &dispon_cmd, 1);
//	  HAL_GPIO_WritePin(GPIOA, CS_Pin, 1);
//	  //





	  //установить параметр пикселя
	  HAL_GPIO_WritePin(GPIOA, DC_RS_Pin, 0);
	  HAL_GPIO_WritePin(GPIOA, CS_Pin, 0);
	  HAL_SPI_Transmit_DMA(&hspi1, &interf_pix_format_cmd, 1);
	  HAL_GPIO_WritePin(GPIOA, CS_Pin, 1);
	  //
	  //параметры (16 бит 565)
	  HAL_GPIO_WritePin(GPIOA, DC_RS_Pin, 1);
	  HAL_GPIO_WritePin(GPIOA, CS_Pin, 0);
	  HAL_SPI_Transmit_DMA(&hspi1, &interf_pix_format_prm, 1);
	  HAL_GPIO_WritePin(GPIOA, CS_Pin, 1);
	  //




	  //вкл дисплей
	  HAL_GPIO_WritePin(GPIOA, DC_RS_Pin, 0);
	  HAL_GPIO_WritePin(GPIOA, CS_Pin, 0);
	  HAL_SPI_Transmit_DMA(&hspi1, &dispon_cmd, 1);
	  HAL_GPIO_WritePin(GPIOA, CS_Pin, 1);
	  //
}


//****************************************************************************
//есть функция устанавливающая Хстарт и Хенд, и функция для Устарт и Уенд
//параметрами вышеупомянутой функции являются Хстарт Хенд Устарт Уенд
//каждый из этих параметров передается двумя 8битными числами
//и если координата больше 255, то появляется старший разряд
//этот разряд помещается в MSB, а в LSB помещается остаток
//****************************************************************************
//данная функция преобразует 16 битное число в 2 восьмибитных
void coord_to_MSByte_LSByte(uint16_t *dec, uint8_t *MSB, uint8_t *LSB){
	if(*dec < 256){
		*MSB = 0;
		*LSB = *dec;
	}else{
		*MSB = 1;
		*LSB = *dec - 256;
	}
}





void send_coordinates(uint16_t XS_dec, uint16_t XE_dec, uint16_t YS_dec, uint16_t YE_dec){
	  uint8_t column_set_cmd = 0x2a;
	  uint8_t raw_set_cmd = 0x2b;

	  uint8_t XS_MSB, XS_LSB,
	  	  	  XE_MSB, XE_LSB,
			  YS_MSB, YS_LSB,
			  YE_MSB, YE_LSB;

	  coord_to_MSByte_LSByte(&XS_dec, &XS_MSB, &XS_LSB);
	  coord_to_MSByte_LSByte(&XE_dec, &XE_MSB, &XE_LSB);
	  coord_to_MSByte_LSByte(&YS_dec, &YS_MSB, &YS_LSB);
	  coord_to_MSByte_LSByte(&YE_dec, &YE_MSB, &YE_LSB);



	  //установить столбец
	  	HAL_GPIO_WritePin(GPIOA, DC_RS_Pin, 0);
	    HAL_GPIO_WritePin(GPIOA, CS_Pin, 0);
	    HAL_SPI_Transmit_DMA(&hspi1, &column_set_cmd, 1);
	    HAL_GPIO_WritePin(GPIOA, CS_Pin, 1);
	    //
	    //parameters
	    HAL_GPIO_WritePin(GPIOA, DC_RS_Pin, 1);
	    HAL_GPIO_WritePin(GPIOA, CS_Pin, 0);
	    HAL_SPI_Transmit_DMA(&hspi1, &XS_MSB, 1);
	    HAL_GPIO_WritePin(GPIOA, CS_Pin, 1);
	    HAL_GPIO_WritePin(GPIOA, CS_Pin, 0);
	    HAL_SPI_Transmit_DMA(&hspi1, &XS_LSB, 1);
	    HAL_GPIO_WritePin(GPIOA, CS_Pin, 1);
	    HAL_GPIO_WritePin(GPIOA, CS_Pin, 0);
	    HAL_SPI_Transmit_DMA(&hspi1, &XE_MSB, 1);
	    HAL_GPIO_WritePin(GPIOA, CS_Pin, 1);
	    HAL_GPIO_WritePin(GPIOA, CS_Pin, 0);
	    HAL_SPI_Transmit_DMA(&hspi1, &XE_LSB, 1);
	    HAL_GPIO_WritePin(GPIOA, CS_Pin, 1);
	    //

	    //установить строку
	    HAL_GPIO_WritePin(GPIOA, DC_RS_Pin, 0);
	    HAL_GPIO_WritePin(GPIOA, CS_Pin, 0);
	    HAL_SPI_Transmit_DMA(&hspi1, &raw_set_cmd, 1);
	    HAL_GPIO_WritePin(GPIOA, CS_Pin, 1);
	    //
	    //параметры
	    HAL_GPIO_WritePin(GPIOA, DC_RS_Pin, 1);
	    HAL_GPIO_WritePin(GPIOA, CS_Pin, 0);
	    HAL_SPI_Transmit_DMA(&hspi1, &YS_MSB, 1);
	    HAL_GPIO_WritePin(GPIOA, CS_Pin, 1);
	    HAL_GPIO_WritePin(GPIOA, CS_Pin, 0);
	    HAL_SPI_Transmit_DMA(&hspi1, &YS_LSB, 1);
	    HAL_GPIO_WritePin(GPIOA, CS_Pin, 1);
	    HAL_GPIO_WritePin(GPIOA, CS_Pin, 0);
	    HAL_SPI_Transmit_DMA(&hspi1, &YE_MSB, 1);
	    HAL_GPIO_WritePin(GPIOA, CS_Pin, 1);
	    HAL_GPIO_WritePin(GPIOA, CS_Pin, 0);
	    HAL_SPI_Transmit_DMA(&hspi1, &YE_LSB, 1);
	    HAL_GPIO_WritePin(GPIOA, CS_Pin, 1);
	    //
}

//here is problem
void color_to_MSByte_LSByte(uint16_t two_bytes, uint8_t *MSByte, uint8_t *LSByte){
	*MSByte = two_bytes >> 8;
	*LSByte = two_bytes - 0xFF00;
}




void fill_by_color(uint16_t color, uint16_t length){
	uint8_t MSByte, LSByte;
	color_to_MSByte_LSByte(color, &MSByte, &LSByte);
	uint8_t memwr_cmd = 0x2c;
	//MSByte = 0x00;
	//LSByte = 0xf8;
	uint16_t i = 0;	//16
	uint8_t data[length*2];	//multiple two because 1 pix has two bytes, thus cycle must be 2 times long
	for(i = 0; i < length*2; i = i+2){
		data[i] = LSByte;
		data[i+1] = MSByte;
	}


	 //записать в память
	  HAL_GPIO_WritePin(GPIOA, DC_RS_Pin, 0);
	  HAL_GPIO_WritePin(GPIOA, CS_Pin, 0);
	  HAL_SPI_Transmit_DMA(&hspi1, &memwr_cmd, 1);
	  HAL_GPIO_WritePin(GPIOA, CS_Pin, 1);
	  //передача цвета в память дисплея
	  HAL_GPIO_WritePin(GPIOA, DC_RS_Pin, 1);
	  HAL_GPIO_WritePin(GPIOA, CS_Pin, 0);

	  //HAL_SPI_Transmit(&hspi1, &data[0], length*2, 10);
	  HAL_SPI_Transmit_DMA(&hspi1, &data[0], length*2);
	  HAL_Delay(15);

	  HAL_GPIO_WritePin(GPIOA, CS_Pin, 1);
}

void fill_by_data(uint8_t image_data[], uint16_t length/*max65535*/){
	  uint8_t memwr_cmd = 0x2c;

	  //записать в память
	  HAL_GPIO_WritePin(GPIOA, DC_RS_Pin, 0);
	  HAL_GPIO_WritePin(GPIOA, CS_Pin, 0);
	  HAL_SPI_Transmit_DMA(&hspi1, &memwr_cmd, 1);
	  HAL_GPIO_WritePin(GPIOA, CS_Pin, 1);

	  //передача цвета в память дисплея
	  HAL_GPIO_WritePin(GPIOA, DC_RS_Pin, 1);
	  HAL_GPIO_WritePin(GPIOA, CS_Pin, 0);
	  HAL_SPI_Transmit_DMA(&hspi1, &image_data[0], length);
	  HAL_Delay(15);
	  HAL_GPIO_WritePin(GPIOA, CS_Pin, 1);
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
