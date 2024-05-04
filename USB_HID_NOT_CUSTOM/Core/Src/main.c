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
#include "usb_device.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
//extern USBD_HandleTypeDef dds_usb;
USBD_HandleTypeDef hUsbDeviceFS;
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
	uint8_t i = 0;
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
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
 // uint8_t move_left_edge[] = {0, 127}
  uint8_t USB_Report_Buf_left_edge[] =
    {
		  0,   -127,  0, 0
  		  /*1,   0,  0, 0,
  		  1,   50,  0, 0,
  		  0,   0,  0, 0*/
    };
  uint8_t USB_Report_Buf_up_edge[] = {
		  0, 0, -127, 0
  };
  uint8_t windows_key_aka_GUI_key[] = {8, 0,0,0,0,0,0,0};
  uint8_t reboot_keys[] = {0, 0,0,0,0,0,0,0};
  uint8_t win_s[] = {8, 0, 0x16, 0, 0, 0, 0, 0};
  uint8_t note[] = {
		  0, 0, 0x36, 0, 0, 0, 0, 0,	//б
		  0, 0, 0x0e, 0, 0, 0, 0, 0,	//л
		  0, 0, 0x0d, 0, 0, 0, 0, 0,	//о
		  0, 0, 0x15, 0, 0, 0, 0, 0,	//к
		  0, 0, 0x1c, 0, 0, 0, 0, 0,	//н
		  0, 0, 0x0d, 0, 0, 0, 0, 0,	//о
		  0, 0, 0x11, 0, 0, 0, 0, 0		//т
  };
  uint8_t enter_key[] = {0, 0, 0x28, 0, 0, 0, 0, 0};
  uint8_t lCtrl_plus[] = {
		  1, 0, 0, 0, 0, 0, 0, 0,
		  1, 0, 0x57, 0, 0, 0, 0, 0
  };

  uint8_t ya_gey[] = {
		  0, 0, 0x1d, 0, 0, 0, 0, 0,	//z
		  0, 0, 0x2c, 0, 0, 0, 0, 0,	//space
		  0, 0, 0x18, 0, 0, 0, 0, 0,	//u
		  0, 0, 0x17, 0, 0, 0, 0, 0,	//e
		  0, 0, 0x14, 0, 0, 0, 0, 0		//q
  };


  HAL_Delay(500);
  		USBD_HID_SendReport(&hUsbDeviceFS, &win_s[0], 8);
  		//HAL_Delay(500);
  		//USBD_HID_SendReport(&hUsbDeviceFS, &reboot_keys[0], 8);
  		HAL_Delay(80);
  		USBD_HID_SendReport(&hUsbDeviceFS, &note[0], 8);
  		HAL_Delay(80);
  		USBD_HID_SendReport(&hUsbDeviceFS, &note[8], 8);
  		HAL_Delay(80);
  		USBD_HID_SendReport(&hUsbDeviceFS, &note[16], 8);
  		HAL_Delay(80);
  		USBD_HID_SendReport(&hUsbDeviceFS, &note[24], 8);
  		HAL_Delay(80);
  		USBD_HID_SendReport(&hUsbDeviceFS, &enter_key[0], 8);

  		/*for(i = 0; i<10; i = i+1){
  			HAL_Delay(50);
  			USBD_HID_SendReport(&hUsbDeviceFS, &lCtrl_plus[0], 8);
  			HAL_Delay(50);
  			  			USBD_HID_SendReport(&hUsbDeviceFS, &lCtrl_plus[8], 8);
  		}*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  /*for(i = 0; i<7; i = i+1){
		  HAL_Delay(100);
		  USBD_HID_SendReport(&hUsbDeviceFS, &USB_Report_Buf_left_edge[0], 4);
		  HAL_Delay(100);
		  USBD_HID_SendReport(&hUsbDeviceFS, &USB_Report_Buf_up_edge[0], 4);
	  }*/

	  		HAL_Delay(100);
	  		USBD_HID_SendReport(&hUsbDeviceFS, &ya_gey[0], 8);
	  		HAL_Delay(80);
	  		USBD_HID_SendReport(&hUsbDeviceFS, &ya_gey[8], 8);
	  		HAL_Delay(80);
	  		USBD_HID_SendReport(&hUsbDeviceFS, &ya_gey[16], 8);
	  		HAL_Delay(80);
	  		USBD_HID_SendReport(&hUsbDeviceFS, &ya_gey[24], 8);
	  		HAL_Delay(80);
	  		USBD_HID_SendReport(&hUsbDeviceFS, &ya_gey[32], 8);
	  		HAL_Delay(80);
	  		USBD_HID_SendReport(&hUsbDeviceFS, &enter_key[0], 8);


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
