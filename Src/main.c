/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
#include "w2812b.h"
#include <stdio.h> //UART
#include "microSD.h"


/* Private typedef -----------------------------------------------------------*/
//GPIO_InitTypeDef GPIO_InitStruct;

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
char buffer[100];

/* Timer handler declaration */

/* Timer Output Compare Configuration Structure declaration */
TIM_OC_InitTypeDef sConfig;
UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

void transmit_uart(char *string){
	uint8_t len = strlen(string);
	HAL_UART_Transmit(&huart2, (uint8_t*) string, len, 200);
}

FATFS fs;
FATFS *pfs;
FIL fil;
FRESULT fres;
DWORD fre_clust;
uint32_t totalSpace, freeSpace;

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();		// Configure system clock to 180MHz

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  // microSD Setup
  microSD_init();
  MX_FATFS_Init();

  // WS2812B Setup
  LED_Init();
  LED_set_color_all(0x00, 0x00, 0x00);	//Set color order of array. Ex: R0,G0,B0,R1,G1,B1
  LED_update(1);

  /* Wait for microSD to initialize */
  HAL_Delay(500);

  /* Check if microSD is connected physically */
	while (!check_microSD_conn()){
  	transmit_uart("MicroSD card not detected!\r\n");
  	HAL_Delay(1000);
	}
	transmit_uart("MicroSD card detected!\r\n");

	/* Waiting for the Micro SD module to initialize */
	HAL_Delay(500);

	fres = f_mount(&fs, "", 0);
	if (fres == FR_OK) {
		transmit_uart("Micro SD card is mounted successfully!\r\n");
	} else if (fres != FR_OK) {
		transmit_uart("Micro SD card's mount error!\r\n");
	}

	// FA_OPEN_APPEND opens file if it exists and if not then creates it,
	// the pointer is set at the end of the file for appending FA_OPEN_APPEND | FA_WRITE | FA_READ
  /*
	if (f_open(&fil, "log-file.txt", FA_OPEN_APPEND | FA_WRITE | FA_READ) != FR_OK) {
    Error_Handler(OPEN_ERROR);
  }
	*/

	/*
	fres = f_open(&fil, "log-file.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
	if (fres == FR_OK) {
		transmit_uart("File opened for reading and checking the free space.\r\n");
	} else if (fres != FR_OK) {
		transmit_uart("File was not opened for reading and checking the free space!\r\n");
	}
	*/
	if(f_open(&fil, "first.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE) != FR_OK) {
		transmit_uart("FAIL - File not opened\r\n");
		Error_Handler(OPEN_ERROR);
	} else {
		transmit_uart("PASS - File successfully opened\r\n");
	}


	fres = f_getfree("", &fre_clust, &pfs);
	totalSpace = (uint32_t) ((pfs->n_fatent - 2) * pfs->csize * 0.5);
	freeSpace = (uint32_t) (fre_clust * pfs->csize * 0.5);

	char mSz[12];
	sprintf(mSz, "%lu", freeSpace);

	if (fres == FR_OK) {
		transmit_uart("The free space is: ");
		transmit_uart(mSz);
		transmit_uart("\r\n");
	} else if (fres != FR_OK) {
		transmit_uart("The free space could not be determined!\r\n");
	}

	for (uint8_t i = 0; i < 10; i++) {
		f_puts("This text is written in the file.\r\n", &fil);
	}

	fres = f_close(&fil);
	if (fres == FR_OK) {
		transmit_uart("The file is closed.\r\n");
	} else if (fres != FR_OK) {
		transmit_uart("The file was not closed.\r\n");
	}

	/* Open file to read */
	/*
	fres = f_open(&fil, "log-file.txt", FA_READ);
	if (fres == FR_OK) {
		transmit_uart("File opened for reading.\r\n");
	} else if (fres != FR_OK) {
		transmit_uart("File was not opened for reading!\r\n");
	}
	*/
	if(f_open(&fil, "first.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE) != FR_OK) {
		transmit_uart("FAIL - File not opened\r\n");
		Error_Handler(OPEN_ERROR);
	} else {
		transmit_uart("PASS - File successfully opened for reading\r\n");
	}

	transmit_uart("--------------------------------------\n\r");
	while (f_gets(buffer, sizeof(buffer), &fil)) {
		char mRd[100];
		sprintf(mRd, "%s", buffer);
		transmit_uart(mRd);
		transmit_uart("\r");
	}
	transmit_uart("--------------------------------------\n\r");

	/* Close file */
	fres = f_close(&fil);
	if (fres == FR_OK) {
		transmit_uart("The file is closed.\r\n");
	} else if (fres != FR_OK) {
		transmit_uart("The file was not closed.\r\n");
	}

	f_mount(NULL, "", 1);
	if (fres == FR_OK) {
		transmit_uart("The Micro SD card is unmounted!\r\n");
	} else if (fres != FR_OK) {
		transmit_uart("The Micro SD was not unmounted!");
	}


	/*
	// Check if microSD is mounted
	if (check_microSD_mount()) {
		transmit_uart("MicroSD card is mounted successfully!\r\n");
	} else {
		transmit_uart("MicroSD card's mount error!\r\n");
	}

	if (open_file()) {
		transmit_uart("File opened for reading and checking the free space.\r\n");
	} else {
		transmit_uart("File was not opened for reading and checking the free space!\r\n");
	}
	*/

  /* Infinite loop */
  while (1) {
  	// Check if microSD card is detected
  	if (check_microSD_conn()){

  		// Send pulse lighting to W2812B LED Strip
  		pulse();
  	} else {

  		// Loop until microSD card is detected
  		while (!(check_microSD_conn())){
  			transmit_uart("MicroSD card not detected!\r\n");
  			HAL_Delay(1000);
  		}
			transmit_uart("MicroSD card detected!\r\n");
  	}
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 360
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            PLL_R                          = 2
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
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
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler(SYSCONF_ERROR1);
  }

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler(SYSCONF_ERROR2);
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void){
  // GPIO Ports Clock Enable
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Configure GPIO pin Output Level
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  // Configure GPIO pin : B1_Pin
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  // Configure GPIO pin : LD2_Pin
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : SD_CS_Pin */
	GPIO_InitStruct.Pin = SD_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler(UART_ERROR);
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(uint8_t ERROR)
{
  while (1) {
    /* Turn LED2 on */
  	switch(ERROR){
  	case SYSCONF_ERROR1:
  		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  		break;
  	case SYSCONF_ERROR2:
  		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  		break;
  	case TIM_INIT_ERROR:
  	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  	  break;
  	case EN_PWM_ERROR:
  	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  	  break;
  	case TIM_CONFIG_ERROR:
  	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  	  break;
  	case DMA_ERROR:
  	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  	  break;
  	case GPIO_ERROR:
  	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  	  break;
  	case UART_ERROR:
  		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  		break;
  	case OPEN_ERROR:
  		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  		break;
  	default:
  	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  	}
  }
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
