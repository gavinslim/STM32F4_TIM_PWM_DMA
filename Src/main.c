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
#include <tm1637_.h>
#include "microSD.h"

/* Private typedef -----------------------------------------------------------*/
//GPIO_InitTypeDef GPIO_InitStruct;

/* Private define ------------------------------------------------------------*/
#define ON 1
#define OFF 0
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
//char buffer[100];
uint8_t UART6_rxBuffer[10];

/* Timer Definitions */
//int curr_time = COUNTDOWN_TIME;
int min_time = COUNTDOWN_MIN;
int sec_time = COUNTDOWN_SEC;
int time_flag = 0;
int end_dur = 5;

/* Locker countdown */
int start_lockdown = 0;
int lock_counter = 0;
int lock_progress = 0;

/* LED colour */
typedef enum color{
	GREEN,
	ORANGE,
	PURPLE,
	BLUE,
	RED
}color;

color LED_color;

/* Timer handler declaration */

/* Timer Output Compare Configuration Structure declaration */
TIM_OC_InitTypeDef sConfig;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(void);

void transmit_uart(char *string){
	uint8_t len = strlen(string);
	HAL_UART_Transmit(&huart2, (uint8_t*) string, len, 200);
}

void lock_5sec(GPIO_TypeDef* lockPort, uint16_t lockPin){
	if (lock_progress == 0){
		transmit_uart("Locking for 5seconds\r\n");
		start_lockdown = 1;
	}

	if (start_lockdown == 1){
		HAL_GPIO_WritePin(lockPort, lockPin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(lockPort, lockPin, GPIO_PIN_RESET);
	}

}

void solenoid_control(GPIO_TypeDef* lockPort, uint16_t lockPin, uint8_t status){
	if (status == ON){
		HAL_GPIO_WritePin(lockPort, lockPin, GPIO_PIN_SET);
	  start_lockdown = 1;
	} else {
		HAL_GPIO_WritePin(lockPort, lockPin, GPIO_PIN_RESET);
	}
}

/* Callback called by HAL_UART_IRQHandler when given number of bytes is received */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == USART6){
		/* Receive data in interrupt mode */
		HAL_UART_Receive_IT(&huart6, UART6_rxBuffer, 1);
		//transmit_uart((char*) UART6_rxBuffer);

		switch(UART6_rxBuffer[0]){
		case (uint8_t)'R':
			LED_color = RED;
			transmit_uart("Received 'Red'\r\n");
		  //lock_5sec(LOCK1_GPIO_Port, LOCK1_Pin);
			break;
		case (uint8_t)'G':
			LED_color = GREEN;
			transmit_uart("Received 'Green'\r\n");
			break;
		case (uint8_t)'B':
			LED_color = BLUE;
			transmit_uart("Received 'Blue'\r\n");
			break;
		case (uint8_t)'P':
			LED_color = PURPLE;
			transmit_uart("Received 'Purple'\r\n");
			break;
		case (uint8_t)'O':
			LED_color = ORANGE;
			transmit_uart("Received 'Orange'\r\n");
		  break;
		default:
			break;
		}
	}
}


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
  MX_USART6_UART_Init();

  /* Initialize countdown timer */
  MX_TIM3_Init();
  HAL_TIM_Base_Start_IT(&htim3);	//Interrupt triggers every second
  time_flag = 0;

  // microSD Setup
  microSD_init();
  MX_FATFS_Init();

  // WS2812B Setup
  LED_Init();
  LED_set_color_all(0x00, 0x00, 0x00);	//Set color order of array. Ex: R0,G0,B0,R1,G1,B1
  LED_update(1);

  // Bluetooth Setup
	HAL_UART_Receive_IT(&huart6, UART6_rxBuffer, 1);

  // ------------------------------ //
  //       microSD Card Init        //
  // -------------------------------//

  /* Wait for microSD to initialize */
  HAL_Delay(500);

  /* Check if microSD is connected physically */
	transmit_uart("-----------------------\r\n");
	while (check_microSD_conn() == FAIL){
  	transmit_uart("MicroSD card not detected!\r\n");
  	pulse_red();
  	//HAL_Delay(1000);
	}
	transmit_uart("MicroSD card detected!\r\n");
	transmit_uart("-----------------------\r\n");

	//char file_name[50] = "Crystal.txt";
	//open_file(file_name);
	//write_file();
	//close_file();

	//open_file(file_name);
	//read_file();
	//close_file();
	chk_microSD();

	// Timer Count Down //
	// ---------------- //
  tm1637Init();
  tm1637SetBrightness(3);	 // Optionally set brightness. 0 is off. By default, initialized to full brightness.
  tm1637DisplayTime(99, 99, 1);   // Display the value "1234" and turn on the `:` that is between digits 2 and 3.

	// ------------- //
	// Infinite Loop //
	// ------------- //
  while (1) {
  	if (check_microSD_conn() == PASS){
  		/*
  		switch(UART6_rxBuffer[0]){
  		case (uint8_t)'R':
  			LED_color = RED;
  			transmit_uart("Red\r\n");
  		  //lock_5sec(LOCK1_GPIO_Port, LOCK1_Pin);
  			break;
  		case (uint8_t)'G':
  			LED_color = GREEN;
  			transmit_uart("Green\r\n");
  			break;
  		case (uint8_t)'B':
  			LED_color = BLUE;
  			transmit_uart("Blue\r\n");
  			break;
  		case (uint8_t)'P':
  			LED_color = PURPLE;
  			transmit_uart("Purple\r\n");
  			break;
  		case (uint8_t)'O':
  			LED_color = ORANGE;
  			transmit_uart("Orange\r\n");
  		  break;
  		default:
  			break;
  		}
			*/

  		// Check status of LED strip
  		switch(LED_color){
  		case GREEN:
  			pulse();
  			break;
  		case ORANGE:
  			pulse_orange();
  			break;
  		case BLUE:
  			pulse_blue();
  			break;
  		case RED:
  			pulse_red();
  			break;
  		case PURPLE:
  			pulse_purple();
  			break;
  		default:
  			break;
  		}

  		// Check status of solenoid


  		//_tm1637Start();
  		// Solenoid 1
  		/*
  		if (HAL_GPIO_ReadPin(BUTTON1_GPIO_Port, BUTTON1_Pin)) {
    		transmit_uart("Button 1 Pressed!\r\n");
    		HAL_GPIO_WritePin(LOCK1_GPIO_Port, LOCK1_Pin, GPIO_PIN_SET);
    		LED_color = GREEN;
  			time_flag = !time_flag;	//Toggle flag to countdown timer
    	} else {
    		HAL_GPIO_WritePin(LOCK1_GPIO_Port, LOCK1_Pin, GPIO_PIN_RESET);
    	}

    	// Solenoid 2
    	if (HAL_GPIO_ReadPin(BUTTON2_GPIO_Port, BUTTON2_Pin)){
    		transmit_uart("Button 2 Pressed!\r\n");
    		HAL_GPIO_WritePin(LOCK2_GPIO_Port, LOCK2_Pin, GPIO_PIN_SET);
    		LED_color = ORANGE;
    	} else {
    		HAL_GPIO_WritePin(LOCK2_GPIO_Port, LOCK2_Pin, GPIO_PIN_RESET);
    	}

  		if (HAL_GPIO_ReadPin(BUTTON2_GPIO_Port, BUTTON2_Pin)) {
    		transmit_uart("Button 2 Pressed!\r\n");
  			solenoid_control(LOCK2_GPIO_Port, LOCK1_Pin, ON);
  		} else {
    		solenoid_control(LOCK2_GPIO_Port, LOCK1_Pin, OFF);
  		}

    	// Solenoid 3
    	if (HAL_GPIO_ReadPin(BUTTON3_GPIO_Port, BUTTON3_Pin)){
    		transmit_uart("Button 3 Pressed!\r\n");
    		HAL_GPIO_WritePin(LOCK3_GPIO_Port, LOCK3_Pin, GPIO_PIN_SET);
    		LED_color = BLUE;
    	} else {
    		HAL_GPIO_WritePin(LOCK3_GPIO_Port, LOCK3_Pin, GPIO_PIN_RESET);
    	}

    	// Solenoid 4
    	if (HAL_GPIO_ReadPin(BUTTON4_GPIO_Port, BUTTON4_Pin)){
    		transmit_uart("Button 4 Pressed!\r\n");
    		HAL_GPIO_WritePin(LOCK4_GPIO_Port, LOCK4_Pin, GPIO_PIN_SET);
    		LED_color = RED;
    	} else {
    		HAL_GPIO_WritePin(LOCK4_GPIO_Port, LOCK4_Pin, GPIO_PIN_RESET);
    	}

    	// Solenoid 5
    	if (HAL_GPIO_ReadPin(BUTTON5_GPIO_Port, BUTTON5_Pin)){
    		transmit_uart("Button 5 Pressed!\r\n");
    		HAL_GPIO_WritePin(LOCK5_GPIO_Port, LOCK5_Pin, GPIO_PIN_SET);
    		LED_color = PURPLE;
    	} else {
    		HAL_GPIO_WritePin(LOCK5_GPIO_Port, LOCK5_Pin, GPIO_PIN_RESET);
    	}
			*/

    	// Clear buffer
  		//memset(UART6_rxBuffer, '\0', sizeof(UART6_rxBuffer));

  	} else {
  		while (check_microSD_conn() == FAIL){
  			transmit_uart("MicroSD card not detected!\r\n");
  			pulse_red();
  		}
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

	/*Configure GPIO pin :
	 * Button 1 PA1
	 * Button 2 PA10
	 * Button 3 PA11
	 * Button 4 PA12
	 * Lock 1	PA9
	 * Lock 2	PA8
	 * Lock 3
	 * Lock 4 */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PA9 */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* Button4 PB14, Lock PB15 */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler(UART_ERROR);
  }
  /* USER CODE BEGIN USART6_Init 2 */
  HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART6_IRQn);
  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 41999;	//84MHz/(42000 * 2000); max value of PSC and ARR is 2^16
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler(UART_ERROR);
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler(UART_ERROR);
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler(UART_ERROR);
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/* TIM3 interrupt called every second */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim3){

	// Start lock countdown if enabled
	if (start_lockdown == 1){
		lock_counter = 5;

		start_lockdown = 0;
		lock_progress = 1;
	}

	// Countdown lock counter
	if (lock_progress == 1){
		lock_counter--;
		transmit_uart("Counting down...\r\n");
		if (lock_counter == 0){
			lock_progress = 0;
			transmit_uart("Done locking.\r\n");
		}
	}

	//transmit_uart("time\r\n");

	if (time_flag == 1){
		if (min_time == 0 && sec_time == 0){

			//Blink every other second
			if (end_dur % 2 == 1){
				tm1637SetBrightness(0);
			} else {
				tm1637SetBrightness(3);
			}

			// After 5 seconds, reset timer
			if (end_dur == 0){
				end_dur = sec_time - (sec_time - 5);
				min_time = COUNTDOWN_MIN;
				sec_time = COUNTDOWN_SEC;
			}
			end_dur--;
		} else {
			if (sec_time == 0){
				min_time--;
				sec_time = SIXTY_SECONDS - 1;
			} else {
				sec_time--;
			}
		}
		tm1637DisplayTime(min_time, sec_time, 1);
	}
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
  	switch(ERROR) {
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
