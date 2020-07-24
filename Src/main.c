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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
#include <string.h>


/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/
#define LED_STRIP_CNT		 		10		// Number of LEDs in strip
#define LED_NUM_COLOR				3			// Number of colors in each LED (RGB = 3)
#define LED_BITS_PER_COLOR 	8			// Each color is represented by 8 bits
/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim2;

static uint8_t temp_led_data[LED_STRIP_CNT * LED_NUM_COLOR * LED_BITS_PER_COLOR];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);

void ChangeDuty(uint8_t period);

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	volatile uint32_t timeout;
	int flag;
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();

  // Configure DMA with TIMER Update request to transfer data from memory to TIMER Capture Compare Register 1 (CCR1)
  // TIM2 Channel 2 will generate complementary PWM signal with frequency = 800kHz. A variable duty cycle
  // that is modified by DMA will be used to adjust the PWM signal.

  flag = 1;

  while (1) {
  	// Wait
  	timeout = 0xFFFFFF;
  	while (--timeout) {}
   	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

   	// Update LED sequence colors
  	// Set LED colors
   	if (flag) {
   		ChangeDuty(67);
   		flag = 0;
   	} else {
   		ChangeDuty(33);
   		flag = 1;
   	}

   	// Send sequence to DMA

  	// Send reset pulse

   	// DMA sends variable duty cycle to TIM output channel

  	// Send reset pulse

  }

}

// D = 0.64 = 1b -> Period = 67
//   = 0.32 = 0b -> Period = 33
// Reset pulse - 50us pulse low
void ResetPulse (void) {
	// 50us = 1.25us * 40 pulses at Duty Cycle = 0

}

// Change Duty Cycle
void ChangeDuty (uint8_t period) {
	TIM_OC_InitTypeDef sConfigOC = {0};

  // PWM Channel Configuration
  sConfigOC.OCMode = TIM_OCMODE_PWM1;					// Set to PWM Mode 1. PWM Output = HIGH if timer counter < Timer CCR, else LOW
  sConfigOC.Pulse = period;												// Capture Compare Register (CCR) value. 50% DUTY = 52
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;

  // Initialize PWM configuration for TIM2 Channel 1
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  // Start TIM2 PWM Channel 1
  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)	 // Must call this function after every HAL_TIM_PWM_ConfigChannel()
  {
    Error_Handler();
  }
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
    Error_Handler();
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
    Error_Handler();
  }
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/*
  Reference: http://stm32f4-discovery.net/2014/05/stm32f4-stm32f429-discovery-pwm-tutorial/
	TIM2 is connected to APB1 bus and on the F411 device, it is connected to 42MHz clock.
	Timer also has internal phase-locked loops (PLL) that doubles the frequency of the clock up to 84MHz.
	Note: Some TIM are connected to APB2, which is connected to 84MHz by default. Therefore, setting PLL
	will double the clock frequency to 168MHz. Take care to note which ABPX your timer is connected to.

	Hardware configuration for Timer:
	---------------------------------
	HCLK = SYSCLK / AHB Prescaler
	     = 84MHz / 1 = 84MHz

	APB1CLK = HCLK / APB1 Prescaler 	Note: APB1CLK = APB1 peripheral clock
	        = 84MHz / 2 = 42MHz

	TIM2CLK = APB1CLK*2
					= 42MHz * 2 = 84MHz

	Calculation for Timer Config:
	-----------------------------
	TIM2_counter_clk = TIM2CLK / (Prescaler + 1)

	To achieve max frequency for timer, set prescaler to 0 and PLL enabled:
	TIM2_counter_clk = 84MHz / (0 + 1) = 84MHz

	PWM_freq = TIM2_counter_clk / (TIM_Period + 1), or
	TIM_Period = TIM2_counter_clk / PWM_freq - 1

	TIM_Period is 16bit so max is 65535

	To achieve 800kHz for PWM, TIM_Period = 84000000 / 800000 - 1 = 104 = 0x68

	Note: Max timer frequency on APB1 is 50MHz and on APB2 is up to 100MHz, by setting the TIMPRE bit in RCC_DCKCFGR register.
				If APBx prescaler is 1 or 2 or 4, then TIMxCLK = HCKL, otherwise TIMxCLK >= 4x PCLKx

	=====
	PWM has 2 modes: MODE1 and MODE2
		- MODE1 = Timer output is HIGH if counter < CCR, else LOW
		- MODE2 = Timer output is LOW if counter < CCR, else HIGH

	Duty Cycle = CCR / (TIM_Period + 1)
	TIM2 Channel1 duty cycle = (TIM2_CCR1/ TIM_Period + 1)* 100 = 50%
 */

static void MX_TIM2_Init(void){
	//TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	//TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	memset(&sConfigOC, 0, sizeof(sConfigOC));

	// Timer Configuration
	htim2.Instance = TIM2;												// Select TIM2
	htim2.Init.Prescaler = 0; 										// Set prescaler = 0 to maximize timer frequency
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;	// Count up
	htim2.Init.Period = 104;											// Period = 104 to achieve 800kHz PWM. Period = Auto Reload Register
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  // Initialize PWM Generation for TIM2
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  // PWM Channel Configuration
  sConfigOC.OCMode = TIM_OCMODE_PWM1;					// Set to PWM Mode 1. PWM Output = HIGH if timer counter < Timer CCR, else LOW
  sConfigOC.Pulse = 52;												// Capture Compare Register (CCR) value. 50% DUTY = 52
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;

  // Initialize PWM configuration for TIM2 Channel 1
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  // Start TIM2 PWM Channel 1
  /*
  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)	 // Must call this function after every HAL_TIM_PWM_ConfigChannel()
  {
    Error_Handler();
  }
  */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1_Pin */
  GPIO_InitStruct.Pin = PA1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PA0_GPIO_Port, &GPIO_InitStruct);

  /*
  GPIO_InitStruct.Pin = PA0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;		//Select Alternate Function 1 (AF1) for PA0 for TIM2_CH2 function
  HAL_GPIO_Init(PA0_GPIO_Port, &GPIO_InitStruct);
	*/

}

/* USER CODE BEGIN 4 */

// GPIO Initialization for PWM
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(htim_pwm->Instance==TIM2)
  {
    /* Peripheral clock enable */
    __TIM2_CLK_ENABLE();

    /**TIM2 GPIO Configuration
    PA0     ------> TIM2_CH1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
