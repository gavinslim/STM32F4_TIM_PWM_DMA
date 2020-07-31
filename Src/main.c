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

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Timer handler declaration */
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch1;

/* Timer Output Compare Configuration Structure declaration */
TIM_OC_InitTypeDef sConfig;

/* Capture Compare buffer */
uint32_t tmp_led_data[TMP_LED_SIZE];

/* Array of R, G, B colours */
static uint8_t leds_colors[LED_CFG_BYTES_PER_LED * LED_CFG_STRIP_CNT];

/* Timer Period*/
uint32_t uwTimerPeriod  = 0;


/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);

static void Stop_PWM(void);
//static void Start_PWM(void);

//static uint8_t LED_reset(void);

void blue(void){
  uint32_t index;

  uint8_t g_segment = LED_CFG_BITS_PER_LED - 16;
  uint8_t b_segment = LED_CFG_BITS_PER_LED - 8;
  uint8_t r_segment = LED_CFG_BITS_PER_LED;

  for (index = 0; index < LED_CFG_BITS_PER_LED; index++) {
  	if (index < g_segment){
  		if (index == 0){
    		tmp_led_data[index] = (uint32_t)(((uint32_t) 33 * (uwTimerPeriod - 1)) / 100);
  		} else {
  			tmp_led_data[index] = (uint32_t)(((uint32_t) 33 * (uwTimerPeriod - 1)) / 100);
  		}
  	} else if (index >= g_segment && index < b_segment) {
  		tmp_led_data[index] = (uint32_t)(((uint32_t) 33 * (uwTimerPeriod - 1)) / 100);
  	} else if (index >= b_segment && index < r_segment) {
  		tmp_led_data[index] = (uint32_t)(((uint32_t) 67 * (uwTimerPeriod - 1)) / 100);
  	}
  }
}

void green(void){
  uint32_t index;

  uint8_t g_segment = LED_CFG_BITS_PER_LED - 16;
  uint8_t b_segment = LED_CFG_BITS_PER_LED - 8;
  uint8_t r_segment = LED_CFG_BITS_PER_LED;

  for (index = 0; index < LED_CFG_BITS_PER_LED; index++) {
  	if (index < g_segment){
  		if (index < 3){
    		tmp_led_data[index] = (uint32_t)(((uint32_t) 33 * (uwTimerPeriod - 1)) / 100);
  		} else {
  			tmp_led_data[index] = (uint32_t)(((uint32_t) 67 * (uwTimerPeriod - 1)) / 100);
  		}
  	} else if (index >= g_segment && index < b_segment) {
  		tmp_led_data[index] = (uint32_t)(((uint32_t) 33 * (uwTimerPeriod - 1)) / 100);
  	} else if (index >= b_segment && index < r_segment) {
  		tmp_led_data[index] = (uint32_t)(((uint32_t) 33 * (uwTimerPeriod - 1)) / 100);
  	}
  }
}

void orange(void){
  uint32_t index;

  uint8_t g_segment = LED_CFG_BITS_PER_LED - 16;
  uint8_t b_segment = LED_CFG_BITS_PER_LED - 8;
  uint8_t r_segment = LED_CFG_BITS_PER_LED;

  for (index = 0; index < LED_CFG_BITS_PER_LED; index++) {
  	if (index < g_segment){
  		tmp_led_data[index] = (uint32_t)(((uint32_t) 33 * (uwTimerPeriod - 1)) / 100);
  	} else if (index >= g_segment && index < b_segment) {
  		tmp_led_data[index] = (uint32_t)(((uint32_t) 67 * (uwTimerPeriod - 1)) / 100);
  	} else if (index >= b_segment && index < r_segment) {
  		tmp_led_data[index] = (uint32_t)(((uint32_t) 67 * (uwTimerPeriod - 1)) / 100);
  	}
  }
}

void resetPulse(void){
  uint32_t index;
  for (index = 0; index < (LED_CFG_BITS_PER_LED); index++) {
  	tmp_led_data[index] = (uint32_t)(((uint32_t) 0 * (uwTimerPeriod - 1)) / 100);
  }
}

void reset_lightup(void){
  uint32_t index;
  uint8_t rst_num_pulse = 40;
  uint8_t green_end = 58;
  uint8_t blue_end = 66;
  uint8_t red_end = 74;

  for (index = 0; index < rst_num_pulse; index++) {
  	tmp_led_data[index] = (uint32_t)(((uint32_t) 10 * (uwTimerPeriod - 1)) / 100);
  }

  // Write Green LED
	tmp_led_data[rst_num_pulse] = (uint32_t)(((uint32_t) 33 * (uwTimerPeriod - 1)) / 100);
  for (index = rst_num_pulse + 1; index < green_end; index++){
  	tmp_led_data[index] = (uint32_t)(((uint32_t) 67 * (uwTimerPeriod - 1)) / 100);
  }

  // Write Blue LED
  for (index = green_end; index < blue_end; index++){
  	tmp_led_data[index] = (uint32_t)(((uint32_t) 33 * (uwTimerPeriod - 1)) / 100);
  }

  // Write Red LED
  for (index = blue_end; index < red_end; index++){
  	tmp_led_data[index] = (uint32_t)(((uint32_t) 33 * (uwTimerPeriod - 1)) / 100);
  }
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{

  HAL_Init();

  /* Configure the system clock to 180 MHz */
  SystemClock_Config();

  // Initialize and start GPIO for TIM2
  MX_GPIO_Init();

  // Compute the value of ARR regiter to generate signal frequency at 800kHz
  uwTimerPeriod = (uint32_t)((SystemCoreClock / 800000) - 1);



  //reset_lightup();

  /*
  uint32_t index;
  for (index = 0; index < (LED_CFG_BITS_PER_LED); index++) {
  	tmp_led_data[index] = (uint32_t)(((uint32_t) 64 * (uwTimerPeriod - 1)) / 100);
  }
	*/
  // Initialize and start TIM2 DMA generation
  MX_TIM2_Init();
  orange();
  //LED_reset();
  //Start_PWM();

  //HAL_DMAEx_MultiBufferStart
  //__HAL_DMA_ENABLE(&htim2);
  //HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);
  //HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
  while (1)
  {
  }

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* Turn LED2 on */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  while (1) {
  }
}



static uint8_t LED_reset(void){
	memset(tmp_led_data, 0, sizeof(tmp_led_data));

  uint32_t index;
  /*
  for (index = 0; index < (LED_STRIP_TOTAL_BITS - 1); index++) {
  	tmp_led_data[index] = (uint32_t)(((uint32_t) 64 * (uwTimerPeriod - 1)) / 100);
  }
  tmp_led_data[LED_STRIP_TOTAL_BITS - 1] = (uint32_t)(((uint32_t) 0 * (uwTimerPeriod - 1)) / 100);
	*/
  /*
  for (index = 0; index < (LED_STRIP_TOTAL_BITS); index++) {
  	tmp_led_data[index] = (uint32_t)(((uint32_t) 64 * (uwTimerPeriod - 1)) / 100);
  }
  */
	return 1;
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

static void MX_TIM2_Init(void)
{
  htim2.Instance               = TIM2;
  htim2.Init.Period            = uwTimerPeriod;
  htim2.Init.RepetitionCounter = 3;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  htim2.Init.Prescaler         = 0;
  htim2.Init.ClockDivision     = 0;
  htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)			// Calls HAL_TIM_PWM_MspInit()
  {
    // Initialization Error
    Error_Handler();
  }

  // Configure PWM Channel
  sConfig.OCMode       = TIM_OCMODE_PWM1;
  sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfig.Pulse        = tmp_led_data[0];
  sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    // Configuration Error
    Error_Handler();
  }

  // Initiate PWM generation
  if (HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, tmp_led_data, TMP_LED_SIZE) != HAL_OK)
  {
    // Starting Error
    Error_Handler();
  }

  //HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
  //HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);
  //HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, tmp_led_data, LED_STRIP_TOTAL_BITS);
  /*
  if (HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1) != HAL_OK){
    Error_Handler();
  }
  */

}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  //GPIO Ports Clock Enable
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  //Configure GPIO pin Output Level
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  //Configure GPIO pin : B1_Pin
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  //Configure GPIO pin : LD2_Pin
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/*
static void Start_PWM(void)
{

  // Initiate PWM generation

  if (HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, tmp_led_data, LED_STRIP_TOTAL_BITS) != HAL_OK)
  {
    // Starting Error
    Error_Handler();
  }


	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

}
*/


static void Stop_PWM(void){
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
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
