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
GPIO_InitTypeDef GPIO_InitStruct = {0};

TIM_HandleTypeDef htim2 = {0};
TIM_OC_InitTypeDef sConfigOC = {0};

DMA_HandleTypeDef hdma_tim2_ch1 = {0};

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Timer handler declaration */


/* Timer Output Compare Configuration Structure declaration */
TIM_OC_InitTypeDef sConfig;

/* Capture Compare buffer */
uint32_t tmp_led_data[TMP_LED_SIZE];

/* Array of R, G, B colours */
//static uint8_t leds_colors[LED_CFG_BYTES_PER_LED * LED_CFG_STRIP_CNT];

/* Timer Period*/
uint32_t uwTimerPeriod  = 0;


/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void LED_Init(void);

static uint8_t LED_set_color_all(uint8_t red, uint8_t green, uint8_t blue){
	size_t index;
	for (index = 0; index < LED_CFG_STRIP_CNT; index++){
		LED_colors[index * LED_CFG_BYTES_PER_LED + 0] = red;
		LED_colors[index * LED_CFG_BYTES_PER_LED + 1] = green;
		LED_colors[index * LED_CFG_BYTES_PER_LED + 2] = blue;
	}
	return 1;
}

static uint8_t led_reset_pulse(void){

	// Stop PWM generation from DMA1
	//HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);
	//HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);


	// Modify DMA config to Normal Mode
  hdma_tim2_ch1.Init.Mode = DMA_NORMAL;

  // Set all data for tmp_led_data to 0
  memset(tmp_led_data, 0, sizeof(tmp_led_data));

  uint32_t index;
  for (index = 0; index < LED_CFG_BITS_PER_LED; index++) {
  	tmp_led_data[index] = (uint32_t)(((uint32_t) 50 * (uwTimerPeriod - 1)) / 100);
  }

  // Initialize TIM2 DMA handle
  if (HAL_DMA_Init(&hdma_tim2_ch1) != HAL_OK) {
    Error_Handler(DMA_ERROR);
  }

	//__HAL_DMA_ENABLE(&hdma_tim2_ch1);
  // Start PWM generation from DMA1
  // Set memory address = tmp_led_data, with data length = TMP_LED_SIZE
  if (HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, tmp_led_data, TMP_LED_SIZE) != HAL_OK){
    Error_Handler(EN_PWM_ERROR);
  }

  return 1;
}

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

  SystemClock_Config();		// Configure system clock to 180MHz
  LED_Init();							// Initialize and start GPIO for TIM2

  // Compute the value of ARR regiter to generate signal frequency at 800kHz
  uwTimerPeriod = (uint32_t)((SystemCoreClock / 800000) - 1);

  //orange();
  uint32_t index;
  for (index = 0; index < LED_CFG_BITS_PER_LED; index++) {
  	tmp_led_data[index] = (uint32_t)(((uint32_t) 80 * (uwTimerPeriod - 1)) / 100);
  }

  led_reset_pulse();

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
  	default:
  	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
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

static void LED_Init(void)
{


  /* ------------- */
  /*  GPIO Config  */
  /* ------------- */

  // GPIO Ports Clock Enable
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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

  // Configure TIM2_Channel 1 (PA0) as output, push-pull and alternate function mode
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* -------------- */
  /* TIM2 Channel 1 */
  /* -------------- */

  // Enable TIM2 clock
  __HAL_RCC_TIM2_CLK_ENABLE();

  // TIM Time Base handle Structure definition
  htim2.Instance               = TIM2;
  htim2.Init.Prescaler         = 0;
  htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim2.Init.Period            = 104;
  htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  // Note: HAL_TIM_PWM_Init() calls HAL_TIM_PWM_MspInit()
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
    Error_Handler(TIM_INIT_ERROR);     // Initialization Error
  }

  // Set TIM Output Compare (OC) Configuration Structure definition
  sConfigOC.OCMode       = TIM_OCMODE_PWM1;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.Pulse        = 0;
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  // Apply TIM OC configs to htim2 (TIM2)
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler(TIM_CONFIG_ERROR); // Configuration Error
  }

  /* ------------------------------ */
  /* DMA1 Channel 3 Stream 5 Config */
  /* ------------------------------ */

  // Enable DMA1 clock
	__HAL_RCC_DMA1_CLK_ENABLE();

	// DMA handle Structure definition (Based on RM0383 STM32F411 Ref Manual (Table 27) TIM2_CH1 corresponds to DMA1 Channel 3 Stream 5)
  hdma_tim2_ch1.Instance = DMA1_Stream5;
  hdma_tim2_ch1.Init.Channel = DMA_CHANNEL_3;
  hdma_tim2_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;							// Memory to Peripheral mode
  hdma_tim2_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_tim2_ch1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_tim2_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD ;
  hdma_tim2_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD ;
  hdma_tim2_ch1.Init.Mode = DMA_CIRCULAR;														// Set in circular mode
  hdma_tim2_ch1.Init.Priority = DMA_PRIORITY_LOW;										// Low priority
  hdma_tim2_ch1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  hdma_tim2_ch1.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_tim2_ch1.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_tim2_ch1.Init.PeriphBurst = DMA_PBURST_SINGLE;

  // Enable Half-Transfer and Full-Transfer complete interrupts
  __HAL_DMA_ENABLE_IT(&hdma_tim2_ch1, (DMA_IT_TC | DMA_IT_HT));

  // Linking a PPP peripheral to DMA structure pointer (PPP = STM32 peripheral or block)
  // hdma[TIM_DMA_ID_CC1] = Capture/Compare 1 DMA requests peripheral
  // Link htim2 (TIM peripheral) to hdma_tim2_ch1 (DMA struc pointer) with TIM_DMA_ID_CC1
  __HAL_LINKDMA(&htim2, hdma[TIM_DMA_ID_CC1], hdma_tim2_ch1);

  // Initialize TIM2 DMA handle
  if (HAL_DMA_Init(&hdma_tim2_ch1) != HAL_OK) {
    Error_Handler(DMA_ERROR);
  }

  // Initiate PWM generation
  /*
  if (HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, tmp_led_data, TMP_LED_SIZE) != HAL_OK) {
    // Starting Error
    Error_Handler(EN_PWM_ERROR);
  }
	*/
  // ##-2- Configure the NVIC for DMA #########################################
  // NVIC configuration for DMA transfer complete interrupt
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
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
