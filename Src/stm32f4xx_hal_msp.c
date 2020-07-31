/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : stm32f4xx_hal_msp.c
  * Description        : This file provides code for the MSP Initialization
  *                      and de-Initialization codes.
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

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_tim2_ch1;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief TIM MSP Initialization
  *        This function configures the hardware resources used:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  *           - DMA configuration for transmission request by peripheral
  * @param htim: TIM handle pointer
  * @retval None
  */

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef   GPIO_InitStruct;
  static DMA_HandleTypeDef  hdma_tim2_ch1;

  // ----------------------------------
  // Enable peripherals and GPIO Clocks
  // ----------------------------------

  __HAL_RCC_TIM2_CLK_ENABLE();	// Enable TIM2 clock
	__HAL_RCC_GPIOA_CLK_ENABLE();	// Enable TIM2 GPIO clock
	__HAL_RCC_DMA1_CLK_ENABLE();	// Enable DMA1 clock

  // Configure TIM2_Channel 1 (PA0) as output, push-pull and alternate function mode
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Configure DMA1 parameters
  // Based on RM0383 STM32F411 Ref Manual (Table 27), TIM2_CH1 corresponds to DMA1 Channel 3 Stream 5
  hdma_tim2_ch1.Instance = DMA1_Stream5;
  hdma_tim2_ch1.Init.Channel = DMA_CHANNEL_3;
  hdma_tim2_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;							// Memory to Peripheral mode
  hdma_tim2_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_tim2_ch1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_tim2_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD ;
  hdma_tim2_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD ;
  hdma_tim2_ch1.Init.Mode = DMA_CIRCULAR;														// Set in circular mode
  hdma_tim2_ch1.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_tim2_ch1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  hdma_tim2_ch1.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_tim2_ch1.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_tim2_ch1.Init.PeriphBurst = DMA_PBURST_SINGLE;

  // Enable Half-Transfer and Full-Transfer complete interrupts
  __HAL_DMA_ENABLE_IT(&hdma_tim2_ch1, (DMA_IT_TC | DMA_IT_HT));

  // Link hdma_tim2_ch1 to hdma[TIM_DMA_ID_CC3] (channel3)
  __HAL_LINKDMA(htim, hdma[TIM_DMA_ID_CC1], hdma_tim2_ch1);

  // Initialize TIM2 DMA handle
  //HAL_DMA_Init(htim->hdma[TIM_DMA_ID_CC1]);
  if (HAL_DMA_Init(&hdma_tim2_ch1) != HAL_OK)
  {
    Error_Handler();
  }

  // ##-2- Configure the NVIC for DMA #########################################
  // NVIC configuration for DMA transfer complete interrupt
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
}


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
