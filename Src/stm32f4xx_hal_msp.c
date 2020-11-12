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

/*
 * This file defines all initialization functions to configure the IP instances
 * according to the user configuration (pin allocation, enabling of block, use
 * of DMA and interrupts
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

//Msp = MCU Support Package

void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);

  /* System interrupt init*/

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}


/**
* @brief TIM_Base MSP Initialization
* This function configures the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
    /* TIM3 interrupt Init */
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }

}

/**
* @brief TIM_Base MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();

    /* TIM3 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM3_IRQn);
  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }

}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
  //GPIO_InitTypeDef   GPIO_InitStruct;
  //static DMA_HandleTypeDef  hdma_tim2_ch1;

  // ----------------------------------
  // Enable peripherals and GPIO Clocks
  // ----------------------------------

  /*
  __HAL_RCC_TIM2_CLK_ENABLE();	// Enable TIM2 clock
	__HAL_RCC_GPIOA_CLK_ENABLE();	// Enable TIM2 GPIO clock
	__HAL_RCC_DMA1_CLK_ENABLE();	// Enable DMA1 clock
	*/

	/*
  // Configure TIM2_Channel 1 (PA0) as output, push-pull and alternate function mode
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	*/

	/*
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
  */
}


/**
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(huart->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
  else if(huart->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspInit 0 */

  /* USER CODE END USART6_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART6 GPIO Configuration
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX
    */
    GPIO_InitStruct.Pin = USART6_TX_Pin|USART6_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN USART6_MspInit 1 */

  /* USER CODE END USART6_MspInit 1 */
  }
}


/**
* @brief SPI MSP Initialization
* This function configures the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hspi->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspInit 0 */

  /* USER CODE END SPI2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI2 GPIO Configuration
    PC2     ------> SPI2_MISO  --> DO
    PC3     ------> SPI2_MOSI  --> DI
    PB10     ------> SPI2_SCK
    */
    //GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Pin = MICRO_D0_Pin|MICRO_DI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(MICRO_DATA_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = MICRO_CLK_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(MICRO_CLK_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_MspInit 1 */

  /* USER CODE END SPI2_MspInit 1 */
  }
}

/**
* @brief SPI MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
  if(hspi->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspDeInit 0 */

  /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();

    /**SPI2 GPIO Configuration
    PC2     ------> SPI2_MISO
    PC3     ------> SPI2_MOSI
    PB10     ------> SPI2_SCK
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_2|GPIO_PIN_3);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);

  /* USER CODE BEGIN SPI2_MspDeInit 1 */

  /* USER CODE END SPI2_MspDeInit 1 */
  }

}

/**
* @brief UART MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
/*
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==USART2)
  {

    // Peripheral clock disable
    __HAL_RCC_USART2_CLK_DISABLE();

    //USART2 GPIO Configuration
    //PA2     ------> USART2_TX
    //PA3     ------> USART2_RX

    HAL_GPIO_DeInit(GPIOA, USART_TX_Pin|USART_RX_Pin);

  }
  else if(huart->Instance==USART6)
  {
    // Peripheral clock disable
    __HAL_RCC_USART6_CLK_DISABLE();

    //USART6 GPIO Configuration
    //PC6     ------> USART6_TX
    //PC7     ------> USART6_RX

    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6|GPIO_PIN_7);
  }

}
*/
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
