/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(uint8_t ERROR);

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define UART_ERROR 7
//#define OPEN_ERROR 8
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define USART6_TX_Pin GPIO_PIN_6
#define USART6_TX_GPIO_Port GPIOC
#define USART6_RX_Pin GPIO_PIN_7
#define USART6_RX_GPIO_Port GPIOC

#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

// Button 1 PA1, Lock PA9
#define BUTTON1_Pin GPIO_PIN_1
#define BUTTON1_GPIO_Port GPIOA
#define LOCK1_Pin GPIO_PIN_9
#define LOCK1_GPIO_Port	GPIOA

// Button 2 PA10, Lock PA8
#define BUTTON2_Pin GPIO_PIN_10
#define BUTTON2_GPIO_Port GPIOA
#define LOCK2_Pin GPIO_PIN_8
#define LOCK2_GPIO_Port GPIOA

// Button 3 PA11, Lock PC9
#define BUTTON3_Pin GPIO_PIN_11
#define BUTTON3_GPIO_Port GPIOA
#define LOCK3_Pin GPIO_PIN_9
#define LOCK3_GPIO_Port GPIOC

// Button4 PA12, Lock PC8
#define BUTTON4_Pin GPIO_PIN_12
#define BUTTON4_GPIO_Port GPIOA
#define LOCK4_Pin GPIO_PIN_8
#define LOCK4_GPIO_Port GPIOC

//microSD
#define MICRO_D0_Pin GPIO_PIN_2
#define MICRO_DI_Pin GPIO_PIN_3
#define MICRO_DATA_GPIO_Port GPIOC
#define MICRO_CLK_Pin GPIO_PIN_10
#define MICRO_CLK_GPIO_Port GPIOB
enum {
	FAIL,
	PASS
};
//#define SD_CS_Pin GPIO_PIN_12
//#define SD_CS_GPIO_Port GPIOB

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
