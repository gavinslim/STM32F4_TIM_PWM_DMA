/*
 * w2812b.h
 *
 *  Created on: Jul 28, 2020
 *      Author: Gavin
 */

#ifndef W2812B_H_
#define W2812B_H_

#include <stdint.h>
#include "string.h"
#include "stm32f4xx.h"

//LED Strip variables
#define LED_CFG_STRIP_CNT		 			5 //109		// Number of LEDs in strip
#define LED_CFG_NUM_COLOR					1		// Number of colors in each LED (RGB = 3)
#define LED_CFG_BYTES_PER_LED 		3		// Each color is represented by 8 bits
#define LED_CFG_BITS_PER_LED	(LED_CFG_STRIP_CNT * LED_CFG_BYTES_PER_LED * 8)	//8 bits = 1 byte

#define TOTAL_BITS_PER_LED        (LED_CFG_BYTES_PER_LED * 8)
#define TMP_LED_SIZE              (50 + LED_CFG_STRIP_CNT * TOTAL_BITS_PER_LED)
#define LED_CFG_RAW_BITS_PER_LED	(LED_CFG_BYTES_PER_LED * 8)
#define NUM_BITS_PER_COLOR			8

//Reset Flag
#define NOT_RESETTING				0			// Not in reset pulse
#define RESET_AT_START			1			// Reset at start of PWM stream
#define RESET_AT_END				2			// Reset at end of PWM stream

//Update Flag
#define NOT_UPDATING 				0
#define UPDATING						1

//Interrupt Events
#define HT_EVENT						0
#define TC_EVENT						1

#define TIM_PERIOD					104
#define HIGH_BIT						2 * (TIM_PERIOD / 3)
#define LOW_BIT							TIM_PERIOD / 3

/* Errors */
#define SYSCONF_ERROR1 		0
#define SYSCONF_ERROR2 		1
#define EN_PWM_ERROR 			2
#define TIM_CONFIG_ERROR 	3
#define TIM_INIT_ERROR 		4
#define GPIO_ERROR				5
#define DMA_ERROR					6

#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC

/* DMA and TIM Structures */
GPIO_InitTypeDef GPIO_InitStruct;
DMA_InitTypeDef DMA_InitStructure;
DMA_HandleTypeDef hdma_tim2_ch1;
TIM_HandleTypeDef htim2;
TIM_OC_InitTypeDef sConfigOC;

/* Capture Compare buffer */
uint32_t tmp_led_data[2 * LED_CFG_RAW_BITS_PER_LED];	// DMA buffer array of size 48
uint8_t LED_colors[LED_CFG_BYTES_PER_LED * LED_CFG_STRIP_CNT];	//Size is total number of bytes

/* Flags */
uint8_t          rst_flag;     /*!< Status if we are sending reset pulse or led data */
volatile uint8_t update_flag;        /*!< Is updating in progress? */
uint32_t         current_led;        /*!< Current LED number we are sending */

void LED_Init(void);

uint8_t set_LED_colors(size_t led_idx, uint8_t red, uint8_t green, uint8_t blue);
uint8_t LED_set_color_all(uint8_t red, uint8_t green, uint8_t blue);
uint8_t write_PWM_data(size_t led_idx, uint32_t* LED_array);
uint8_t LED_reset_pulse(uint8_t rst);
uint8_t led_is_update_finished(void);
uint8_t LED_update(uint8_t block);

void Error_Handler(uint8_t ERROR);
void led_update_sequence(uint8_t event);
void DMA1_Stream5_IRQHandler(void);
void rainbow(size_t i);
void pulse(void);
void pingpong(void);

#endif /* W2812B_H_ */
