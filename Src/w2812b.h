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

//LED Strip variables
#define LED_CFG_STRIP_CNT		 			2 //109		// Number of LEDs in strip
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



//uint8_t set_LED_colors(size_t led_idx, uint8_t red, uint8_t green, uint8_t blue);


#endif /* W2812B_H_ */
