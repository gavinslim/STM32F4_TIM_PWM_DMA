/*
 * w2812b.h
 *
 *  Created on: Jul 28, 2020
 *      Author: Gavin
 */

#ifndef W2812B_H_
#define W2812B_H_

#include <stdint.h>

#define LED_CFG_STRIP_CNT		 			1		// Number of LEDs in strip
#define LED_CFG_NUM_COLOR					1		// Number of colors in each LED (RGB = 3)
#define LED_CFG_BYTES_PER_LED 		3		// Each color is represented by 8 bits
#define LED_CFG_BITS_PER_LED	(LED_CFG_STRIP_CNT * LED_CFG_BYTES_PER_LED * 8)	//8 bits = 1 byte

#define TMP_LED_SIZE              74
//uint32_t * resetPulse(uint32_t uwTimerPeriod);

static uint8_t LED_colors[LED_CFG_BYTES_PER_LED * LED_CFG_STRIP_CNT];

#endif /* W2812B_H_ */
