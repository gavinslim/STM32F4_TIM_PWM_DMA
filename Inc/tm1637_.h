#ifndef TM1637_H_
#define TM1637_H_

#include "stm32f4xx.h"

#define TIM_1MHZ_PERIOD				83
#define TIM_CONFIG_ERROR  3
#define TIM_INIT_ERROR 		4

void tm1637Init(void);
void tm1637DisplayDecimal(int v, int displaySeparator);
void tm1637DisplayTime(int min, int sec, int displaySeparator);
void tm1637SetBrightness(char brightness);
void _tm1637Start(void);

void Error_Handler(uint8_t ERROR);

/* TIM Strcutures */
TIM_HandleTypeDef htim3;
TIM_OC_InitTypeDef sConfigOC;


#endif /* TM1637_H_ */
