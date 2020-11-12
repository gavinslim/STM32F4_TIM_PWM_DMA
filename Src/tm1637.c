/*
 * tm1637.c
 *
 *  Created on: Sep 25, 2020
 *      Author: Gavin
 */


#include <tm1637.h>


#define CLK_PORT GPIOB
#define DIO_PORT GPIOB
#define CLK_PIN GPIO_PIN_15
#define DIO_PIN GPIO_PIN_14
#define CLK_PORT_CLK_ENABLE __HAL_RCC_GPIOB_CLK_ENABLE
#define DIO_PORT_CLK_ENABLE __HAL_RCC_GPIOB_CLK_ENABLE


void TM1637_Init(void){
	__HAL_RCC_GPIOB_CLK_ENABLE();
	/*
	 * D0	 - PB15
	 * CLK - PB14
	 */
	GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Configure GPIO pin : PB15 and PB14
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void write_byte(unsigned char b){

}

void delaymicrosec(unsigned int i){
	volatile unsigned int j;
	while(i--){
		for(j=0; j<10; j++){
			__NOP();
		}
	}
}


void set_dio_low(void){
	HAL_GPIO_WritePin(DIO_PORT, DIO_PIN, GPIO_PIN_RESET);
}

void set_dio_high(void){
	HAL_GPIO_WritePin(DIO_PORT, DIO_PIN, GPIO_PIN_SET);
}

void set_clk_low(void){
	HAL_GPIO_WritePin(CLK_PORT, CLK_PIN, GPIO_PIN_RESET);
}

void set_clk_high(void){
	HAL_GPIO_WritePin(CLK_PORT, CLK_PIN, GPIO_PIN_SET);
}

