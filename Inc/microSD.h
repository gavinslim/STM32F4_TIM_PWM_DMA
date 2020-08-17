/*
 * microSD.h
 *
 *  Created on: Jul 28, 2020
 *      Author: Gavin
 */

#ifndef MICROSD_H_
#define MICROSD_H_

#include "stdio.h"
#include "string.h"
#include "stm32f4xx.h"
#include "fatfs.h"
#include "fatfs_sd.h"

// Errors
#define SPI_ERROR 0

// Pins
#define SD_CS_Pin 			GPIO_PIN_12
#define SD_CS_GPIO_Port GPIOB

void microSD_init (void);
uint8_t check_microSD_conn (void);
uint8_t check_microSD_mount (void);
uint8_t open_file (void);

void Error_Handler(uint8_t ERROR);

#endif /* MICROSD_H_ */
