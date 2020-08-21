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
#define OPEN_ERROR 8
#define SPI_ERROR  9

// Pins
#define SD_CS_Pin 			GPIO_PIN_12
#define SD_CS_GPIO_Port GPIOB

void Error_Handler(uint8_t ERROR);

void microSD_init (void);
uint8_t check_microSD_conn (void);
uint8_t check_microSD_mount (void);

uint8_t mount_sd(void);
uint8_t open_file(char* file_name);
uint32_t get_freespace(void);
uint8_t write_file(void);
uint8_t close_file(void);
uint8_t read_file(char* outStr);
uint8_t unmount(void);

#endif /* MICROSD_H_ */
