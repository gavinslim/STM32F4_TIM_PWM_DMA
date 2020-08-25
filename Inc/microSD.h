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

// File type
typedef enum {
	MP3,
	WAV,
	TXT
} filetype;

void Error_Handler(uint8_t ERROR);

void microSD_init (void);
void transmit_uart_SD(char *string);

uint8_t check_microSD_conn (void);
uint8_t check_microSD_mount (void);

void mount_sd(void);
void open_file(char* file_name);
void get_freespace(void);
void write_file(void);
void close_file(void);
void read_file(void);
void unmount(void);

// Finding files in microSD card
void find_txt_file(void);
void find_mp3_file(void);
void find_file (filetype f_type);

#endif /* MICROSD_H_ */
