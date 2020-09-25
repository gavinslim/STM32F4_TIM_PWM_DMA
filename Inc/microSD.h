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

/*
 * CD Pin - PH1
 * CS Pin - PB12
 * DI Pin - PC3
 * DO Pin - PC2
 * CLK Pin - PB10 (PWM/D6)
 */

SPI_HandleTypeDef hspi2;
UART_HandleTypeDef huart2;

FATFS fs;
FATFS *pfs;
FIL fil;
FRESULT fres;
DWORD fre_clust;
uint32_t totalSpace, freeSpace;

char buffer[100];
char file_name[50];

// File type
typedef enum {
	MP3,
	WAV,
	TXT
} filetype;

typedef enum {
	SD_FAIL,
	SD_PASS
} sd_ret_val;

void Error_Handler(uint8_t ERROR);
void microSD_init (void);
void transmit_uart_SD(char *string);

uint8_t check_microSD_conn (void);
uint8_t check_microSD_mount (void);

sd_ret_val mount_sd(void);
sd_ret_val unmount_sd(void);
sd_ret_val open_file(char* file_name);
sd_ret_val get_freespace(void);
sd_ret_val write_file(void);
sd_ret_val close_file(void);
sd_ret_val read_file(void);

void chk_microSD(void);

// Finding files in microSD card
sd_ret_val find_file (filetype f_type);

#endif /* MICROSD_H_ */
