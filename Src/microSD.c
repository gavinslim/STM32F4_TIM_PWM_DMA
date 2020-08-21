/*
 * microSD.c
 *
 *  Created on: Aug 7, 2020
 *      Author: Gavin
 */

#include "microSD.h"

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

void microSD_init (void){
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Configure GPIO pin : PC2_Pin
	/*
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	*/

  // Configure GPIO pin : PH1_Pin
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_1, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  // Configure GPIO pin : SD_CS_Pin
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);


  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;	//SPI_BAUDRATEPRESCALER_2
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler(SPI_ERROR);
  }
}

uint8_t check_microSD_conn (void) {
	if(!(HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_1))){
		return 0;
	}
	return 1;
}

uint8_t mount_sd(void){
	fres = f_mount(&fs, "", 0);
	if (fres == FR_OK) {
		return 1;
	} else {
		return 0;
	}
}

uint8_t open_file(char* file_name){
	if(f_open(&fil, file_name, FA_OPEN_ALWAYS | FA_READ | FA_WRITE) != FR_OK) {
		return 0;
	} else {
		return 1;
	}
}

uint32_t get_freespace(void){
	fres = f_getfree("", &fre_clust, &pfs);
	totalSpace = (uint32_t) ((pfs->n_fatent - 2) * pfs->csize * 0.5);
	freeSpace = (uint32_t) (fre_clust * pfs->csize * 0.5);

	if (fres == FR_OK) {
		return freeSpace;
	} else if (fres != FR_OK) {
		return 0;
	}
	return 0;
}

uint8_t write_file(void){
	for (uint8_t i = 0; i < 10; i++) {
		f_puts("This text is written in the file.\r\n", &fil);
	}
	return 1;
}

uint8_t close_file(void){
	fres = f_close(&fil);
	if (fres == FR_OK) {
		return 1;
	} else if (fres != FR_OK) {
		return 0;
	}
	return 0;
}

uint8_t read_file(char* outStr){
	while (f_gets(buffer, sizeof(buffer), &fil)) {
		char mRd[100];
		sprintf(mRd, "%s", buffer);

		for (int i = 0; i < 100; ++i){
			outStr[i] = mRd[i];
		}
	}
	return 1;
}

uint8_t unmount(void){
	f_mount(NULL, "", 1);
	if (fres == FR_OK) {
		return 1;
	} else if (fres != FR_OK) {
		return 0;
	}
}

/*
char mRd[100];
if (read_file(mRd)){
	transmit_uart("PASS - Done reading");
	transmit_uart(mRd);
}
*/
/*
fres = f_mount(&fs, "", 0);
if (fres == FR_OK) {
	transmit_uart("Micro SD card is mounted successfully!\r\n");
} else if (fres != FR_OK) {
	transmit_uart("Micro SD card's mount error!\r\n");
}

if(f_open(&fil, "first.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE) != FR_OK) {
	transmit_uart("FAIL - File not opened\r\n");
	Error_Handler(OPEN_ERROR);
} else {
	transmit_uart("PASS - File successfully opened\r\n");
}

fres = f_getfree("", &fre_clust, &pfs);
totalSpace = (uint32_t) ((pfs->n_fatent - 2) * pfs->csize * 0.5);
freeSpace = (uint32_t) (fre_clust * pfs->csize * 0.5);

char mSz[12];
sprintf(mSz, "%lu", freeSpace);

if (fres == FR_OK) {
	transmit_uart("The free space is: ");
	transmit_uart(mSz);
	transmit_uart("\r\n");
} else if (fres != FR_OK) {
	transmit_uart("The free space could not be determined!\r\n");
}

for (uint8_t i = 0; i < 10; i++) {
	f_puts("This text is written in the file.\r\n", &fil);
}

fres = f_close(&fil);
if (fres == FR_OK) {
	transmit_uart("The file is closed.\r\n");
} else if (fres != FR_OK) {
	transmit_uart("The file was not closed.\r\n");
}

if(f_open(&fil, "first.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE) != FR_OK) {
	transmit_uart("FAIL - File not opened\r\n");
	Error_Handler(OPEN_ERROR);
} else {
	transmit_uart("PASS - File successfully opened for reading\r\n");
}

transmit_uart("--------------------------------------\n\r");
while (f_gets(buffer, sizeof(buffer), &fil)) {
	char mRd[100];
	sprintf(mRd, "%s", buffer);
	transmit_uart(mRd);
	transmit_uart("\r");
}
transmit_uart("--------------------------------------\n\r");

// Close file
fres = f_close(&fil);
if (fres == FR_OK) {
	transmit_uart("The file is closed.\r\n");
} else if (fres != FR_OK) {
	transmit_uart("The file was not closed.\r\n");
}

f_mount(NULL, "", 1);
if (fres == FR_OK) {
	transmit_uart("The Micro SD card is unmounted!\r\n");
} else if (fres != FR_OK) {
	transmit_uart("The Micro SD was not unmounted!");
}
*/

