/*
 * microSD.c
 *
 *  Created on: Aug 7, 2020
 *      Author: Gavin
 */

#include "microSD.h"


SPI_HandleTypeDef hspi2;

/*
FATFS fs;
FATFS *pfs;
FIL fil;
FRESULT fres;
DWORD fre_clust;
uint32_t totalSpace, freeSpace;
*/
//char buffer[100];

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

/*
uint8_t check_microSD_mount (void) {
	fres = f_mount(&fs, "", 0);
	if (fres == FR_OK) {
		return 1;
    //transmit_uart("Micro SD card is mounted successfully!\n");
	} else if (fres != FR_OK) {
		return 0;
		//transmit_uart("Micro SD card's mount error!\n");
	}
	return 0;
}

uint8_t open_file (void) {
	// FA_OPEN_APPEND opens file if it exists and if not then creates it,
	// the pointer is set at the end of the file for appending
	fres = f_open(&fil, "log-file.txt", FA_OPEN_APPEND | FA_WRITE | FA_READ);
	if (fres == FR_OK) {
		return 1;
		//transmit_uart("File opened for reading and checking the free space.\n");
	} else if (fres != FR_OK) {
		return 0;
		//transmit_uart("File was not opened for reading and checking the free space!\n");
	}
	return 0;
}
*/

