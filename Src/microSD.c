/*
 * microSD.c
 *
 *  Created on: Aug 7, 2020
 *  Author:     Gavin
 *  Purpose:    Functions for controlling microSD card
 */

#include "microSD.h"



void microSD_init (void){
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = {0};

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

void transmit_uart_SD(char *string){
	uint8_t len = strlen(string);
	HAL_UART_Transmit(&huart2, (uint8_t*) string, len, 200);
}

sd_ret_val check_microSD_conn (void) {
	if(!(HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_1))){
		return SD_FAIL;
	}
	return SD_PASS;
}

sd_ret_val mount_sd(void){
	//fres = f_mount(&fs, "", 0);
	pfs = malloc(sizeof (FATFS));
	fres = f_mount(pfs, "", 0);
	if (fres == FR_OK) {
		transmit_uart_SD("PASS - MicroSD card mounted successfully!\r\n");
		return SD_PASS;
	} else {
		transmit_uart_SD("FAIL - MicroSD card's mount error!\r\n");
		return SD_FAIL;
	}
	return SD_FAIL;
}

sd_ret_val unmount_sd(void){
	//f_mount(NULL, "", 1);
	f_mount(0, "", 0);
	if (fres == FR_OK) {
		transmit_uart_SD("PASS - MicroSD successfully unmounted.\r\n");
		free(pfs);
		return SD_PASS;
	} else if (fres != FR_OK) {
		transmit_uart_SD("FAIL - MicroSD failed to unmount.\r\n");
		free(pfs);
		return SD_FAIL;
	}
	return SD_FAIL;
}

sd_ret_val open_file(char* file_name){
	if(f_open(&fil, file_name, FA_OPEN_ALWAYS | FA_READ | FA_WRITE) != FR_OK) {
		transmit_uart_SD("FAIL - File not opened.\r\n");
		return SD_FAIL;
	} else {
		transmit_uart_SD("PASS - File successfully opened.\r\n");
		return SD_PASS;
	}
	return SD_FAIL;
}

sd_ret_val get_freespace(void){
	fres = f_getfree("", &fre_clust, &pfs);
	totalSpace = (uint32_t) ((pfs->n_fatent - 2) * pfs->csize * 0.5);
	freeSpace = (uint32_t) (fre_clust * pfs->csize * 0.5);

	char mSz[12];
	sprintf(mSz, "%lu", freeSpace);

	if (fres == FR_OK) {
		transmit_uart_SD("PASS - Free space (kb): ");
		transmit_uart_SD(mSz);
		transmit_uart_SD("\r\n");
		return SD_PASS;
	} else if (fres != FR_OK) {
		transmit_uart_SD("FAIL - Free space failed to be determined.\r\n");
		return SD_FAIL;
	}
	return SD_FAIL;
}

sd_ret_val write_file(void){
	for (uint8_t i = 0; i < 10; i++) {
		f_puts("This text is written in the file.\n", &fil);
	}
	transmit_uart_SD("PASS - Writing complete.\r\n");
	return SD_PASS;
}

sd_ret_val close_file(void){
	fres = f_close(&fil);
	if (fres == FR_OK) {
		transmit_uart_SD("PASS - File successfully closed.\r\n");
		return SD_PASS;
	} else if (fres != FR_OK) {
		transmit_uart_SD("FAIL - File failed to close.\r\n");
		return SD_FAIL;
	}
	return SD_FAIL;
}

sd_ret_val read_file(void){
	transmit_uart_SD("Reading File...\r\n");
	transmit_uart_SD("Contents of File:\r\n");
	transmit_uart_SD("-----------------\r\n");

	while (f_gets(buffer, sizeof(buffer), &fil)) {
		char mRd[100];
		sprintf(mRd, "%s", buffer);

		transmit_uart_SD(mRd);
		transmit_uart_SD("\r\n");
	}

	transmit_uart_SD("-----------------\r\n");
	return SD_PASS;
}


sd_ret_val find_file (filetype f_type)
{
    FRESULT fr;     /* Return value */
    DIR dj;         /* Directory object */
    FILINFO fno;    /* File information */

    uint8_t idx = 1;
    char index[5];
    unsigned long file_size_kB;
    char file_size[10];

    char file_fmt_name[5];
    char file_fmt[10];

    switch(f_type) {
    case MP3:
    	strcpy(file_fmt_name, "MP3");
    	strcpy(file_fmt, "???*.mp3");
    	break;
    case WAV:
    	strcpy(file_fmt_name, "WAV");
    	strcpy(file_fmt, "???*.wav");
    	break;
    case TXT:
    	strcpy(file_fmt_name, "TXT");
    	strcpy(file_fmt, "???*.txt");
    	break;
    default:
    	transmit_uart_SD("FAIL - Invalid file type, find_file().");
    	return SD_FAIL;
    }

    transmit_uart_SD("\r\n");
    transmit_uart_SD(file_fmt_name);
    transmit_uart_SD(" Files Found:\n\r");
    transmit_uart_SD("----------------\n\r");
    //fr = f_findfirst(&dj, &fno, "", "dsc*.mp3");  /* Start to search for photo files */
    fr = f_findfirst(&dj, &fno, "", file_fmt);

    while (fr == FR_OK && fno.fname[0]) {         /* Repeat while an item is found */

    	// Determine file number
      sprintf(index, "%u", idx);

      // Determine file size in Kilobytes
      file_size_kB = fno.fsize / 1000;
      sprintf(file_size, "%lu", file_size_kB);

      // Print out file index
      transmit_uart_SD("File ");
    	transmit_uart_SD(index);
    	transmit_uart_SD(" - ");

    	// Print out file name
    	transmit_uart_SD("Name: ");
    	transmit_uart_SD(fno.fname);							/* Print the object name */

    	// Print out file size
    	transmit_uart_SD(" | Size: ");
    	transmit_uart_SD(file_size);
    	transmit_uart_SD(" kB\r\n");

    	fr = f_findnext(&dj, &fno);               /* Search for next item */

    	idx++;
    }
    transmit_uart_SD("----------------\n\r");
    f_closedir(&dj);
    return SD_PASS;
}

void chk_microSD(void){
	if(mount_sd()){
		get_freespace();
		find_file(TXT);
		find_file(MP3);
		find_file(WAV);
		unmount_sd();
	}
}
