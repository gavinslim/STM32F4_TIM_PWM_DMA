/*
 * w2812b.c
 *
 *  Created on: Jul 28, 2020
 *      Author: Gavin
 */

#include "w2812b.h"
#include "string.h"

// Rainbow
void rainbow(size_t i){
	volatile uint32_t timeout;
	for (size_t i = 0; i < LED_CFG_STRIP_CNT; i++) {
		set_LED_colors((i + 0) % LED_CFG_STRIP_CNT, 0x1F, 0, 0);
		set_LED_colors((i + 1) % LED_CFG_STRIP_CNT, 0x1F, 0, 0);
		set_LED_colors((i + 2) % LED_CFG_STRIP_CNT, 0, 0x1F, 0);
		set_LED_colors((i + 3) % LED_CFG_STRIP_CNT, 0, 0x1F, 0);
		set_LED_colors((i + 4) % LED_CFG_STRIP_CNT, 0, 0, 0x1F);
		set_LED_colors((i + 5) % LED_CFG_STRIP_CNT, 0, 0, 0x1F);

		LED_update(1);
		LED_set_color_all(0, 0, 0);

		timeout = 0x7FFFF;
		while (--timeout) {}
	}
	return;
}

// Pulse
void pulse(void){
	volatile uint32_t timeout;
	for (size_t i = 0; i < LED_CFG_STRIP_CNT; i++) {
		set_LED_colors((i + 0) % LED_CFG_STRIP_CNT, 0x1F, 0, 0x1F);
		//set_LED_colors((i + 1) % LED_CFG_STRIP_CNT, 0, 0x7, 0);
		//set_LED_colors((i + 2) % LED_CFG_STRIP_CNT, 0, 0x1F, 0);
		//set_LED_colors((i + 3) % LED_CFG_STRIP_CNT, 0, 0x1F, 0);
		//set_LED_colors((i + 4) % LED_CFG_STRIP_CNT, 0, 0x7F, 0);
		//set_LED_colors((i + 5) % LED_CFG_STRIP_CNT, 0, 0x1F, 0);
		//set_LED_colors((i + 6) % LED_CFG_STRIP_CNT, 0, 0x1F, 0);
		//set_LED_colors((i + 7) % LED_CFG_STRIP_CNT, 0, 0x7, 0);
		//set_LED_colors((i + 8) % LED_CFG_STRIP_CNT, 0, 0x7, 0);

		LED_update(1);
		LED_set_color_all(0, 0, 0);
		timeout = 0x7FFFF;
		while (--timeout) {}
	}
	return;
}

void pingpong(void){
	volatile uint32_t timeout;
	for (size_t i = 0; i < LED_CFG_STRIP_CNT; i++) {
		set_LED_colors((i + 0) % LED_CFG_STRIP_CNT, 0, 0x7, 0);
		//set_LED_colors((i + 9) % LED_CFG_STRIP_CNT, 0, 0x7, 0);
		LED_update(1);
		LED_set_color_all(0, 0, 0);

		timeout = 0x7FFFF;
		while (--timeout) {}
	}
	for (size_t i = LED_CFG_STRIP_CNT - 2; i > 0; i--) {
		set_LED_colors((i + 0) % LED_CFG_STRIP_CNT, 0, 0x7, 0);
		//set_LED_colors((i + 9) % LED_CFG_STRIP_CNT, 0, 0x7, 0);
		LED_update(1);
		LED_set_color_all(0, 0, 0);

		timeout = 0x7FFFF;
		while (--timeout) {}
	}
	return;
}


uint8_t LED_set_color_all(uint8_t red, uint8_t green, uint8_t blue){
	for (size_t led_idx = 0; led_idx < LED_CFG_STRIP_CNT; led_idx++){
		LED_colors[led_idx * LED_CFG_BYTES_PER_LED + 0] = green;
		LED_colors[led_idx * LED_CFG_BYTES_PER_LED + 1] = red;
		LED_colors[led_idx * LED_CFG_BYTES_PER_LED + 2] = blue;
	}
	return 1;
}

// Set R,G,B values for a specific LED
// index: LED index in array, starting from '0'
// return 1 if successful, else 0 for fail
uint8_t set_LED_colors(size_t led_idx, uint8_t red, uint8_t green, uint8_t blue){
	if (led_idx < LED_CFG_STRIP_CNT){
		LED_colors[led_idx * LED_CFG_BYTES_PER_LED + 0] = green;
		LED_colors[led_idx * LED_CFG_BYTES_PER_LED + 1] = red;
		LED_colors[led_idx * LED_CFG_BYTES_PER_LED + 2] = blue;
		return 1;
	}
	return 0;
}

// Iterate through each byte of LED_colors[] and determine if bit in LED_array[] is a HIGH_BIT or LOW_BIT
uint8_t write_PWM_data(size_t led_idx, uint32_t* LED_array){

	if (led_idx < LED_CFG_STRIP_CNT){

		// Retrieve color hex values for RGB
		uint8_t green_hex_val = LED_colors[LED_CFG_BYTES_PER_LED * led_idx + 0];
		uint8_t red_hex_val = LED_colors[LED_CFG_BYTES_PER_LED * led_idx + 1];
		uint8_t blue_hex_val = LED_colors[LED_CFG_BYTES_PER_LED * led_idx + 2];

		// Set each 24 bits in LED_array based on hex value. Ex: 0x1F = 8'b00011111, therefore LED_array[0] = LOW_BIT
		for (size_t i = 0; i < NUM_BITS_PER_COLOR; i++){
			LED_array[i] = 			(green_hex_val & (1 << (7 - i))) ? HIGH_BIT : LOW_BIT;		// Set green color
			LED_array[i + 8] = 	(red_hex_val & (1 << (7 - i))) ? HIGH_BIT : LOW_BIT;			// Set blue color
			LED_array[i + 16] = (blue_hex_val & (1 << (7 - i))) ? HIGH_BIT : LOW_BIT;		// Set red color
		}

		return 1;
	}
	return 0;
}

uint8_t LED_reset_pulse(uint8_t rst){
	rst_flag = rst;		// Set reset flag

  hdma_tim2_ch1.Init.Mode = DMA_NORMAL;							// Modify DMA config to Normal Mode
  memset(tmp_led_data, 0, sizeof(tmp_led_data));	  // Set all data for tmp_led_data to 0

  // Initialize TIM2 DMA handle
  if (HAL_DMA_Init(&hdma_tim2_ch1) != HAL_OK) {
    Error_Handler(DMA_ERROR);
  }

  // Clear Half Transfer and Transfer Complete flags for DMA1 Stream5
  __HAL_DMA_CLEAR_FLAG(&hdma_tim2_ch1, DMA_FLAG_HTIF1_5);
  __HAL_DMA_CLEAR_FLAG(&hdma_tim2_ch1, DMA_FLAG_TCIF1_5);

  // Disable Half Transfer Interrupt
  __HAL_DMA_DISABLE_IT(&hdma_tim2_ch1, DMA_IT_HT);

  // Enable Complete Transfer Interrupt
  __HAL_DMA_ENABLE_IT(&hdma_tim2_ch1, DMA_IT_TC);

  // Start Reset Pulse. Contains 40 low pulses: 40 x 800kHz = ~50us
  if (HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t*)tmp_led_data, 2 * LED_CFG_RAW_BITS_PER_LED) != HAL_OK){
    Error_Handler(EN_PWM_ERROR);
  }
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
  return 1;
}

// return 1 if not updating, 0 if updating process is in progress
uint8_t led_is_update_finished(void) {
	return !update_flag;
	//return !is_updating;                        /* Return updating flag status */
}

// Block = 1 if want to wait until update process is finished
uint8_t LED_update(uint8_t block){
	if (update_flag) {
		return 0;
	}
	update_flag = 1;		// Signify that LEDs are being updated

	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	LED_reset_pulse(RESET_AT_START);	// Reset LED scheme with 50us pulse low

	if (block){
		while(!led_is_update_finished());
	}

	return 1;
}

// Update led sequence. Called on TC and HT events.
// At HT event, first 24 elements are transferred
// At TC event, second 24 elements are transferred
void led_update_sequence(uint8_t event) {

	event = !!event;       // Toggle transfer-complete flag

	// Check for reset pulse at the end of the PWM stream
	if (rst_flag == RESET_AT_END){

		HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);	// Stop DMA
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

		update_flag = NOT_UPDATING;										// No longer updating
		return;
	}

	// Check if reset pulse happened at the start of the PWM stream
	if (rst_flag == RESET_AT_START){
		if (!event) { return; }		// If HT event, return and wait until TC event

		HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);	// Disable PWM generation to update LED sequence
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

		rst_flag = NOT_RESETTING;											// No longer resetting

		current_led = 0;
	} else {

	// Not resetting, move to next LED and process data
		current_led++;	// Move to next LED
		//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
	}

	// Prepare data for the next LED
	if (current_led < LED_CFG_STRIP_CNT){

		// 1. Write PWM signal for first LED (current_led = 0) into first-half of DMA buffer
		// 2. Write PWM signal for second LED (current_led = 1) into second-half of DMA buffer
		// 3. Set DMA to circular, clear interrupt flags and start DMA PWM generation
		// 4. Write PWM signal for third and beyond LEDs (current_led >= 2)
		if ((current_led == 0) || !event) {
			write_PWM_data(current_led, &tmp_led_data[0]);	// Step 1
		} else {
			write_PWM_data(current_led, &tmp_led_data[LED_CFG_RAW_BITS_PER_LED]);	// Step 4
		}

		if (current_led == 0){

			current_led++;
			write_PWM_data(current_led, &tmp_led_data[LED_CFG_RAW_BITS_PER_LED]);	// Step 2

		  hdma_tim2_ch1.Init.Mode = DMA_CIRCULAR;										// Step 3
		  // Initialize TIM2 DMA handle
		  if (HAL_DMA_Init(&hdma_tim2_ch1) != HAL_OK) {
		    Error_Handler(DMA_ERROR);
		  }

		  __HAL_DMA_CLEAR_FLAG(&hdma_tim2_ch1, DMA_FLAG_HTIF1_5);
		  __HAL_DMA_CLEAR_FLAG(&hdma_tim2_ch1, DMA_FLAG_TCIF1_5);

		  __HAL_DMA_ENABLE_IT(&hdma_tim2_ch1, DMA_IT_HT);	// Enable interrupt for half-transfer event

		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
		  // Start PWM generation
		  if (HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t*)tmp_led_data, 2 * LED_CFG_RAW_BITS_PER_LED) != HAL_OK){
		    Error_Handler(EN_PWM_ERROR);
		  }
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
		}

	// When all LEDs have been lit up, wait for all data to be transmitted before modifying DMA
	// !TC && (LED_CFG_STRIP_CNT & 0x01): Half-Transfer event occurred and even-numbered LED
	// TC && !(LED_CFG_STRIP_CNT & 0x01): Transfer-Complete event occured and odd-numbered LED
	} else if ((!event && (LED_CFG_STRIP_CNT & 0x01)) || (event && !(LED_CFG_STRIP_CNT & 0x01))) {
		HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
		LED_reset_pulse(RESET_AT_END);
	}

}


// DMA1 Stream 5 Global Interrupt
void DMA1_Stream5_IRQHandler(void){
	// Check for Half-Transfer (HT) event
	if (__HAL_DMA_GET_FLAG(&hdma_tim2_ch1, DMA_FLAG_HTIF1_5)){
	  __HAL_DMA_CLEAR_FLAG(&hdma_tim2_ch1, DMA_FLAG_HTIF1_5);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	  led_update_sequence(HT_EVENT);
	// Check for Transfer-Complete (TC) event
	} else if (__HAL_DMA_GET_FLAG(&hdma_tim2_ch1, DMA_FLAG_TCIF1_5)) {
	  __HAL_DMA_CLEAR_FLAG(&hdma_tim2_ch1, DMA_FLAG_TCIF1_5);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	  led_update_sequence(TC_EVENT);
	}
}


void LED_Init(void)
{
  /* ------------- */
  /*  GPIO Config  */
  /* ------------- */

  // Configure TIM2_Channel 1 (PA0) as output, push-pull and alternate function mode
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Configure GPIO pin: PA8, debugging for DMA IRQ
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Configure GPIO pin: PB0, PB10, debugging for is_updating flag
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* -------------- */
  /* TIM2 Channel 1 */
  /* -------------- */

  // Enable TIM2 clock
  __HAL_RCC_TIM2_CLK_ENABLE();

  // TIM Time Base handle Structure definition
  // Period = TIM2_counter_clk / PWM_freq - 1
  // Period = 84MHz / 800kHz - 1 = 104
  htim2.Instance               = TIM2;
  htim2.Init.Prescaler         = 0;										// Set to 0 to acheive max frequency for timer at 84MHz
  htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim2.Init.Period            = TIM_PERIOD;
  htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;	//TIM_AUTORELOAD_PRELOAD_DISABLE

  // TIM2 interrupt Init
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);

  // Note: HAL_TIM_PWM_Init() calls HAL_TIM_PWM_MspInit()
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
    Error_Handler(TIM_INIT_ERROR);     // Initialization Error
  }

  // Set TIM Output Compare (OC) Configuration Structure definition
  sConfigOC.OCMode       = TIM_OCMODE_PWM1;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.Pulse        = 0;
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  // Apply TIM OC configs to htim2 (TIM2)
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler(TIM_CONFIG_ERROR); // Configuration Error
  }

  /* ------------------------------ */
  /* DMA1 Channel 3 Stream 5 Config */
  /* ------------------------------ */

  // Enable DMA1 clock
	__HAL_RCC_DMA1_CLK_ENABLE();

	// DMA handle Structure definition (Based on RM0383 STM32F411 Ref Manual (Table 27) TIM2_CH1 corresponds to DMA1 Channel 3 Stream 5)
  hdma_tim2_ch1.Instance = DMA1_Stream5;
  hdma_tim2_ch1.Init.Channel = DMA_CHANNEL_3;
  hdma_tim2_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;							// Memory to Peripheral mode
  hdma_tim2_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_tim2_ch1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_tim2_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD ;
  hdma_tim2_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD ;
  hdma_tim2_ch1.Init.Mode = DMA_CIRCULAR;														// Set in circular mode
  hdma_tim2_ch1.Init.Priority = DMA_PRIORITY_LOW;										// Low priority
  hdma_tim2_ch1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  hdma_tim2_ch1.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_tim2_ch1.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_tim2_ch1.Init.PeriphBurst = DMA_PBURST_SINGLE;

  // Enable Half-Transfer and Full-Transfer complete interrupts
  __HAL_DMA_ENABLE_IT(&hdma_tim2_ch1, (DMA_IT_TC | DMA_IT_HT));

  // Linking a PPP peripheral to DMA structure pointer (PPP = STM32 peripheral or block)
  // hdma[TIM_DMA_ID_CC1] = Capture/Compare 1 DMA requests peripheral
  // Link htim2 (TIM peripheral) to hdma_tim2_ch1 (DMA struc pointer) with TIM_DMA_ID_CC1
  __HAL_LINKDMA(&htim2, hdma[TIM_DMA_ID_CC1], hdma_tim2_ch1);

  // Initialize TIM2 DMA handle
  if (HAL_DMA_Init(&hdma_tim2_ch1) != HAL_OK) {
    Error_Handler(DMA_ERROR);
  }

  // ##-2- Configure the NVIC for DMA #########################################
  // NVIC configuration for DMA transfer complete interrupt
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
}
