/*
 * u8g2_porting.h
 *
 *  Created on: Jan 2, 2026
 *      Author: Z440
 */

#ifndef INC_U8G2_PORTING_H_
#define INC_U8G2_PORTING_H_

#include "u8g2.h"


u8g2_t InfoDisplay;



uint8_t u8g2_gpio_and_delay_stm32(U8X8_UNUSED u8x8_t *u8x8,
U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int,
U8X8_UNUSED void *arg_ptr) {
	switch (msg) {
	//Initialize SPI peripheral
	case U8X8_MSG_GPIO_AND_DELAY_INIT:
		/* HAL initialization contains all what we need so we can skip this part. */
		break;

		//Function which implements a delay, arg_int contains the amount of ms
	case U8X8_MSG_DELAY_MILLI:
		HAL_Delay(arg_int);
		break;

		//Function which delays 10us
	case U8X8_MSG_DELAY_10MICRO:
		for (uint16_t n = 0; n < 320; n++)
		{
			__NOP();
		}
		break;

		//Function which delays 100ns
	case U8X8_MSG_DELAY_100NANO:
		__NOP();
		break;

		//Function to define the logic level of the clockline
	case U8X8_MSG_GPIO_SPI_CLOCK:
//		if (arg_int)
//			HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, RESET);
//		else
//			HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, SET);
//		break;

		//Function to define the logic level of the data line to the display
	case U8X8_MSG_GPIO_SPI_DATA:

//		if (arg_int)
//		{
//			HAL_GPIO_WritePin(MOSI_GPIO_Port, MOSI_Pin, SET);
//		}
//		else
//		{
//			HAL_GPIO_WritePin(MOSI_GPIO_Port, MOSI_Pin, RESET);
//		}
//		break;

		// Function to define the logic level of the CS line
	case U8X8_MSG_GPIO_CS:
//		if (arg_int)
//			HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, RESET);
//		else
//			HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, SET);
//		break;

		//Function to define the logic level of the Data/ Command line
	case U8X8_MSG_GPIO_DC:
//			if (arg_int) HAL_GPIO_WritePin(CD_LCD_PORT, CD_LCD_PIN, SET);
//			else HAL_GPIO_WritePin(CD_LCD_PORT, CD_LCD_PIN, RESET);
		break;

		//Function to define the logic level of the RESET line
	case U8X8_MSG_GPIO_RESET:
//		if (arg_int)
//			HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, SET);
//		else
//			HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, RESET);
//		break;

	default:
		return 0; //A message was received which is not implemented, return 0 to indicate an error
	}

	return 1; // command processed successfully.
}



uint8_t u8g2_i2c_stm32(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
	static uint8_t buffer[32]; /* u8g2/u8x8 will never send more than 32 bytes between START_TRANSFER and END_TRANSFER */
	static uint8_t buf_idx;
	uint8_t *data;

	switch (msg)
	{
	case U8X8_MSG_BYTE_SEND:
		data = (uint8_t*) arg_ptr;
		while (arg_int > 0)
		{
			buffer[buf_idx++] = *data;
			data++;
			arg_int--;
		}
		break;


	case U8X8_MSG_BYTE_START_TRANSFER:
		buf_idx = 0;
		break;


	case U8X8_MSG_BYTE_END_TRANSFER:
		HAL_I2C_Master_Transmit(&hi2c1, u8x8_GetI2CAddress(u8x8), buffer, buf_idx, 1000);
		break;


	default:
		return 0;
	}
	return 1;
}



void initDisplay(u8g2_t *u8g2, const u8g2_cb_t *rotation, u8x8_msg_cb byte_cb, u8x8_msg_cb gpio_and_delay_cb, const uint8_t *font) {
	u8g2_Setup_ssd1306_i2c_128x64_noname_f(u8g2, rotation, byte_cb, gpio_and_delay_cb);
	u8g2_InitDisplay(u8g2); // send init sequence to the display, display is in sleep mode after this,
	u8g2_SetPowerSave(u8g2, 0); // wake up display

	u8g2_ClearDisplay(u8g2);
	u8g2_SetFont(u8g2, font); //u8g2_font_ncenB14_tr
	u8g2_DrawStr(u8g2, 0, 15, "Hello!");
	u8g2_SendBuffer(u8g2);
}

#endif /* INC_U8G2_PORTING_H_ */
