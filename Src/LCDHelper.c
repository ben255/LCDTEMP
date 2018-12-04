/*
 * LCDHelper.c
 *
 *  Created on: 9 maj 2018
 *      Author: kabe1725
 */
#include "stm32f4xx_hal.h"
#include "LCDHelper.h"

void TextLCD_Strobe(TextLCDType * lcd);
void TextLCD_Cmd(TextLCDType *lcd, uint8_t cmd);
void TextLCD_Data(TextLCDType *lcd, uint8_t data);

void TextLCD_Init(TextLCDType *lcd, GPIO_TypeDef * controlPort, uint16_t rsPin, uint16_t rwPin, uint16_t ePin, GPIO_TypeDef * dataPort, uint16_t dataPins){
	lcd->controlPort = controlPort;
	lcd->dataPort = dataPort;
	lcd->rsPin = rsPin;
	lcd->rwPin = rwPin;
	lcd->ePin = ePin;
	lcd->dataPins = dataPins;


	HAL_Delay(5);
	  HAL_GPIO_WritePin(lcd->controlPort, LCD_RW_Pin, 0);
	  HAL_GPIO_WritePin(lcd->controlPort, LCD_RS_Pin, 0);

	  HAL_GPIO_WritePin(lcd->dataPort, 0x38, 1);
	  TextLCD_Strobe(lcd);
	  HAL_GPIO_WritePin(lcd->dataPort, 0x38, 0);
	  HAL_Delay(5);

	  HAL_GPIO_WritePin(lcd->dataPort, 0x06, 1);
	  TextLCD_Strobe(lcd);
	  HAL_GPIO_WritePin(lcd->dataPort, 0x06, 0);
	  HAL_Delay(5);

	  HAL_GPIO_WritePin(lcd->dataPort, 0x0E, 1);
	  TextLCD_Strobe(lcd);
	  HAL_GPIO_WritePin(lcd->dataPort, 0x0E, 0);
	  HAL_Delay(5);

	  HAL_GPIO_WritePin(lcd->dataPort, 0x01, 1);
	  TextLCD_Strobe(lcd);
	  HAL_GPIO_WritePin(lcd->dataPort, 0x01, 0);
	  HAL_Delay(5);

}

void TextLCD_Home(TextLCDType *lcd){

	HAL_GPIO_WritePin(lcd->controlPort, LCD_RW_Pin, 0);
	HAL_GPIO_WritePin(lcd->controlPort, LCD_RS_Pin, 0);

	HAL_GPIO_WritePin(lcd->dataPort, 0x03, 1);
	TextLCD_Strobe(lcd);
	HAL_GPIO_WritePin(lcd->dataPort, 0x03, 0);
	HAL_Delay(5);


}

void TextLCD_Clear(TextLCDType *lcd){

	HAL_GPIO_WritePin(lcd->controlPort, LCD_RW_Pin, 0);
	HAL_GPIO_WritePin(lcd->controlPort, LCD_RS_Pin, 0);

	HAL_GPIO_WritePin(lcd->dataPort, 0x01, 1);
	TextLCD_Strobe(lcd);
	HAL_GPIO_WritePin(lcd->dataPort, 0x01, 0);
	HAL_Delay(5);

}

void TextLCD_Position(TextLCDType * lcd, int x, int y){
	HAL_GPIO_WritePin(lcd->controlPort, LCD_RW_Pin, 0);
	HAL_GPIO_WritePin(lcd->controlPort, LCD_RS_Pin, 0);
	uint16_t position = 0x00;

	if(y == 0)
		position = 0x80 + x;
	else if(y == 1)
		position = 0xC0 + x;

	HAL_GPIO_WritePin(lcd->dataPort, position, 1);
	TextLCD_Strobe(lcd);
	HAL_GPIO_WritePin(lcd->dataPort, position, 0);
	HAL_Delay(5);
}

void TextLCD_Putchar(TextLCDType *lcd, uint8_t data){
	HAL_GPIO_WritePin(lcd->controlPort, LCD_RW_Pin, 0);
	HAL_GPIO_WritePin(lcd->controlPort, LCD_RS_Pin, 1);

	HAL_GPIO_WritePin(lcd->dataPort, data, 1);
	TextLCD_Strobe(lcd);
	HAL_GPIO_WritePin(lcd->dataPort, data, 0);
	HAL_Delay(5);
}

void TextLCD_Puts(TextLCDType *lcd, char *string){
	while(*string)
		TextLCD_Putchar(lcd, *string++);
}



void TextLCD_Printf(TextLCDType *lcd, char * message){

}

void TextLCD_Strobe(TextLCDType * lcd){
	HAL_GPIO_WritePin(lcd->controlPort, lcd->ePin, 1);
	HAL_Delay(5);
	HAL_GPIO_WritePin(lcd->controlPort, lcd->ePin, 0);
	HAL_Delay(5);
}

void TextLCD_Cmd(TextLCDType *lcd, uint8_t cmd){

}

void TextLCD_Data(TextLCDType *lcd, uint8_t data){

}
