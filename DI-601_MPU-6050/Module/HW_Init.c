/*
 * HW_Init.c
 *
 * Created: 2/15/2019 9:49:33 AM
 *  Author: NANO
 */ 

/*************** Includes ***************/

#include "HW_Init.h"

/*************** Includes ***************/

void HW_Init(void)  /* Function for initialize hardware */
{
	/* Port Init */
	
	_LCD_PORT_DIR = _ALL_OUTPUT;
	
	_I2C_PORT_DIR = ( 1 << _SDA ) | _SCL;
	
	/* LCD Config */
	
 	Lcd_Config.Font = _FONT_5X7;
 	Lcd_Config.Mode = _4BIT_INTERFACE;
 	Lcd_Config.NumberOfLine = _2LINE_DISPLAY;
 	
 	Lcd_Init();
	
	/* MPU6050 Init */
	
	if ( MPU6050_AutoInit(100) == _MPU_OK )
	{
		Lcd_GotoXY(0,0); /* Goto xy location in LCD */
		Lcd_PutString("Sensor init     "); /* Show value on LCD */
	}
	else
	{
		Lcd_GotoXY(0,0); /* Goto xy location in LCD */
		Lcd_PutString("Sensor not init "); /* Show value on LCD */
	}
	_Delay_Ms(_WAIT_FOR_NEXT_SHOW); /* Wait */
	Lcd_Clear();
	
}
