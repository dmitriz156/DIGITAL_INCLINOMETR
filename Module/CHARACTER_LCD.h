/*
------------------------------------------------------------------------------
 * File   : CHARACTER_LCD.h
 * Author : Majid Derhambakhsh
 * Version: V0.0.5
 * Created: 2/15/2019 9:28:18 AM
 * Brief  :
 * Support: Majid.do16@gmail.com
------------------------------------------------------------------------------
 * @description:
 *
 * @attention  :    This file is for AVR/ARM microcontroller
------------------------------------------------------------------------------
*/

#ifndef __CHARACTER_LCD_H_
#define __CHARACTER_LCD_H_

/* ------------------------------------- Includes ------------------------------------- */

/*----------------------------------------------------------*/

#ifdef __CODEVISIONAVR__  /* Check compiler */

#include "GPIO/gpio_unit.h"        /* Import AVR IO library */
#include <delay.h>       /* Import delay library */

/*----------------------------------------------------------*/

#elif defined(__GNUC__)  /* Check compiler */

#include "GPIO/gpio_unit.h"        /* Import HAL library */
#include <util/delay.h>  /* Import delay library */

/*----------------------------------------------------------*/

#elif defined(USE_HAL_DRIVER)  /* Check compiler */

#include "stm32f1xx_hal.h"        /* Import HAL library */

/*----------------------------------------------------------*/

#else                     /* Compiler not found */

#error Chip or GPIO Library not supported  /* Send error */

#endif /* __CODEVISIONAVR__ */

/*----------------------------------------------------------*/

#include "CHARACTER_LCD_CONFIG.h" /* Import config file */
#include <stdint.h> /* Import standard int */

/* -------------------------------------- Define -------------------------------------- */

/* -------------- Commands -------------- */

#define _CLEAR_DISPLAY                                0x01 /* Clear display command */
#define _RETURN_HOME                                  0x02 /* Return home command */
#define _SHIFT_CURSOR_TO_LEFT_AFTER_WRITE_CHARACTER   0x04 /* Shift cursor mode command */
#define _SHIFT_CURSOR_TO_RIGHT_AFTER_WRITE_CHARACTER  0x06 /* Shift cursor mode command */
#define _SHIFT_CURSOR_TO_LEFT                         0x10 /* Shift cursor command */
#define _SHIFT_CURSOR_TO_RIGHT                        0x14 /* Shift cursor command */
#define _SHIFT_TO_LEFT                                0x18 /* Shift LCD command */
#define _SHIFT_TO_RIGHT                               0x1C /* Shift LCD command */
#define _DISPLAY_OFF                                  0x08 /* Display off command */
#define _DISABLE_LCD_CURSOR                           0x0C /* Disable cursor command */
#define _BLINK_LCD_CURSOR                             0x0D /* Blink cursor command */
#define _FIXED_LCD_CURSOR                             0x0E /* Fixed cursor command */

/* ----------- DD RAM ADDRESS ----------- */

#define _FIRST_ROW  0x80 /* First line address */
#define _SECOND_ROW 0xC0 /* Second line address */
#define _THIRD_ROW  0x94 /* Third line address */
#define _FOURTH_ROW 0xD4 /* Fourth line address */

/* ------------- LCD Config ------------- */

#define _8BIT_INTERFACE 0x3 /* 8Bit interface selector */
#define _4BIT_INTERFACE 0x2 /* 4Bit interface selector */
#define _1LINE_DISPLAY  0 /* number of lines */
#define _2LINE_DISPLAY  1 /* number of lines */
#define _FONT_5X10      1 /* character font */
#define _FONT_5X7       0 /* character font */

/* -------------------------------------- */

#define _R0   0 /* Row 0 selector */
#define _R1   1 /* Row 1 selector */
#define _R2   2 /* Row 2 selector */
#define _R3   3 /* Row 3 selector */
#define _NBIT 3 /* Select number of line bit in register */
#define _FBIT 2 /* Select font bit in register */
#define _WAIT_FOR_POWER_ON 15 /* Delay time for power on */
#define _WAIT_FOR_SET_MODE 5

/* --------------- Public --------------- */

#define _NIBBLE 0xF  /* Nibble value */
#define _BYTE   0xFF /* Byte value */
#define _HIGH_BYTE 4 /* Select high byte */

/*----------------------------------------------------------*/

#ifdef __CODEVISIONAVR__  /* Check compiler */

#define _Delay_Ms(t)                                    delay_ms(t) /* Select delay function */
#define _GPIO_WritePin(gpiox , gpio_pin , pin_state)    GPIO_WritePin(gpiox , gpio_pin , pin_state) /* Select GPIO function */

/*----------------------------------------------------------*/

#elif defined(__GNUC__)   /* Check compiler */

#define _Delay_Ms(t)                                    _delay_ms(t) /* Select delay function */
#define _GPIO_WritePin(gpiox , gpio_pin , pin_state)    GPIO_WritePin(gpiox , gpio_pin , pin_state) /* Select GPIO function */

/*----------------------------------------------------------*/

#elif defined(USE_HAL_DRIVER)   /* Check compiler */

#define _Delay_Ms(t)                                    HAL_Delay(t) /* Select delay function */
#define _GPIO_WritePin(gpiox , gpio_pin , pin_state)    HAL_GPIO_WritePin(gpiox , gpio_pin , pin_state) /* Select GPIO function */
#define _GPIO_PIN_RESET                                 GPIO_PIN_RESET /* Select GPIO reset instruction */
#define _GPIO_PIN_SET                                   GPIO_PIN_SET /* Select GPIO set instruction */

#endif /* __CODEVISIONAVR__ */

/*----------------------------------------------------------*/

/* -------------------------------------- Struct -------------------------------------- */

typedef struct /* type Structure for config LCD */
{
	
	uint8_t Mode; /* Variable for set LCD mode */
	uint8_t Font; /* Variable for set LCD font */
	uint8_t NumberOfLine; /* Variable for set LCD number of line */
	
}LCD_Init_t;

extern LCD_Init_t Lcd_Config; /* Structure for config LCD */

/* ----------------------------------- Proto types ------------------------------------ */

void Lcd_SendCommand(uint8_t cmd); /* Function for send command */

void Lcd_Init(void); /* Function for initialize LCD */

void Lcd_PutChar(int8_t data); /* Function for send character to LCD */

void Lcd_PutString(int8_t *str); /* Function for send string to LCD */

void Lcd_Clear(void); /* Function for clear LCD */

void Lcd_GotoXY(uint8_t column , uint8_t row); /* Function for set x & y address */

void Lcd_ShiftToLeft(uint8_t shift_number , uint16_t shift_time); /* Function for shift LCD */

void Lcd_ShiftToRight(uint8_t shift_number , uint16_t shift_time); /* Function for shift LCD */

/* ------------------------------------------------------------------------------------ */

#endif /* __CHARACTER_LCD_H_ */

/* Program End */