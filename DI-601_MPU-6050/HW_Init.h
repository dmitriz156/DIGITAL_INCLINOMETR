/*
 * HW_Init.h
 *
 * Created: 2/15/2019 9:46:16 AM
 *  Author: Majid Derhambakhsh
 */ 


#ifndef __HW_INIT_H_
#define __HW_INIT_H_

#ifndef F_CPU

//#define F_CPU 8000000UL         /* Set clock frequency to 1MHz */
#define F_CPU 14745600UL

#endif

/*********************** Includes ************************/

#include <avr/io.h> /* Import IO Lib */
#include <stdio.h> /* Import Standard IO Lib */
#include "mpu6050.h" /* Import MPU60X0 Lib */
#include "CHARACTER_LCD.h" /* Import LCD Lib */

/************************ Defines ************************/

/* ------------------- LCD ------------------- */

#define _LCD_ROW_1    1 /* LCD Row 1 */
#define _LCD_COLUMN_5 5 /* LCD Column 1 */
#define _LCD_PORT_DIR DDRC     /* LCD PORT */
#define _LCD_PORT_STATE PORTC  /* LCD PORT */
#define _STR_LENGHT_FOR_LCD 17 /* LCD Lenght + 1 */

/* ------------------- I2C ------------------- */

#define _SDA 1 
#define _SCL 1 
#define _I2C_PORT_DIR DDRC    /* I2C PORT */
#define _I2C_PORT_STATE PORTC /* I2C PORT */

/* ------------------ Public ----------------- */

#define _ALL_OUTPUT 0xFF
#define _WAIT_FOR_NEXT_SHOW 1500

/*********************** Prototype ***********************/

void HW_Init(void); /* Function for initialize hardware */

#endif /* __HW_INIT_H_ */
