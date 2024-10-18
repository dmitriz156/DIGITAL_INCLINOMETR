/*
------------------------------------------------------------------------------
 * File   : CHARACTER_LCD_CONFIG.h
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

#ifndef __CHARACTER_LCD_CONFIG_H_
#define __CHARACTER_LCD_CONFIG_H_

/* -------------------------------------- Define -------------------------------------- */

#define _LCD_CTRL_PORT &PORTC /* Select Control port */
#define _LCD_DATA_PORT &PORTC /* Select DATA port */

#define _LCD_D0_PIN 3 /* Select D0 pin in 8Bit mode Or D4 pin in 4Bit mode */
#define _LCD_RS_PIN 0 /* Select RS pin */
#define _LCD_RW_PIN 1 /* Select RW pin */
#define _LCD_EN_PIN 2 /* Select EN pin */

/* Example:

 AVR:
     #define _LCD_CTRL_PORT &PORTB
     #define _LCD_DATA_PORT &PORTA

     #define _LCD_D0_PIN 0
     #define _LCD_RS_PIN 5
     #define _LCD_RW_PIN 6
     #define _LCD_EN_PIN 7

 ARM:
     #define _LCD_CTRL_PORT GPIOB
     #define _LCD_DATA_PORT GPIOA

     #define _LCD_D0 0
     #define _LCD_RS 5
     #define _LCD_RW 6
     #define _LCD_EN 7

*/

/* ------------------------------------------------------------------------------------ */

#endif /* __CHARACTER_LCD_CONFIG_H_ */