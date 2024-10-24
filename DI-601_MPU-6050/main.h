
#ifndef MAIN_H_
#define MAIN_H_
#include <stdio.h>
#include <stdbool.h>

typedef struct
{
	unsigned char Bit0:1;
	unsigned char Bit1:1;
	unsigned char Bit2:1;
	unsigned char Bit3:1;
	unsigned char Bit4:1;
	unsigned char Bit5:1;
	unsigned char Bit6:1;
	unsigned char Bit7:1;
}IO_PORT,PINState,EECR_Reg,EEARL_Reg,UCSR0A_Reg,UCSRB_Reg,GICR_Reg;

#define F_CPU 14745600UL
#define BAUD 19200
#define UBRR_SPEED F_CPU/16/BAUD-1
//47
#define LED_OUT										((volatile IO_PORT*)_SFR_MEM_ADDR(DDRD))->Bit3
#define LED										    ((volatile IO_PORT*)_SFR_MEM_ADDR(PORTD))->Bit3
#define PB5_OUT										((volatile IO_PORT*)_SFR_MEM_ADDR(DDRB))->Bit5 //SCK
#define PB5										    ((volatile IO_PORT*)_SFR_MEM_ADDR(PORTB))->Bit5
#define RX_TX_DIRECTION								((volatile IO_PORT*)_SFR_MEM_ADDR(DDRD))->Bit2
// Вибір напрямку передачі RS 485
#define RX_TX										((volatile IO_PORT*)_SFR_MEM_ADDR(PORTD))->Bit2
#define USART_TX_9BIT    							((volatile IO_PORT*)_SFR_MEM_ADDR(UCSRB))->Bit0
#define USART_RX_9BIT    							((volatile IO_PORT*)_SFR_MEM_ADDR(UCSRB))->Bit1

#define FRAMING_ERROR        (1<<FE)
#define PARITY_ERROR         (1<<PE)
#define DATA_OVERRUN         (1<<DOR)
#define DATA_REGISTER_EMPTY  (1<<UDRE)
#define RX_COMPLETE          (1<<RXC)

/*====================================================================================*/
#define BIT_0          		((volatile IO_PORT*)_SFR_MEM_ADDR(PORTD))->Bit0//PORTD.0
#define BIT_1          		((volatile IO_PORT*)_SFR_MEM_ADDR(PORTD))->Bit2//PORTD.2
#define BIT_2          		((volatile IO_PORT*)_SFR_MEM_ADDR(PORTD))->Bit4//PORTD.4
#define IN_P          		((volatile PINState*)_SFR_MEM_ADDR(PINB))->Bit0//PINB.0
#define IN_G          		((volatile PINState*)_SFR_MEM_ADDR(PIND))->Bit7//PIND.7
#define TX_ON               ((volatile IO_PORT*)_SFR_MEM_ADDR(PORTD))->Bit2 = 1//PORTD.2
#define RX_ON				((volatile IO_PORT*)_SFR_MEM_ADDR(PORTD))->Bit2 = 0//PORTD.2
//Для відладки
#define DIOD                ((volatile IO_PORT*)_SFR_MEM_ADDR(PORTB))->Bit0//PORTB.0
#define LED                 ((volatile IO_PORT*)_SFR_MEM_ADDR(PORTD))->Bit3//PORTD.3

typedef enum {
	MODULAR_RB_DATA_TYPE = 0xAA,
	RB_DATA_TYPE         = 0xFA,
	BOLLARD_DATA_TYPE    = 0xBD
} data_type_t;

typedef struct {
	int16_t Xaccel_raw;
	int16_t Yaccel_raw;
	int16_t Zaccel_raw;
	
	int16_t Xgyro_raw;
	int16_t Ygyro_raw;
	int16_t Zgyro_raw;
	
	int16_t offsetGX;
	int16_t offsetGY;
	int16_t offsetGZ;
	
} raw_axis_data_t;

typedef struct {
	float x_gyro, x_accel;
	float y_gyro, y_accel;
	float z_gyro, z_accel;	
} axis_data_t;

uint8_t crc( uint8_t *code, uint8_t size);

ISR(TIMER2_COMP_vect);
ISR(USART_RXC_vect);
ISR(USART_TXC_vect);

// Write a character to the USART Transmitter buffer
void Putchar(char c);
void Timer_2_Init(void);
void UART_Init(uint16_t ubrr_value);
void UART_data_procesing(void);
void UART_Transmit(char data);
void UART_Transmit_String(const char *str);
void UART_Print(const char *format, ...);
void UART_PrintLn(const char *format, ...);
void UART_SendUint16(uint16_t value);
/*================================================================================*/
int16_t MPU6050_Calibrate(int16_t *Xg, int16_t *Yg, int16_t *Zg);
/*================================================================================*/
float kalman_filter(float measured_angle, float gyro_rate, float dt);//dt -> sec
int16_t median_filter(int16_t value, int16_t *filter_window);

#endif /* MAIN_H_ */