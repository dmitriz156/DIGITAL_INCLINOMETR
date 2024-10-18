/*
 * DI-601_MPU-6050.c
 *
 * Created: 15.04.2024 15:30:55
 */ 
#include "HW_Init.h"
#include "mpu6050.h"
#include "intfx.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define UART_BUFFER_SIZE		128
/*===============Constants that determine the types of protractors=================*/
#define MODULAR_RB_DATA_TYPE		0xAA
#define RB_DATA_TYPE				0xFA
#define ADDRESS_CHANGE_KEY			111
#define DATA_PACKAGE_SIZE			7
/*=================================================================================*/
#define ANGLE_SENS_ADDRESS_1		1
#define ANGLE_SENS_ADDRESS_2		2
#define ANGLE_SENS_ADDRESS_3		3
#define ANGLE_SENS_ADDRESS_4		4
/*=================================================================================*/
#define MODUL_ADDRESS_1				11
#define MODUL_ADDRESS_2				12
#define MODUL_ADDRESS_3				13
#define MODUL_ADDRESS_4				14
/*=================================================================================*/

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
#define LED_OUT										((volatile IO_PORT*)_SFR_MEM_ADDR(DDRD))->Bit3
#define LED										    ((volatile IO_PORT*)_SFR_MEM_ADDR(PORTD))->Bit3
#define PB5_OUT										((volatile IO_PORT*)_SFR_MEM_ADDR(DDRB))->Bit5 //SCK
#define PB5										    ((volatile IO_PORT*)_SFR_MEM_ADDR(PORTB))->Bit5
#define RX_TX_DIRECTION								((volatile IO_PORT*)_SFR_MEM_ADDR(DDRD))->Bit2
// Вибір напрямку передачі RS 485
#define RX_TX										((volatile IO_PORT*)_SFR_MEM_ADDR(PORTD))->Bit2
#define USART_TX_9BIT    							((volatile IO_PORT*)_SFR_MEM_ADDR(UCSRB))->Bit0
#define USART_RX_9BIT    							((volatile IO_PORT*)_SFR_MEM_ADDR(UCSRB))->Bit1

// Constants
#define TAU 0.98  // Комплементарний коефіцієнт
#define DT 0.1     // Час між оновленнями (100 Hz)
#define GYRO_SCALE	(2000/16384)*(M_PI/180)

#define FILTER_WINDOW_SIZE 5 // Розмір вікна фільтра

int16_t filter_window_X[FILTER_WINDOW_SIZE] = {0}; // Вікно фільтра для X_axel
float filter_window_Y[FILTER_WINDOW_SIZE] = {0}; // Вікно фільтра для Y_axel
int16_t filter_window_Z[FILTER_WINDOW_SIZE] = {0}; // Вікно фільтра для Z_axel
int filter_index = 0; // Індекс вікна фільтра
//===================================================================================================
uint8_t start_token = 0xAB;
uint16_t count = 0;
uint8_t send_id = 0;
uint16_t ubrr_value = 47;
//===================================================================================================
float angleX, angleY, angleZ;  // Фільтровані кути
float prevAngleX, prevAngleY, prevAngleZ;  // Попередні кути для фільтра
/* ---------------------------------------- */
uint8_t tx_wr_index,tx_rd_index,tx_counter;
/* ---------------------------------------- */
bool IsCalibrate = 0;
bool Togl_UART;
uint8_t com_resp = 0;
char str[_STR_LENGHT_FOR_LCD]; /* string for convert */
float x_gyro, x_axel;
float y_gyro, y_axel;
float z_gyro, z_axel;
float x_gyro_OLD, x_axel_OLD;
float y_gyro_OLD, y_axel_OLD = 1;
float z_gyro_OLD, z_axel_OLD;
int16_t Xaxel_int, Xgyro_int, offsetGX_int;
int16_t Yaxel_int, Ygyro_int, offsetGY_int;
int16_t Zaxel_int, Zgyro_int, offsetGZ_int;
volatile float X_angle, Y_angle, Z_angle;

volatile float filtered_X;
volatile float filtered_Y;
volatile float filtered_Z;
volatile float OUT_angle ;

int16_t temp;
/* ---------------------------------------- */

/*====================================================================================*/
#define BIT_0          		((volatile IO_PORT*)_SFR_MEM_ADDR(PORTD))->Bit0//PORTD.0
#define BIT_1          		((volatile IO_PORT*)_SFR_MEM_ADDR(PORTD))->Bit2//PORTD.2
#define BIT_2          		((volatile IO_PORT*)_SFR_MEM_ADDR(PORTD))->Bit4//PORTD.4
#define IN_P          		((volatile PINState*)_SFR_MEM_ADDR(PINB))->Bit0//PINB.0
#define IN_G          		((volatile PINState*)_SFR_MEM_ADDR(PIND))->Bit7//PIND.7
#define TX_ON               ((volatile IO_PORT*)_SFR_MEM_ADDR(PORTD))->Bit2//PORTD.2
//Для відладки
#define DIOD                ((volatile IO_PORT*)_SFR_MEM_ADDR(PORTB))->Bit0//PORTB.0
#define LED                 ((volatile IO_PORT*)_SFR_MEM_ADDR(PORTD))->Bit3//PORTD.3

#define MPU6050_R			0b11010001
#define MPU6050_W			0b11010000
//Фактичне значення кута
union {unsigned char angle_c[2]; int angle_i;} u;

//Буфеп передачі і прийому байт
volatile uint8_t Rx_buf[DATA_PACKAGE_SIZE], Tx_buf[4];

//Прапорець першого біта адреси
bool first_bit;

uint32_t Led_counter=0;
uint8_t bufer;
uint8_t Temp;

/*====================================================================================*/
#define FRAMING_ERROR (1<<FE)
#define PARITY_ERROR (1<<PE)
#define DATA_OVERRUN (1<<DOR)
#define DATA_REGISTER_EMPTY (1<<UDRE)
#define RX_COMPLETE (1<<RXC)

volatile uint8_t rx_buffer[DATA_PACKAGE_SIZE];
volatile uint8_t tx_buffer[DATA_PACKAGE_SIZE];
//volatile uint8_t address_change_Buffer[DATA_PACKAGE_SIZE];

volatile uint8_t Address_Change_Counter = 0;
volatile bool Address_Change_FLAG = 0;
volatile bool Enable_Flag = 0;
volatile bool Data_Type_FLAG = 0;
volatile uint8_t Sensor_Addres = ANGLE_SENS_ADDRESS_1;
bool AvrgFLAG = 0;
uint8_t rx_counter, rx_counter;
// This flag is set on USART Receiver buffer overflow
bool rx_buffer_overflow;

uint8_t crc( uint8_t *code, uint8_t size);
//----------------------------------------------------------------------------------
// USART Receiver interrupt service routine
ISR(USART_RXC_vect);

unsigned char tx_wr_index, tx_rd_index, tx_counter;

// USART Transmitter interrupt service routine
ISR(USART_TXC_vect);

// Write a character to the USART Transmitter buffer
void Putchar(char c);
void UART_Init();
void UART_Transmit(char data);
void UART_Transmit_String(const char *str);
void UART_Print(const char *format, ...);
void UART_PrintLn(const char *format, ...);
void UART_SendUint16(uint16_t value);
/*================================================================================*/
int16_t MPU6050_Calibrate(int16_t *Xg, int16_t *Yg, int16_t *Zg);
/*================================================================================*/
int16_t median_filter(int16_t value, int16_t *filter_window);
#define KOEF			0.98

#define KALMAN_ACEL_Q	0.01//0.01
#define KALMAN_GYRO_Q	0.05//0.03
#define KALMAN_R		0.4//0.015
float angle = 0;
float bias = 0;
float P[2][2] = {{0, 0}, {0, 0}};
	
float kalman_filter(float measured_angle, float gyro_rate, float dt) {
// 	static float angle = 0;
// 	static float bias = 0;
// 	static float P[2][2] = {{0, 0}, {0, 0}};

	// Перший крок: оцінка нового стану на основі прогнозу
	angle += dt * (gyro_rate - bias);

	// Другий крок: оновлення коваріаційної матриці на основі прогнозу
	P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + KALMAN_ACEL_Q);
	P[0][1] -= dt * P[1][1];
	P[1][0] -= dt * P[1][1];
	P[1][1] += KALMAN_GYRO_Q * dt;

	// Третій крок: вирахування калманівського коефіцієнту
	float S = P[0][0] + KALMAN_R;
	float K[2]; // калманівський коефіцієнт
	K[0] = P[0][0] / S;
	K[1] = P[1][0] / S;

	// Четвертий крок: оновлення оцінки на основі вимірів
	float y = measured_angle - angle; // помилка виміру
	angle += K[0] * y;
	bias += K[1] * y;

	// П'ятий крок: оновлення коваріаційної матриці на основі вимірів
	float P00_temp = P[0][0];
	float P01_temp = P[0][1];

	P[0][0] -= K[0] * P00_temp;
	P[0][1] -= K[0] * P01_temp;
	P[1][0] -= K[1] * P00_temp;
	P[1][1] -= K[1] * P01_temp;

	return angle;
}

float varVolt = 0.3; //середнє значення видхилення
float varProcess = 0.2; //швидкість реакції на виміри 
float Pc = 0.0;
float G = 0.0;
float Pp = 1.0;
float Xp, Zp, Xe = 0.0;

float SimpleKalmanFilter(float angle)
{
	Pc = Pp + varProcess;
	G = Pc / (Pc + varVolt);
	Pp = (1 - G) * Pc;
	Xp = Xe;
	Zp = Xp;
	Xe = G * (angle - Zp) + Xp;
	return Xe;
}

// Функція для розбиття значення типу float на масив байтів uint8_t
void float_to_byte(float value, uint8_t *array, int start_index) {
	uint8_t *byte_ptr = (uint8_t *)&value; // Вказівник на перший байт float-значення

	for (int i = 0; i < sizeof(float); i++) {
		array[start_index + i] = *(byte_ptr + i); // Запис кожного байта в масив
	}
}

// Функція для конвертації масиву байтів uint8_t в значення типу float
float byte_to_float(uint8_t *array, int start_index) {
	float value;
	uint8_t *byte_ptr = (uint8_t *)&value; // Вказівник на перший байт float-значення

	for (int i = 0; i < sizeof(float); i++) {
		*(byte_ptr + i) = array[start_index + i]; // Зчитування кожного байта з масиву
	}

	return value;
}

int main(void)
{
	sei();
	wdt_enable(WDTO_1S);
	_LCD_PORT_DIR = _ALL_OUTPUT;
	_I2C_PORT_DIR = ( 1 << _SDA ) | _SCL;
	
	/* MPU6050 Init */
	MPU6050_AutoInit(100);
	UART_Init();
	RX_TX_DIRECTION = 1;
	//RX_TX = 1;
	LED_OUT = 1;
	_delay_ms(100);
	//MPU6050_Calibrate(offsetGX, offsetGY, offsetGZ);
	/* ---------------------------------------- */
    /* Replace with your application code */
    while (1) 
    {
		//UART_Init();
		MPU6050_GetRawAccelX(&Xaxel_int, 20);
		MPU6050_GetRawAccelY(&Yaxel_int, 20);
		MPU6050_GetRawAccelZ(&Zaxel_int, 20);
		MPU6050_GetRawGyroX(&Xgyro_int, 20);
		MPU6050_GetRawGyroY(&Ygyro_int, 20);
		MPU6050_GetRawGyroZ(&Zgyro_int, 20);
		
		//LED = ~LED;
		wdt_reset();
// 		MPU6050_GetAccelAngleX(&x_axel, 100);
// 		MPU6050_GetGyroX(&x_gyro, 100);

		x_axel = ( _MPU_RAD_TO_DEG * ( atan2( -Yaxel_int , -Zaxel_int ) + _MATH_PI ) );
		y_axel = ( _MPU_RAD_TO_DEG * ( atan2( -Xaxel_int , -Zaxel_int ) + _MATH_PI ) );
		z_axel = ( _MPU_RAD_TO_DEG * ( atan2( -Yaxel_int , -Xaxel_int ) + _MATH_PI ) );

		x_gyro = ((Xgyro_int /*- offsetGX*/) / _MPU_GYRO_SENS_2000_SENS /*/ (1000 / 120)*/);
		y_gyro = ((Ygyro_int /*- offsetGY*/) / _MPU_GYRO_SENS_2000_SENS /*/ (1000 / 120)*/);
		z_gyro = ((Zgyro_int /*- offsetGZ*/) / _MPU_GYRO_SENS_2000_SENS /*/ (1000 / 120)*/);

// 		//===============Complementary filter begin================
// 		filtered_Y = (int16_t)((0.98 * y_axel) - (0.02 * y_gyro));
//		Y_angle = KOEF * (y_axel * 0.11) + (1 - KOEF) * y_gyro;
// 		//================Complementary filter end================
// 		//==================Simple KALMAN filter==================
// 		filtered_Y = KOEF * y_axel + (1 - KOEF) * y_axel_OLD;
// 		y_axel_OLD = y_axel;
// 		//========================================================
		//X_angle = kalman_filter(x_axel, x_gyro, 110);
		Y_angle = kalman_filter(y_axel, y_gyro, 110);
		//Z_angle = kalman_filter(z_axel, z_gyro, 110); //t prev = 120
		
	//====================================Output depending on the type BEGIN======================================
		if (Data_Type_FLAG == 1){Y_angle -= 90.0;}
		if (Y_angle <= 180.0){
			Y_angle = Y_angle;
		}else{
			Y_angle = Y_angle - 360.0;
		}
		//OUT_angle = SimpleKalmanFilter(Y_angle);
	//=====================================Output depending on the type END=======================================
	//====!!!!!!!!!!!====cod to display in arduino ide monitor=====!!!!!!!!!=======
		//float_to_byte(Y_angle, Tx_buf, 0);
		//Y_angle = median_filter(Y_angle, filter_window_Y);
		//filtered_Y = (int16_t)(Y_angle * 10.0);
		//_delay_ms(10);
		//fY = median_filter(y_axel, filter_window_Y);
		//UART_Transmit_String("X:,Y:,Z:,X_f:,Y_f:,Z_f:\r\n");
// 		UART_Transmit_String("X:,Y:,Z:\r\n");
// 		UART_PrintLn("%d", (int16_t)Y_angle);

		//UART_PrintLn("%d,%d,%d,%d,%d,%d", Xaxel, Yaxel, Zaxel, Xgyro, Xgyro, Xgyro);
		//UART_PrintLn("%d,%d,%d", filtered_X, filtered_Y, filtered_Z); //%.2f	
	
		//_delay_ms(10);
    }
}

//----------------------------------------------------------------------------------
//функкція обчислення СРС
uint8_t crc( uint8_t *code, uint8_t size)
{
	uint8_t j, i, Data, tmp, CRC = 0;
	for (j = 0; j < size; j++)
	{
		Data = code[j];
		for (i = 0; i < 8; i++)
		{
			tmp = 1 & (Data ^ CRC);
			CRC >>= 1;
			Data >>= 1;
			if ( 0 != tmp ) CRC ^= 0x8c;
		}
	}
	return CRC;
}
//----------------------------------------------------------------------------------

ISR(USART_RXC_vect)
{
	//first_bit = (UCSRB & (1 << RXB8));
	
	//Якщо отримали ознаку першого байту, - обнулити лічильник буферу
	if (USART_RX_9BIT == 1)
	{
		rx_counter = 0;
		//first_bit = 0;
	}
	rx_buffer[rx_counter] = UDR;
	if (rx_counter < DATA_PACKAGE_SIZE-1){ rx_counter ++;}
	//if (rx_counter == DATA_PACKAGE_SIZE)
	else
	{
		LED = ~LED;
		Enable_Flag = 1;
		//Якщо прийнятий стартовий байт
		//якщо адреса співпадає
		//Або адреса дорівнює зарезервованій адресі
		//І співпадає CRC
		if ( (rx_buffer[0] == Sensor_Addres) && (Enable_Flag == 1) && (crc(rx_buffer, DATA_PACKAGE_SIZE-1) == rx_buffer[6]))
		{
			if (rx_buffer[2] == RB_DATA_TYPE){Data_Type_FLAG = 1;}
			if (rx_buffer[2] == MODULAR_RB_DATA_TYPE){Data_Type_FLAG = 0;}
			if (rx_buffer[3] == ADDRESS_CHANGE_KEY)
			{
				if (Address_Change_Counter < 10)
				{
					Address_Change_Counter ++;
				}else{
					Sensor_Addres = rx_buffer[4];
					Address_Change_Counter = 0;
				}
			}else{
				Address_Change_Counter = 0;
			}
			for (uint8_t i = 0; i < DATA_PACKAGE_SIZE; i++){
				rx_buffer[i] = 0;
			}
			TX_ON = 1;
			memset(tx_buffer, 0, DATA_PACKAGE_SIZE);
			tx_buffer[0] = Sensor_Addres;
			float_to_byte(Y_angle, tx_buffer, 1);
			tx_buffer[5] = Sensor_Addres;	
			tx_buffer[6] = crc(tx_buffer, DATA_PACKAGE_SIZE-1);           //обрахувати контрольну суму і вислати останнім байтом
			_delay_ms(1);

			Enable_Flag = 0;
			
			UCSRB |= (1 << TXB8);
			tx_counter = (DATA_PACKAGE_SIZE-1);
			//Передавання пакету данних
			UDR = tx_buffer[0];

		}
		//else{TX_ON = 0;}
	}
}


// USART Transmitter interrupt service routine
ISR(USART_TXC_vect)
{
	if (tx_counter > 0)
	{
		tx_counter--;
		UCSRB &= ~(1 << TXB8);       //Скинули 9-й біт
		
		UDR = tx_buffer[(DATA_PACKAGE_SIZE-1) - tx_counter];
	}
	else
	{
		TX_ON = 0;
	}
}

// Write a character to the USART Transmitter buffer

/* ======================================================================================================*/

void UART_Init()
{
	/*UCSRA=0x00;
	UCSRB=0xDC;
	UCSRC=0x86;
	UBRRH=0x00;
	UBRRL=0x2F;*/

	UBRRH = (unsigned char)(ubrr_value >> 8);
	UBRRL = (unsigned char)ubrr_value;
	//Увімкнення переривань на передачу та прийом Увімкнення передавача та приймача
	UCSRB |= (1 << RXCIE) | (1 << TXCIE) | (1 << TXEN) | (1 << RXEN) | (1 << UCSZ2);
	// Встановлення формату кадру: 9 бітів даних, 1 стоп біт, без парності
	UCSRC |= (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);	
}

void UART_Transmit(char data) {
	// Чекаємо на вільний буфер передачі
	while (!(UCSRA & (1 << UDRE)));
	// Передаємо дані в регістр даних
	UDR = data;
}

void UART_Transmit_String(const char *str) {
	while (*str) {
		UART_Transmit(*str++);
	}
}

void UART_Print(const char *format, ...) {
	char buffer[UART_BUFFER_SIZE];
	va_list args;
	va_start(args, format);
	vsnprintf(buffer, UART_BUFFER_SIZE, format, args);
	va_end(args);
	UART_Transmit_String(buffer);
}

void UART_PrintLn(const char *format, ...) {
	char buffer[UART_BUFFER_SIZE];
	va_list args;
	va_start(args, format);
	vsnprintf(buffer, UART_BUFFER_SIZE, format, args);
	va_end(args);
	strcat(buffer, "\r\n");
	UART_Transmit_String(buffer);
}

void UART_SendUint16(uint16_t value) {
	UART_Transmit(value & 0xFF);
	UART_Transmit((value >> 8) & 0xFF);
}
/*================================================================================*/
int16_t MPU6050_Calibrate(int16_t *Xg, int16_t *Yg, int16_t *Zg){
	int16_t Bef_Cal_x, Bef_Cal_y, Bef_Cal_z;
	int16_t Cal_x, Cal_y, Cal_z; 
	uint16_t iNumCM = 200;
	for (int i = 0; i < iNumCM; i ++)
	{
		MPU6050_GetRawGyroX(&Bef_Cal_x, 20);
		MPU6050_GetRawGyroY(&Bef_Cal_y, 20);
		MPU6050_GetRawGyroZ(&Bef_Cal_z, 20);
		Cal_x += Bef_Cal_x;
		Cal_y += Bef_Cal_y;
		Cal_z += Bef_Cal_z;
	}
	Xg = Cal_x / iNumCM;
	Yg = Cal_y / iNumCM;
	Zg = Cal_z / iNumCM;
}
/*================================================================================*/
//==================================================================================
int16_t median_filter(int16_t value, int16_t *filter_window) {
	filter_window[filter_index] = value; // Додаємо нове значення до вікна фільтра
	filter_index = (filter_index + 1) % FILTER_WINDOW_SIZE; // Перевизначаємо індекс для кругового буфера
	
	// Сортуємо значення в вікні фільтра
	for (int i = 0; i < FILTER_WINDOW_SIZE - 1; i++) {
		for (int j = 0; j < FILTER_WINDOW_SIZE - i - 1; j++) {
			if (filter_window[j] > filter_window[j + 1]) {
				// Обмін значень
				int16_t temp = filter_window[j];
				filter_window[j] = filter_window[j + 1];
				filter_window[j + 1] = temp;
			}
		}
	}
	
	// Повертаємо медіанне значення з вікна фільтра
	return filter_window[FILTER_WINDOW_SIZE / 2];
}
//==================================================================================
