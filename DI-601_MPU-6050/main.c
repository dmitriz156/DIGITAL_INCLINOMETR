/*
 * DI-601_MPU-6050.c
 *
 * Created: 15.04.2024 15:30:55
 */ 
#include "main.h"
#include "HW_Init.h"
#include "mpu6050.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

//#define DEBUG_MOD
#define UART_BUFFER_SIZE		128
/*===============Constants that determine the types of protractors=================*/
// #define MODULAR_RB_DATA_TYPE		0xAA
// #define RB_DATA_TYPE				0xFA
// #define BOLLARD_DATA_TYPE		0xBD
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



// Constants
#define TAU 0.98  // Комплементарний коефіцієнт
#define DT 0.1     // Час між оновленнями (100 Hz)
#define GYRO_SCALE	(2000/16384)*(M_PI/180)

#define FILTER_WINDOW_SIZE 5 // Розмір вікна фільтра

int16_t filter_window_X[FILTER_WINDOW_SIZE] = {0}; // Вікно фільтра для X_accel
float filter_window_Y[FILTER_WINDOW_SIZE] = {0}; // Вікно фільтра для Y_accel
int16_t filter_window_Z[FILTER_WINDOW_SIZE] = {0}; // Вікно фільтра для Z_accel
int filter_index = 0; // Індекс вікна фільтра
//===================================================================================================
uint8_t start_token = 0xAB;
uint16_t count = 0;
uint8_t send_id = 0;
//===================================================================================================
/* ---------------------------------------- */
uint8_t tx_wr_index, tx_rd_index, tx_counter;
/* ---------------------------------------- */
bool IsCalibrate = 0;
bool Togl_UART;
uint8_t com_resp = 0;
char str[_STR_LENGHT_FOR_LCD]; /* string for convert */

/* ---------------------------------------- */
raw_axis_data_t All_Axis_ROW = {0};
axis_data_t		All_Axis     = {0};

volatile float X_angle = 0;
volatile float Y_angle = 0;
volatile float Z_angle = 0;

uint8_t UART_NeedToSend = 0;
uint8_t sensitivity = 0;
int16_t MAX_Accel_Value_X = 0;
int16_t MAX_Accel_Value_Y = 0;
int16_t MAX_Accel_Value_Z = 0;

volatile float filtered_X;
volatile float filtered_Y;
volatile float filtered_Z;
volatile float OUT_angle;

int16_t temp;
/* ---------------------------------------- */

#define MPU6050_R			0b11010001
#define MPU6050_W			0b11010000

//Буфеп передачі і прийому байт
volatile uint8_t Rx_buf[DATA_PACKAGE_SIZE];
volatile uint8_t Tx_buf[4];

//Прапорець першого біта адреси
bool first_bit;

uint32_t Led_counter=0;
uint8_t bufer;
uint8_t Temp;

volatile uint8_t rx_buffer[DATA_PACKAGE_SIZE];
volatile uint8_t tx_buffer[DATA_PACKAGE_SIZE];
//volatile uint8_t address_change_Buffer[DATA_PACKAGE_SIZE];

volatile uint8_t Address_Change_Counter = 0;
volatile bool Address_Change_FLAG = 0;
volatile bool UART_RX_Complete_FLAG = 0;
volatile bool UART_TX_Complete_FLAG = 0;
volatile uint8_t UART_TX_Pre_Counter = 0;
volatile data_type_t Angle_Type = 0;
volatile uint8_t Sensor_Addres = ANGLE_SENS_ADDRESS_2;
bool AvrgFLAG = 0;
uint8_t rx_counter, rx_counter;
// This flag is set on USART Receiver buffer overflow
bool rx_buffer_overflow;

/*------kalman_filter coeficient------*/
#define KOEF			0.98
#define KALMAN_ACEL_Q	0.01//0.01
#define KALMAN_GYRO_Q	0.05//0.03
#define KALMAN_R		0.1//0.015

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
	//wdt_enable(WDTO_1S);
	_LCD_PORT_DIR = _ALL_OUTPUT;
	_I2C_PORT_DIR = ( 1 << _SDA ) | _SCL;
	
	/* MPU6050 Init */
	MPU6050_AutoInit(100);
	Timer_2_Init();
	UART_Init(UBRR_SPEED);
	RX_TX_DIRECTION = 1;
	LED_OUT = 1;
	_delay_ms(100);
	//MPU6050_Calibrate(offsetGX, offsetGY, offsetGZ);
	/* ---------------------------------------- */
    while (1) 
    {
// 		MPU6050_GetGyro(Gyro_Data_arr, 100);
// 		MPU6050_GetAccel(Accel_Data_arr, 100);

// 		MPU6050_GetRawAccelX(&All_Axis_ROW.Xaccel_raw, 100);...Y,Z
// 		MPU6050_GetRawGyroX(&All_Axis_ROW.Xgyro_raw, 100);...

		
		MPU6050_GetRawAccel(&All_Axis_ROW.Xaccel_raw, 20);
		MPU6050_GetRawGyro(&All_Axis_ROW.Xgyro_raw, 20);
		//wdt_reset();
// 		MPU6050_GetAccelAngleX(&x_accel, 100);
// 		MPU6050_GetGyroX(&x_gyro, 100);
		All_Axis.x_accel = ( _MPU_RAD_TO_DEG * ( atan2( -All_Axis_ROW.Yaccel_raw , -All_Axis_ROW.Zaccel_raw ) + _MATH_PI ) );
		All_Axis.y_accel = ( _MPU_RAD_TO_DEG * ( atan2( -All_Axis_ROW.Xaccel_raw , -All_Axis_ROW.Zaccel_raw ) + _MATH_PI ) );
		All_Axis.z_accel = ( _MPU_RAD_TO_DEG * ( atan2( -All_Axis_ROW.Yaccel_raw , -All_Axis_ROW.Xaccel_raw ) + _MATH_PI ) );
		All_Axis.x_gyro = ((All_Axis_ROW.Xgyro_raw /*- offsetGX*/) / _MPU_GYRO_SENS_250_SENS /*/ (1000 / 120)*/);
		All_Axis.y_gyro = ((All_Axis_ROW.Ygyro_raw /*- offsetGY*/) / _MPU_GYRO_SENS_250_SENS /*/ (1000 / 120)*/);
		All_Axis.z_gyro = ((All_Axis_ROW.Zgyro_raw /*- offsetGZ*/) / _MPU_GYRO_SENS_250_SENS /*/ (1000 / 120)*/);

// 		//===============Complementary filter begin================
// 		filtered_Y = (int16_t)((0.98 * y_accel) - (0.02 * y_gyro));
//		Y_angle = KOEF * (y_accel * 0.11) + (1 - KOEF) * y_gyro;
// 		//================Complementary filter end================

// 		//==================Simple KALMAN filter==================
// 		filtered_Y = KOEF * y_accel + (1 - KOEF) * y_accel_OLD;
// 		y_accel_OLD = y_accel;
// 		//========================================================
		//X_angle = kalman_filter(x_accel, x_gyro, 110);
		Y_angle = kalman_filter(All_Axis.y_accel, All_Axis.y_gyro, 51.0);
		//Z_angle = kalman_filter(z_accel, z_gyro, 110); //t prev = 120
	//====================================Output depending on the type BEGIN======================================
		if (Angle_Type == RB_DATA_TYPE){
			Y_angle -= 90.0;
		}
		if (Y_angle <= 180.0){
			Y_angle = Y_angle;
		}else{
			Y_angle = Y_angle - 360.0;
		}
		bool temp_flag;
		#ifdef DEBUG_MOD
		temp_flag = 1;
		#endif
		if (Angle_Type == BOLLARD_DATA_TYPE || temp_flag)
		{
// 			if (abs(All_Axis_ROW.Yaccel_raw) > 12000)
// 			{
				if (All_Axis_ROW.Xaccel_raw > sensitivity)
				{
				}
				if(abs(MAX_Accel_Value_X) < abs(All_Axis_ROW.Xaccel_raw)){
					MAX_Accel_Value_X = All_Axis_ROW.Xaccel_raw;
				}
				if(abs(MAX_Accel_Value_Y) < abs(All_Axis_ROW.Yaccel_raw)){
					MAX_Accel_Value_Y = All_Axis_ROW.Yaccel_raw;
				}
				if(abs(MAX_Accel_Value_Z) < abs(All_Axis_ROW.Zaccel_raw)){
					MAX_Accel_Value_Z = All_Axis_ROW.Zaccel_raw;
				}
// 			}
// 			else
// 			{
// 				MAX_Accel_Value_X = 911;
// 				MAX_Accel_Value_Y = 911;
// 				MAX_Accel_Value_Z = 911;
// 			}
		}
	//=====================================Output depending on the type END=======================================
	#ifdef DEBUG_MOD
		arduino_ploter();
	#endif
    }
}

//----------------------------------------------------------------------------------

float kalman_filter(float measured_angle, float gyro_rate, float dt) {
	static float angle = 0;
	static float bias = 0;
	static float P[2][2] = {{0, 0}, {0, 0}};

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

void arduino_ploter(void)
{
	TX_ON;
	
	//====!!!!!!!!!!!====cod to display in arduino ide monitor=====!!!!!!!!!=======
	//UART_Transmit_String("Xa:,Ya:,Za:,Xgy:,Ygy:,Zgy:\r\n");
	//%.2f
	//UART_PrintLn("%d,%d,%d,%d,%d,%d", All_Axis_ROW.Xaccel_raw, All_Axis_ROW.Yaccel_raw, All_Axis_ROW.Zaccel_raw, All_Axis_ROW.Xgyro_raw, All_Axis_ROW.Ygyro_raw, All_Axis_ROW.Zgyro_raw);
	UART_Transmit_String("MAX_A_X:,MAX_A_Y:,MAX_A_Z:\r\n");
	UART_PrintLn("%d,%d,%d", MAX_Accel_Value_X, MAX_Accel_Value_Y, MAX_Accel_Value_Z);
	MAX_Accel_Value_X = 0;
	MAX_Accel_Value_Y = 0;
	MAX_Accel_Value_Z = 0;
}

void UART_data_procesing(void)
{
	if ( (rx_buffer[0] == Sensor_Addres) && (crc(rx_buffer, DATA_PACKAGE_SIZE-1) == rx_buffer[6]))
	{
		Angle_Type = rx_buffer[2];
		
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
		memset(rx_buffer, 0, DATA_PACKAGE_SIZE);
		
		memset(tx_buffer, 0, DATA_PACKAGE_SIZE);
		tx_buffer[0] = Sensor_Addres;
		if (Angle_Type == RB_DATA_TYPE || Angle_Type == MODULAR_RB_DATA_TYPE)
		{
			float_to_byte(Y_angle, tx_buffer, 1);
		}
		if (Angle_Type == BOLLARD_DATA_TYPE)
		{
			if (MAX_Accel_Value_X > MAX_Accel_Value_Z)
			{
				tx_buffer[1] = (MAX_Accel_Value_X >> 8) & 0xFF;//MSB
				tx_buffer[2] = MAX_Accel_Value_X & 0xFF;//LSB
			}
			else
			{
				tx_buffer[1] = (MAX_Accel_Value_Z >> 8) & 0xFF;//MSB
				tx_buffer[2] = MAX_Accel_Value_Z & 0xFF;//LSB
			}
			
			tx_buffer[3] = (MAX_Accel_Value_Y >> 8) & 0xFF;
			tx_buffer[4] = MAX_Accel_Value_Y & 0xFF;
			
			MAX_Accel_Value_X = 0;
			MAX_Accel_Value_Y = 0;
			MAX_Accel_Value_Z = 0;
		}
		tx_buffer[5] = Sensor_Addres;
		tx_buffer[6] = crc(tx_buffer, DATA_PACKAGE_SIZE-1);           //обрахувати контрольну суму і вислати останнім байтом
		
		UART_TX_Pre_Counter = 1;
	}
}

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

ISR(TIMER2_COMP_vect) {
	// Код, який виконується кожну 1 мс
	if (UART_RX_Complete_FLAG)
	{
		/*LED = !LED;*/
		UART_data_procesing();
		UART_RX_Complete_FLAG = 0;
		return;
	}
	if (UART_TX_Pre_Counter)
	{
		UART_TX_Pre_Counter = 0;
		TX_ON;
		LED = 1;
		UCSRB |= (1 << TXB8);
		tx_counter = (DATA_PACKAGE_SIZE-1);
		//Передавання пакету данних
		UDR = tx_buffer[0];
	}
}

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
	if (rx_counter < DATA_PACKAGE_SIZE-1)
	{
		rx_counter ++;
	}
	else
	{
		UART_RX_Complete_FLAG = 1;
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
		LED = 0;
		RX_ON;
	}
}

/* ======================================================================================================*/

void Timer_2_Init(void)
{
	TCCR2 |= (1 << CS22) | (1 << WGM21);//pres 64, Clear Timer on Compare Match mod
	TIMSK |= (1 << OCIE2);
	OCR2 = 230;
}

void UART_Init(uint16_t ubrr_value)
{
	UBRRH = (unsigned char)(ubrr_value >> 8);
	UBRRL = (unsigned char)ubrr_value;
	#ifdef DEBUG_MOD
	UCSRB |= (1 << TXEN) | (1 << RXEN);
	UCSRC |= (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);
	#else
	UCSRB |= (1 << RXCIE) | (1 << TXCIE) | (1 << TXEN) | (1 << RXEN) | (1 << UCSZ2);
	// Встановлення формату кадру: 9 бітів даних, 1 стоп біт, без парності
	UCSRC |= (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);	
	#endif	
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
