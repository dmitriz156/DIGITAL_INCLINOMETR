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
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

//#define DEBUG_MOD

// Constants
#define TAU 0.98  // ��������������� ����������
#define DT 0.1     // ��� �� ����������� (100 Hz)
#define GYRO_SCALE	(2000/16384)*(M_PI/180)

#define FILTER_WINDOW_SIZE 5 // ����� ���� �������
float buffer[FILTER_WINDOW_SIZE] = {0};

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
kalman_t        X_Axis       = {0};
kalman_t        Y_Axis       = {0};
kalman_t        Z_Axis       = {0};

volatile float X_angle = 0;
volatile float Y_angle = 0;
volatile float Z_angle = 0;
volatile float Calc_Y_angle = 0;
float Ploter_angle = 0;
float Prev_Y_angle = 0;

int16_t Angle_OLD;

uint8_t UART_NeedToSend = 0;
uint8_t sensitivity = 0;
int16_t MAX_Accel_Value_X = 0;
int16_t MAX_Accel_Value_Y = 0;
int16_t MAX_Accel_Value_Z = 0;

int16_t filt_X;
float filt_Y;
int16_t filt_Z;
float y_accel_OLD;

int16_t temp;
/* ---------------------------------------- */

#define MPU6050_R			0b11010001
#define MPU6050_W			0b11010000

//����� �������� � ������� ����
volatile uint8_t Rx_buf[DATA_PACKAGE_SIZE];
volatile uint8_t Tx_buf[4];

//��������� ������� ��� ������
bool first_bit;

uint32_t Led_counter=0;
uint8_t bufer;
uint8_t Temp;

volatile uint8_t rx_buffer[DATA_PACKAGE_SIZE];
volatile uint8_t tx_buffer[DATA_PACKAGE_SIZE];
//volatile uint8_t address_change_Buffer[DATA_PACKAGE_SIZE];

uint8_t MPU_status_A = _MPU_OK;
uint8_t MPU_status_G = _MPU_OK;

bool re_init_flag = 0;
uint8_t ERROR_status_counter = 0;
uint16_t one_sec_counter = 0;
volatile uint8_t LED_Elive_Counter = 0; 
volatile uint8_t Address_Change_Counter = 0;
volatile uint8_t Position_Change_Counter = 0;
volatile bool Address_Change_FLAG = 0;
volatile bool Old_Data_Flag = 0;
volatile bool UART_RX_Complete_FLAG = 0;
volatile bool UART_TX_Complete_FLAG = 0;
volatile uint8_t UART_TX_Pre_Counter = 0;
volatile uint8_t UART_Idle_Line_Counter = 0;
volatile data_type_t Angle_Type = 0;
volatile uint8_t Sensor_Addres = 0;
volatile uint8_t Sensor_Position = 0;
bool AvrgFLAG = 0;
uint8_t rx_counter, rx_counter;
// This flag is set on USART Receiver buffer overflow
bool rx_buffer_overflow;

/*------kalman_filter coeficient------*/
#define KOEF			0.98f // for simple Kalman filter

#define KALMAN_ACEL_Q	0.025f//0.01
#define KALMAN_GYRO_Q	0.02f//0.03
#define KALMAN_R		0.018f//0.015

/*------kalman_filter coeficient------*/
float angle;
float bias;
float P[2][2] = {{0, 0}, {0, 0}};

// ������� ��� �������� �������� ���� float �� ����� ����� uint8_t
void float_to_byte(float value, uint8_t *array, int start_index) {
	uint8_t *byte_ptr = (uint8_t *)&value; // �������� �� ������ ���� float-��������

	for (int i = 0; i < sizeof(float); i++) {
		array[start_index + i] = *(byte_ptr + i); // ����� ������� ����� � �����
	}
}

// ������� ��� ����������� ������ ����� uint8_t � �������� ���� float
float byte_to_float(uint8_t *array, int start_index) {
	float value;
	uint8_t *byte_ptr = (uint8_t *)&value; // �������� �� ������ ���� float-��������

	for (int i = 0; i < sizeof(float); i++) {
		*(byte_ptr + i) = array[start_index + i]; // ���������� ������� ����� � ������
	}

	return value;
}


int main(void)
{
	Sensor_Addres = eeprom_read_byte((uint8_t*)EEPROM_SENS_ADDR);
	Sensor_Position = eeprom_read_byte((uint8_t*)EEPROM_POSITION);
	if (Sensor_Addres > 4 || Sensor_Addres == 0)
	{
		Sensor_Addres = ANGLE_SENS_ADDRESS_1;
		eeprom_write_byte((uint8_t*)EEPROM_SENS_ADDR, Sensor_Addres);
	}
	sei();
	wdt_enable(WDTO_1S);
	_LCD_PORT_DIR = _ALL_OUTPUT;
	_I2C_PORT_DIR = ( 1 << _SDA ) | _SCL;
	
	/* MPU6050 Init */
	MPU6050_AutoInit(100);
	Timer_0_Init();
	Timer_2_Init();
	UART_Init(UBRR_SPEED);
	RX_TX_DIRECTION = 1;
	LED_OUT = 1;
	LED = 1;
	_delay_ms(100);
	//MPU6050_Calibrate(offsetGX, offsetGY, offsetGZ);
	/* ---------------------------------------- */
    while (1) 
    {
		if (re_init_flag == 1)
		{
			re_init_flag = 0;
			MPU6050_AutoInit(100);
			_delay_ms(10);
		}
		MPU_status_A = MPU6050_GetRawAccel(&All_Axis_ROW.Xaccel_raw, 20);
		MPU_status_G = MPU6050_GetRawGyro(&All_Axis_ROW.Xgyro_raw, 20);

		if ((MPU_status_A == _MPU_ERROR || MPU_status_G == _MPU_ERROR) && ERROR_status_counter == 0){
			ERROR_status_counter = 5;
		}
		if ((MPU_status_A == _MPU_OK || MPU_status_G == _MPU_OK) && ERROR_status_counter != 0){
			ERROR_status_counter = 0;
		}

		wdt_reset();
		All_Axis.F_x_accel = All_axis_kalman_filter(&X_Axis, (float)All_Axis_ROW.Xaccel_raw, (float)All_Axis_ROW.Xgyro_raw, 0.0053f);
		All_Axis.F_y_accel = All_axis_kalman_filter(&Y_Axis, (float)All_Axis_ROW.Yaccel_raw, (float)All_Axis_ROW.Ygyro_raw, 0.0053f);
		All_Axis.F_z_accel = All_axis_kalman_filter(&Z_Axis, (float)All_Axis_ROW.Zaccel_raw, (float)All_Axis_ROW.Zgyro_raw, 0.0053f);
		
		#ifdef DEBUG_MOD
		Ploter_angle = ( _MPU_RAD_TO_DEG * ( atan2( -All_Axis_ROW.Xaccel_raw , -All_Axis_ROW.Zaccel_raw ) + _MATH_PI ) );
		#endif
		Calc_Y_angle = ( _MPU_RAD_TO_DEG * ( atan2( -All_Axis.F_x_accel , -All_Axis.F_z_accel ) + _MATH_PI ) );
		//if (Lutch == 0) 
// 		All_Axis.x_accel = ( _MPU_RAD_TO_DEG * ( atan2( -All_Axis_ROW.Yaccel_raw , -All_Axis_ROW.Zaccel_raw ) + _MATH_PI ) );
//		...
// 		All_Axis.x_gyro = ((All_Axis_ROW.Xgyro_raw /*- offsetGX*/) / _MPU_GYRO_SENS_250_SENS /*/ (1000 / 120)*/);

// 		//===============Complementary filter begin================
// 		filtered_Y = (int16_t)((0.98 * y_accel) - (0.02 * y_gyro));
//		Y_angle = KOEF * (y_accel * 0.0053) + (1 - KOEF) * y_gyro;
// 		//================Complementary filter end================

// 		//==================Simple KALMAN filter==================
// 		filt_Y = KOEF * All_Axis.y_accel + (1 - KOEF) * y_accel_OLD;
// 		y_accel_OLD = filt_Y;
// 		//========================================================

		//Y_angle = kalman_filter(All_Axis.y_accel, All_Axis.y_gyro, 0.0053f); //5.3 ms
		//Ploter_angle = Y_angle;
		
	//====================================Output depending on the type BEGIN======================================
		
		if (Angle_Type == RB_DATA_TYPE){
			Calc_Y_angle -= 90.0f;
		}
		if (Calc_Y_angle <= 180.0f){
			Y_angle = Calc_Y_angle;
		}else{
			Y_angle = Calc_Y_angle - 360.0f;
		}
		
		bool temp_flag = 0;
		#ifdef DEBUG_MOD
		temp_flag = 1;
		#endif
		
		if (Angle_Type == BOLLARD_DATA_TYPE || temp_flag == 0)
		{
			if(abs(MAX_Accel_Value_X) < abs(All_Axis_ROW.Xaccel_raw)){
				MAX_Accel_Value_X = All_Axis_ROW.Xaccel_raw;
			}
			if(abs(MAX_Accel_Value_Y) < abs(All_Axis_ROW.Yaccel_raw)){
				MAX_Accel_Value_Y = All_Axis_ROW.Yaccel_raw;
			}
			if(abs(MAX_Accel_Value_Z) < abs(All_Axis_ROW.Zaccel_raw)){
				MAX_Accel_Value_Z = All_Axis_ROW.Zaccel_raw;
			}
		}
		//>>>Angle formation remained the same as in OLD version<<<
		if (Sensor_Position){
			//>>Standart Z/X
			Angle_OLD = 10000 * atan( (All_Axis.F_z_accel / All_Axis.F_x_accel) );
		} else {
			//>>Mobile modular SB (N position) X/Z
			Angle_OLD = 10000 * atan( (-All_Axis.F_x_accel / All_Axis.F_z_accel) );
		}
		
		Prev_Y_angle = Y_angle;
	//=====================================Output depending on the type END=======================================
	#ifdef DEBUG_MOD
		arduino_ploter();
	#endif
    }
}

//----------------------------------------------------------------------------------

float kalman_filter(float measured_angle, float gyro_rate, float dt) {
// 	static float angle;
// 	static float bias;
// 	static float P[2][2];// = {{0, 0}, {0, 0}};

	// ������ ����: ������ ������ ����� �� ����� ��������
	angle += dt * (gyro_rate - bias);

	// ������ ����: ��������� ����������� ������� �� ����� ��������
	P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + KALMAN_ACEL_Q);
	P[0][1] -= dt * P[1][1];
	P[1][0] -= dt * P[1][1];
	P[1][1] += KALMAN_GYRO_Q * dt;

	// ����� ����: ����������� ������������� �����������
	float S = P[0][0] + KALMAN_R;
	float K[2]; // ������������ ����������
	K[0] = P[0][0] / S;
	K[1] = P[1][0] / S;

	// ��������� ����: ��������� ������ �� ����� �����
	float y = measured_angle - angle; // ������� �����
	angle += K[0] * y;
	bias += K[1] * y;

	// �'���� ����: ��������� ����������� ������� �� ����� �����
	float P00_temp = P[0][0];
	float P01_temp = P[0][1];

	P[0][0] -= K[0] * P00_temp;
	P[0][1] -= K[0] * P01_temp;
	P[1][0] -= K[1] * P00_temp;
	P[1][1] -= K[1] * P01_temp;

	return angle;
}

float All_axis_kalman_filter(kalman_t *axis, float measured_angle, float gyro_rate, float dt) {

	// ������ ����: ������ ������ ����� �� ����� ��������
	axis->angle += dt * (gyro_rate - axis->bias);

	// ������ ����: ��������� ����������� ������� �� ����� ��������
	axis->P[0][0] += dt * (dt * axis->P[1][1] - axis->P[0][1] - axis->P[1][0] + KALMAN_ACEL_Q);
	axis->P[0][1] -= dt * axis->P[1][1];
	axis->P[1][0] -= dt * axis->P[1][1];
	axis->P[1][1] += KALMAN_GYRO_Q * dt;

	// ����� ����: ����������� ������������� �����������
	float S = axis->P[0][0] + KALMAN_R;
	float K[2]; // ������������ ����������
	K[0] = axis->P[0][0] / S;
	K[1] = axis->P[1][0] / S;

	// ��������� ����: ��������� ������ �� ����� �����
	float y = measured_angle - axis->angle; // ������� �����
	axis->angle += K[0] * y;
	axis->bias += K[1] * y;

	// �'���� ����: ��������� ����������� ������� �� ����� �����
	float P00_temp = axis->P[0][0];
	float P01_temp = axis->P[0][1];

	axis->P[0][0] -= K[0] * P00_temp;
	axis->P[0][1] -= K[0] * P01_temp;
	axis->P[1][0] -= K[1] * P00_temp;
	axis->P[1][1] -= K[1] * P01_temp;

	return axis->angle;
}

void arduino_ploter(void)
{
	TX_ON;
	
	//====!!!!!!!!!!!====cod to display in arduino ide monitor=====!!!!!!!!!=======
	//%.2f
// 	UART_Transmit_String("angle_y:, filter_y \r\n");
// 	UART_PrintLn("%d,%d", (int16_t)Ploter_angle, (int16_t)Y_angle);
}

void UART_data_procesing(void)
{
	uint8_t Old_Data_Addres = 0;
	
	if (Old_Data_Flag == 1)
	{
		if (Sensor_Addres == 1) {
			Old_Data_Addres = Sensor_Addres;
		} else if (Sensor_Addres > 1) {
			Old_Data_Addres = Sensor_Addres + 1;
		}
		
		if ( (rx_buffer[0] == Old_Data_Addres) && (crc(rx_buffer, OLD_DATA_PACKAGE_SIZE-1) == rx_buffer[4]))
		{
			LED = !LED;
			memset(rx_buffer, 0, DATA_PACKAGE_SIZE);
			memset(tx_buffer, 0, DATA_PACKAGE_SIZE);
			tx_buffer[0] = Old_Data_Addres;

			tx_buffer[1] = (Angle_OLD >> 8) & 0xFF;//MSB
			tx_buffer[2] = Angle_OLD & 0xFF;//LSB
				
			tx_buffer[3] = Old_Data_Addres;
			tx_buffer[4] = crc(tx_buffer, OLD_DATA_PACKAGE_SIZE-1);           //���������� ���������� ���� � ������� ������� ������
			
			TX_ON;
			UART_TX_Pre_Counter = 1;
		}
		return;
	}

	if ( (rx_buffer[0] == Sensor_Addres) && (crc(rx_buffer, DATA_PACKAGE_SIZE-1) == rx_buffer[6]))
	{
		LED = !LED;
		Angle_Type = rx_buffer[2];
			
		if (rx_buffer[3] == ADDRESS_CHANGE_KEY)
		{
			if (Address_Change_Counter < 10)
			{
				Address_Change_Counter ++;
			}else if (Address_Change_Counter == 10){
				if (rx_buffer[4] <= ANGLE_SENS_ADDRESS_4 && rx_buffer[4] != 0)
				{
					Sensor_Addres = rx_buffer[4];
					eeprom_write_byte((uint8_t*)EEPROM_SENS_ADDR, Sensor_Addres);
				}
				else
				{
					Sensor_Addres = ANGLE_SENS_ADDRESS_1;
					eeprom_write_byte((uint8_t*)EEPROM_SENS_ADDR, Sensor_Addres);
				}
				Address_Change_Counter = 0;
			}
		}else{
			Address_Change_Counter = 0;
		}
			
		if (rx_buffer[3] == POSITION_CHANGE_KEY)
		{
			if (Position_Change_Counter < 10)
			{
				Position_Change_Counter ++;
			} else if (Position_Change_Counter == 10){
				if (Sensor_Position == 0){
					Sensor_Position = 0xFF;
					eeprom_write_byte((uint8_t*)EEPROM_POSITION, 0xFF);
				} else {
					Sensor_Position = 0;
					eeprom_write_byte((uint8_t*)EEPROM_POSITION, 0);
				}
				Position_Change_Counter = 0;
			}
		}else{
			Position_Change_Counter = 0;
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
			if (abs(MAX_Accel_Value_X) > abs(MAX_Accel_Value_Z))
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
				
			MAX_Accel_Value_X = 11;
			MAX_Accel_Value_Y = 11;
			MAX_Accel_Value_Z = 11;
				
		}
		tx_buffer[5] = Sensor_Addres;
		tx_buffer[6] = crc(tx_buffer, DATA_PACKAGE_SIZE-1);           //���������� ���������� ���� � ������� ������� ������
			
		TX_ON;
		UART_TX_Pre_Counter = 1;
	}

}

//�������� ���������� ���
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

ISR(TIMER0_OVF_vect) {
	TCNT0 = 75;
	if (UART_Idle_Line_Counter == 1 && (rx_counter == OLD_DATA_PACKAGE_SIZE))
	{
		Old_Data_Flag = 1;
		UART_RX_Complete_FLAG = 1;
	}
	
	if (UART_Idle_Line_Counter > 0)
	{
		UART_Idle_Line_Counter --;
	}
}

ISR(TIMER2_COMP_vect) {
	// ���, ���� ���������� ����� 1 ��
	wdt_reset();
	//>> One sec counting & MPU_6050 data receive ERRORs handlind << BEGIN
	if(one_sec_counter < 1000){
		one_sec_counter ++;
	} else {
		if (ERROR_status_counter > 0){
			if (ERROR_status_counter == 1){
				re_init_flag = 1;
			}
			ERROR_status_counter --;
		}
		one_sec_counter = 0;
	}
	//>> One sec counting & MPU_6050 data receive ERRORs handlind << END
	
	if (UART_TX_Pre_Counter == 1)
	{
		UART_TX_Pre_Counter = 0;
		//TX_ON;
		UCSRB |= (1 << TXB8);
		if (Old_Data_Flag) tx_counter = (OLD_DATA_PACKAGE_SIZE-1);
		else tx_counter = (DATA_PACKAGE_SIZE-1);
		//����������� ������ ������
		UDR = tx_buffer[0];
	}
	
	if (LED_Elive_Counter > 0){ LED_Elive_Counter --; }
		
	if(UART_TX_Complete_FLAG){
		UART_TX_Complete_FLAG = 0;
		RX_ON;
	}
		
	if (UART_RX_Complete_FLAG)
	{
		UART_data_procesing();
		UART_RX_Complete_FLAG = 0;
		LED_Elive_Counter = 200;
	}
	else
	{
		if(LED_Elive_Counter == 1){ 
			RX_ON;
		}
		if (LED_Elive_Counter == 0)
		{
			LED_Elive_Counter = 200;
			LED = !LED;
		}
	}
}

ISR(USART_RXC_vect)
{
	if (USART_RX_9BIT == 1)
	{
		rx_counter = 0;
		Old_Data_Flag = 0;
	}
	rx_buffer [rx_counter] = UDR;
	UART_Idle_Line_Counter = 13;
	
	if (rx_counter < DATA_PACKAGE_SIZE - 1) {
		rx_counter ++;
	}
	else
	{
		UART_RX_Complete_FLAG = 1;
		UART_Idle_Line_Counter = 0;
		Old_Data_Flag = 0;
	}
}


ISR(USART_TXC_vect)
{
	if (tx_counter > 0)
	{
		tx_counter--;
		UCSRB &= ~(1 << TXB8);       //������� 9-� ��
		
		if (Old_Data_Flag) {
			UDR = tx_buffer[(OLD_DATA_PACKAGE_SIZE-1) - tx_counter];
		} else {
			UDR = tx_buffer[(DATA_PACKAGE_SIZE-1) - tx_counter];
		}
	}
	else
	{
		Old_Data_Flag = 0;
		UART_TX_Complete_FLAG = 1;
	}
}

/* ======================================================================================================*/

void Timer_2_Init(void)
{
	TCCR2 |= (1 << CS22) | (1 << WGM21);//pres 64, Clear Timer on Compare Match mod
	TIMSK |= (1 << OCIE2);
	OCR2 = 230;
}
void Timer_0_Init(void)
{
	TCCR0 |= (1 << CS01);
	TIMSK |= (1 << TOIE0);
	TCNT0 = 75;
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
	// ������������ ������� �����: 9 ��� �����, 1 ���� ��, ��� �������
	UCSRC |= (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);	
	#endif	
}

void UART_Transmit(char data) {
	// ������ �� ������ ����� ��������
	while (!(UCSRA & (1 << UDRE)));
	// �������� ��� � ������ �����
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
float moving_average(float new_value) {
	static int index;
	static float sum;

	sum -= buffer[index];
	buffer[index] = new_value;
	sum += new_value;
	index = (index + 1) % FILTER_WINDOW_SIZE;

	return sum / FILTER_WINDOW_SIZE;
}
//==================================================================================
