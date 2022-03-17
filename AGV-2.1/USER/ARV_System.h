#ifndef 	_ARV_SYSTEM_H
#define 	_ARV_SYSTEM_H
#include "sys.h"
#include "stdio.h"
#include "delay.h"
#include "usart.h"
#include "string.h"
#include "math.h"
#include "adc.h"
#include "iwdg.h"
#include "IIC.h"
#include "adc.h"

#define ON  	1
#define OFF	 	0
typedef struct Deviation_gyro
{
	float roll_z;
	float pitch_z;
	float yaw_z;

}MPU_Zero;	

typedef struct Sensor_List
{
		u8 MPU_9250;
		u8 MPU9250_RESET;
		u8 Ultrasonic;
		u8 Temperature;
		u8 BEEP;
		u8 Infrared;
		u8 DHT11;
	
}Sensor_Switch;

typedef struct code
{
	u8 	MPU_9250;								//�����Ǵ���
	u8 	Ult_Outtime ;
	u8	Ultrasonic[3];					//����������
	u8	Temperature;						//��ʪ�ȴ���
	u8	Infrared[2];						//�����ߴ���
	
	
}Error;


void claer_zero(void);
void System_Init(void);
void Parameter_Init(void);

#endif 
