#ifndef __USART_H_
#define __USART_H_
#include "sys.h"
#include "usart.h"

#define HEADER_ONE 			0Xcd 					//发送数据的帧头
#define HEADER_TWO 			0Xeb 					//发送数据的帧头
#define HEADER_THREE		0Xd7 					//发送数据的帧头

#define SEND_DATA_SIZE 			154
#define RECEIVE_DATA_SIZE   10

#define Uart_Tx_DMAStream DMA2_Stream7
#define Uart_Rx_DMAStream DMA2_Stream5
	
#define BUFF_SIZE  			200  	//定义最大接收字节数 200


typedef struct mpu9250
{
	float roll;
	float pitch;
	float yaw;
	float gyro[3];
	float accel[3];
	float magnet[3];
	float quat[4];
}mpu9250_data;

//--------------------------------------------------------------------------------------
/*

传感器板子
		int status;									//小车状态，0表示未初始化，1表示正常，-1表示error
		float power;             		//电源电压【9 13】v
		float theta_imu;         		//方位角，【0 360）°
		int encoder_ppr_imu;        //车轮1转对应的编码器个数
		int encoder_delta_r;     		//右轮编码器增量， 个为单位
		int encoder_delta_l;     		//左轮编码器增量， 个为单位
		unsigned int upwoard;    		//1表示正向安装,0表示反向安装
		unsigned int hbz_status; 		//急停按钮
		float sonar_distance[4]; 		//超声模块距离值 单位m
		float quat[4];          		//IMU四元数
		float IMU[9];           		//IMU 9轴数据
		unsigned int time_stamp_imu;//时间戳

电源控制板
--		  float power;    //电源电压[1.0, 4.0]v				
				float battery;  //电池电压               			应用程
--		  float current;  //充电电流 [0.005,0.1]					转换
		    unsigned int left_sensor1;
--		  unsigned int left_sensor2;  [右边  1]  				合起来的情况
--		  unsigned int right_sensor2; [左边  2]					
		    unsigned int right_sensor1;
--		  float distance1;
  		  float distance2;
		    unsigned int time_stamp;  //时间戳
		    unsigned int version;         						-- 3  应用层

上传的数据包格式：包头+长度+内容
*/

//--------------------------------------------------------------------------------------


typedef struct _SEND_DATA_  
{
		unsigned char buffer[SEND_DATA_SIZE];

		int status;												//小车状态，0表示未初始化，1表示正常，-1表示error
		float power;             					//电源电压【9 13】v
		float theta_imu;         					//方位角，【0 360）°
		int encoder_ppr_imu;        			//车轮1转对应的编码器个数
		int encoder_delta_r;     					//右轮编码器增量， 个为单位
		int encoder_delta_l;     					//左轮编码器增量， 个为单位
		unsigned int upwoard;    					//1表示正向安装,0表示反向安装
		unsigned int hbz_status; 					//急停按钮
		float sonar_distance[4]; 					//超声模块距离值 单位m
		float quat[4];          					//IMU四元数
		float IMU[9];           					//IMU 9轴数据
		unsigned int time_stamp_imu;			//时间戳
				
		float current;										//电流		
		unsigned int left_sensor2;				//红外									
	  unsigned int right_sensor2;       //红外
	  float distance1;       						//后置超声波
	
}SEND_DATA;														

typedef struct _RECEIVE_DATA_  
{
	unsigned char buffer[RECEIVE_DATA_SIZE];

}RECEIVE_DATA;

void MYDMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr);
void uart_DMA_send(u8 *str,u16 ndtr);

int USART_DATA_Handler(u8 *data,u8 cnt);
void uart_init(u32 bound);
void usart1_send(u8 data);
void USART1_SEND(int len);
void data_transition(void);
void error_transition(void);
u8 Check_Sum(unsigned char Count_Number,unsigned char Mode);

extern short gyro[3], accel[3],magnet[3];
extern SEND_DATA Send_Data;	
extern u8 imu_reset; 
#endif


