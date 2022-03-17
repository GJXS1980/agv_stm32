#ifndef __USART_H_
#define __USART_H_
#include "sys.h"
#include "usart.h"

#define HEADER_ONE 			0Xcd 					//�������ݵ�֡ͷ
#define HEADER_TWO 			0Xeb 					//�������ݵ�֡ͷ
#define HEADER_THREE		0Xd7 					//�������ݵ�֡ͷ

#define SEND_DATA_SIZE 			154
#define RECEIVE_DATA_SIZE   10

#define Uart_Tx_DMAStream DMA2_Stream7
#define Uart_Rx_DMAStream DMA2_Stream5
	
#define BUFF_SIZE  			200  	//�����������ֽ��� 200


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

����������
		int status;									//С��״̬��0��ʾδ��ʼ����1��ʾ������-1��ʾerror
		float power;             		//��Դ��ѹ��9 13��v
		float theta_imu;         		//��λ�ǣ���0 360����
		int encoder_ppr_imu;        //����1ת��Ӧ�ı���������
		int encoder_delta_r;     		//���ֱ����������� ��Ϊ��λ
		int encoder_delta_l;     		//���ֱ����������� ��Ϊ��λ
		unsigned int upwoard;    		//1��ʾ����װ,0��ʾ����װ
		unsigned int hbz_status; 		//��ͣ��ť
		float sonar_distance[4]; 		//����ģ�����ֵ ��λm
		float quat[4];          		//IMU��Ԫ��
		float IMU[9];           		//IMU 9������
		unsigned int time_stamp_imu;//ʱ���

��Դ���ư�
--		  float power;    //��Դ��ѹ[1.0, 4.0]v				
				float battery;  //��ص�ѹ               			Ӧ�ó�
--		  float current;  //������ [0.005,0.1]					ת��
		    unsigned int left_sensor1;
--		  unsigned int left_sensor2;  [�ұ�  1]  				�����������
--		  unsigned int right_sensor2; [���  2]					
		    unsigned int right_sensor1;
--		  float distance1;
  		  float distance2;
		    unsigned int time_stamp;  //ʱ���
		    unsigned int version;         						-- 3  Ӧ�ò�

�ϴ������ݰ���ʽ����ͷ+����+����
*/

//--------------------------------------------------------------------------------------


typedef struct _SEND_DATA_  
{
		unsigned char buffer[SEND_DATA_SIZE];

		int status;												//С��״̬��0��ʾδ��ʼ����1��ʾ������-1��ʾerror
		float power;             					//��Դ��ѹ��9 13��v
		float theta_imu;         					//��λ�ǣ���0 360����
		int encoder_ppr_imu;        			//����1ת��Ӧ�ı���������
		int encoder_delta_r;     					//���ֱ����������� ��Ϊ��λ
		int encoder_delta_l;     					//���ֱ����������� ��Ϊ��λ
		unsigned int upwoard;    					//1��ʾ����װ,0��ʾ����װ
		unsigned int hbz_status; 					//��ͣ��ť
		float sonar_distance[4]; 					//����ģ�����ֵ ��λm
		float quat[4];          					//IMU��Ԫ��
		float IMU[9];           					//IMU 9������
		unsigned int time_stamp_imu;			//ʱ���
				
		float current;										//����		
		unsigned int left_sensor2;				//����									
	  unsigned int right_sensor2;       //����
	  float distance1;       						//���ó�����
	
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


