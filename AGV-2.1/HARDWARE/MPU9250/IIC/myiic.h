#ifndef __MPUIIC_H
#define __MPUIIC_H
#include "sys.h"
 		   
//IO��������
#define SDA_IN()  {GPIOA->MODER&=~(3<<(2*2));GPIOA->MODER|=0<<2*2;}	//PB9����ģʽ
#define SDA_OUT() {GPIOA->MODER&=~(3<<(2*2));GPIOA->MODER|=1<<2*2;} //PB9���ģʽ
//IO��������	 
#define IIC_SCL    PAout(1) //SCL
#define IIC_SDA    PAout(2) //SDA	 
#define READ_SDA   PAin(2)  //����SDA 

//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź� 

#endif















