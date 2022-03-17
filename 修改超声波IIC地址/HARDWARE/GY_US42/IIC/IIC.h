#ifndef _IIC_H
#define _IIC_H
#include "sys.h"

//IO��������
#define ULT_SDA_IN()  {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=0<<9*2;}	//PB9����ģʽ
#define ULT_SDA_OUT() {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=1<<9*2;} //PB9���ģʽ
//IO��������	 
#define ULT_IIC_SCL    PBout(8) //SCL
#define ULT_IIC_SDA    PBout(9) //SDA	 
#define ULT_READ_SDA   PBin(9)  //����SDA 

//IIC���в�������
void ULT_IIC_Init(void);                //��ʼ��IIC��IO��				 
void ULT_IIC_Start(void);								//����IIC��ʼ�ź�
void ULT_IIC_Stop(void);	  						//����IICֹͣ�ź�
void ULT_IIC_Send_Byte(u8 txd);					//IIC����һ���ֽ�
u8 ULT_IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 ULT_IIC_Wait_Ack(void); 							//IIC�ȴ�ACK�ź�
void ULT_IIC_Ack(void);									//IIC����ACK�ź�
void ULT_IIC_NAck(void);								//IIC������ACK�ź� 


u8 takeRangeReading(u8 Slave_Address);
u8 requestRange(u8 Slave_Address,uint16_t *distance);
u8 changeAddress(u8 oldAddress, u8 newAddress);
u8 find_addr(u8 add);

#endif

