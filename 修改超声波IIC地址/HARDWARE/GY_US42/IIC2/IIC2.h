#ifndef _IIC_T_H
#define _IIC_T_H
#include "sys.h"

//IO��������
#define ULT_SDA_IN_T()  {GPIOB->MODER&=~(3<<(5*2));GPIOB->MODER|=0<<5*2;}	//PB9����ģʽ
#define ULT_SDA_OUT_T() {GPIOB->MODER&=~(3<<(5*2));GPIOB->MODER|=1<<5*2;} //PB9���ģʽ
//IO��������	 
#define ULT_IIC_SCL_T    PBout(4) //SCL
#define ULT_IIC_SDA_T    PBout(5) //SDA	 
#define ULT_READ_SDA_T   PBin(5)  //����SDA 

//IIC���в�������
void ULT_IIC_Init_t(void);                //��ʼ��IIC��IO��				 
void ULT_IIC_Start_t(void);								//����IIC��ʼ�ź�
void ULT_IIC_Stop_t(void);	  						//����IICֹͣ�ź�
void ULT_IIC_Send_Byte_t(u8 txd);					//IIC����һ���ֽ�
u8 ULT_IIC_Read_Byte_t(unsigned char ack);//IIC��ȡһ���ֽ�
u8 ULT_IIC_Wait_Ack_t(void); 							//IIC�ȴ�ACK�ź�
void ULT_IIC_Ack_t(void);									//IIC����ACK�ź�
void ULT_IIC_NAck_t(void);								//IIC������ACK�ź� 


u8 takeRangeReading_t(u8 Slave_Address);
u8 requestRange_t(u8 Slave_Address,uint16_t *distance);
u8 changeAddress_t(u8 oldAddress, u8 newAddress);
u8 find_addr_t(u8 add);

#endif

