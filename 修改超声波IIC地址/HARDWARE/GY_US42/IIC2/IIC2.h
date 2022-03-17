#ifndef _IIC_T_H
#define _IIC_T_H
#include "sys.h"

//IO方向设置
#define ULT_SDA_IN_T()  {GPIOB->MODER&=~(3<<(5*2));GPIOB->MODER|=0<<5*2;}	//PB9输入模式
#define ULT_SDA_OUT_T() {GPIOB->MODER&=~(3<<(5*2));GPIOB->MODER|=1<<5*2;} //PB9输出模式
//IO操作函数	 
#define ULT_IIC_SCL_T    PBout(4) //SCL
#define ULT_IIC_SDA_T    PBout(5) //SDA	 
#define ULT_READ_SDA_T   PBin(5)  //输入SDA 

//IIC所有操作函数
void ULT_IIC_Init_t(void);                //初始化IIC的IO口				 
void ULT_IIC_Start_t(void);								//发送IIC开始信号
void ULT_IIC_Stop_t(void);	  						//发送IIC停止信号
void ULT_IIC_Send_Byte_t(u8 txd);					//IIC发送一个字节
u8 ULT_IIC_Read_Byte_t(unsigned char ack);//IIC读取一个字节
u8 ULT_IIC_Wait_Ack_t(void); 							//IIC等待ACK信号
void ULT_IIC_Ack_t(void);									//IIC发送ACK信号
void ULT_IIC_NAck_t(void);								//IIC不发送ACK信号 


u8 takeRangeReading_t(u8 Slave_Address);
u8 requestRange_t(u8 Slave_Address,uint16_t *distance);
u8 changeAddress_t(u8 oldAddress, u8 newAddress);
u8 find_addr_t(u8 add);

#endif

