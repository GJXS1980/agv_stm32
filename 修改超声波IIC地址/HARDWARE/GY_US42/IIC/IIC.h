#ifndef _IIC_H
#define _IIC_H
#include "sys.h"

//IO方向设置
#define ULT_SDA_IN()  {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=0<<9*2;}	//PB9输入模式
#define ULT_SDA_OUT() {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=1<<9*2;} //PB9输出模式
//IO操作函数	 
#define ULT_IIC_SCL    PBout(8) //SCL
#define ULT_IIC_SDA    PBout(9) //SDA	 
#define ULT_READ_SDA   PBin(9)  //输入SDA 

//IIC所有操作函数
void ULT_IIC_Init(void);                //初始化IIC的IO口				 
void ULT_IIC_Start(void);								//发送IIC开始信号
void ULT_IIC_Stop(void);	  						//发送IIC停止信号
void ULT_IIC_Send_Byte(u8 txd);					//IIC发送一个字节
u8 ULT_IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 ULT_IIC_Wait_Ack(void); 							//IIC等待ACK信号
void ULT_IIC_Ack(void);									//IIC发送ACK信号
void ULT_IIC_NAck(void);								//IIC不发送ACK信号 


u8 takeRangeReading(u8 Slave_Address);
u8 requestRange(u8 Slave_Address,uint16_t *distance);
u8 changeAddress(u8 oldAddress, u8 newAddress);
u8 find_addr(u8 add);

#endif

