#include "IIC2.h"
#include "delay.h"

//初始化IIC
void ULT_IIC_Init_t(void)
{					     
	RCC->AHB1ENR|=1<<1;    //使能PORTH时钟	   	  
	GPIO_Set(GPIOB,PIN4|PIN5,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);
	ULT_IIC_SCL_T=1;
	ULT_IIC_SDA_T=1;
}
//产生IIC起始信号
void ULT_IIC_Start_t(void)
{
	ULT_SDA_OUT_T();     //sda线输出
	ULT_IIC_SDA_T=1;	  	  
	ULT_IIC_SCL_T=1;
	delay_us(5);
 	ULT_IIC_SDA_T=0;//START:when CLK is high,DATA change form high to low 
	delay_us(5);
	ULT_IIC_SCL_T=0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void ULT_IIC_Stop_t(void)
{
	ULT_SDA_OUT_T();//sda线输出
	ULT_IIC_SCL_T=0;
	ULT_IIC_SDA_T=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(5);
	ULT_IIC_SCL_T=1; 
	ULT_IIC_SDA_T=1;//发送I2C总线结束信号
	delay_us(5);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 ULT_IIC_Wait_Ack_t(void)
{
	u8 ucErrTime=0;
	ULT_SDA_IN_T();      //SDA设置为输入  
	ULT_IIC_SDA_T=1;	   
	ULT_IIC_SCL_T=1;	
	delay_us(5);
	
	while(ULT_READ_SDA_T)
	{
		ucErrTime++;
		if(ucErrTime>200)
		{
			ULT_IIC_Stop_t();
			return 1;
		}
	
	}
	delay_us(5);	
	ULT_IIC_SCL_T=0;//时钟输出0 
	delay_us(5);		

	return 0;  
} 
//产生ACK应答
void ULT_IIC_Ack_t(void)
{
	ULT_IIC_SCL_T=0;
	ULT_SDA_OUT_T();
	ULT_IIC_SDA_T=0;
	delay_us(5);
	ULT_IIC_SCL_T=1;
	delay_us(5);
	ULT_IIC_SCL_T=0;
}
//不产生ACK应答		    
void ULT_IIC_NAck_t(void)
{
	ULT_IIC_SCL_T=0;
	ULT_SDA_OUT_T();
	ULT_IIC_SDA_T=1;
	delay_us(5);
	ULT_IIC_SCL_T=1;
	delay_us(5);
	ULT_IIC_SCL_T=0;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void ULT_IIC_Send_Byte_t(u8 txd)
{                        
    u8 t;   
	ULT_SDA_OUT_T(); 	    
    ULT_IIC_SCL_T=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        ULT_IIC_SDA_T=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(5);   						
		ULT_IIC_SCL_T=1;
				delay_us(5); 
				ULT_IIC_SCL_T=0;	
				delay_us(5);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 ULT_IIC_Read_Byte_t(unsigned char ack)
{
	unsigned char i,receive=0;
	ULT_SDA_IN_T();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        ULT_IIC_SCL_T=0; 
        delay_us(5);
		ULT_IIC_SCL_T=1;
        receive<<=1;
        if(ULT_READ_SDA_T)receive++;   
			delay_us(5); 
    }					 
    if (!ack)
        ULT_IIC_NAck_t();//发送nACK
    else
        ULT_IIC_Ack_t(); //发送ACK   
    return receive;
}

u8 takeRangeReading_t(u8 Slave_Address)
{
	ULT_IIC_Start_t();       
    ULT_IIC_Send_Byte_t(Slave_Address);   //发送设备地址+写信号
 	ULT_IIC_Wait_Ack_t();  
	ULT_IIC_Send_Byte_t(0x51);    				//内部寄存器地址，
 	ULT_IIC_Wait_Ack_t();
	ULT_IIC_Stop_t();   	
	return SET;
}

u8 requestRange_t(u8 Slave_Address,uint16_t *distance)
{
	u8 REG_data[2]={0,0};
	ULT_IIC_Start_t();        
	ULT_IIC_Send_Byte_t(Slave_Address);    		//发送设备地址+写信号
	ULT_IIC_Wait_Ack_t(); 
  	delay_us(5);  
	REG_data[0]=ULT_IIC_Read_Byte_t(1);       	//读出寄存器数据
	REG_data[1]=ULT_IIC_Read_Byte_t(0);       	//读出寄存器数据
	ULT_IIC_Stop_t();                    		//停止信号
	*distance=REG_data[0]<<8|REG_data[1];
	return SET;
}
u8 changeAddress_t(u8 oldAddress, u8 newAddress)
{
	
	ULT_IIC_Start_t(); 
    ULT_IIC_Send_Byte_t(oldAddress);   //发送设备地址+写信号
	ULT_IIC_Wait_Ack_t();
	delay_us(50);
	ULT_IIC_Send_Byte_t(0xaa);    			//指令
	ULT_IIC_Wait_Ack_t();
	ULT_IIC_Send_Byte_t(0xa5);    			//指令
	ULT_IIC_Wait_Ack_t();
	ULT_IIC_Send_Byte_t(newAddress);    //新地址
 	ULT_IIC_Wait_Ack_t();
	ULT_IIC_Stop_t();   	
	return SET;
}

u8 find_addr_t(u8 add)
{
	u8  ack;
	ULT_IIC_Start_t();          
    ULT_IIC_Send_Byte_t(add);   //发送设备地址+写信号
	ack=ULT_IIC_Wait_Ack_t();
	ULT_IIC_Stop_t();  
	return ack;
}
