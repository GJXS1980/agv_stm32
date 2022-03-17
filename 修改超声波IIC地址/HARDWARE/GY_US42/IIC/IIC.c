#include "IIC.h"
#include "delay.h"

//初始化IIC
void ULT_IIC_Init(void)
{					     
	RCC->AHB1ENR|=1<<1;    //使能PORTH时钟	   	  
	GPIO_Set(GPIOB,PIN8|PIN9,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);
	ULT_IIC_SCL=1;
	ULT_IIC_SDA=1;
}
//产生IIC起始信号
void ULT_IIC_Start(void)
{
	ULT_SDA_OUT();     //sda线输出
	ULT_IIC_SDA=1;	  	  
	ULT_IIC_SCL=1;
	delay_us(5);
 	ULT_IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(5);
	ULT_IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void ULT_IIC_Stop(void)
{
	ULT_SDA_OUT();//sda线输出
	ULT_IIC_SCL=0;
	ULT_IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(5);
	ULT_IIC_SCL=1; 
	ULT_IIC_SDA=1;//发送I2C总线结束信号
	delay_us(5);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 ULT_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	ULT_SDA_IN();      //SDA设置为输入  
	ULT_IIC_SDA=1;	   
	ULT_IIC_SCL=1;	
	delay_us(5);
	
	while(ULT_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>200)
		{
			ULT_IIC_Stop();
			return 1;
		}
	
	}
	delay_us(5);	
	ULT_IIC_SCL=0;//时钟输出0 
	delay_us(5);		

	return 0;  
} 
//产生ACK应答
void ULT_IIC_Ack(void)
{
	ULT_IIC_SCL=0;
	ULT_SDA_OUT();
	ULT_IIC_SDA=0;
	delay_us(5);
	ULT_IIC_SCL=1;
	delay_us(5);
	ULT_IIC_SCL=0;
}
//不产生ACK应答		    
void ULT_IIC_NAck(void)
{
	ULT_IIC_SCL=0;
	ULT_SDA_OUT();
	ULT_IIC_SDA=1;
	delay_us(5);
	ULT_IIC_SCL=1;
	delay_us(5);
	ULT_IIC_SCL=0;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void ULT_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	ULT_SDA_OUT(); 	    
    ULT_IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        ULT_IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(5);   						
		ULT_IIC_SCL=1;
		delay_us(5); 
		ULT_IIC_SCL=0;	
		delay_us(5);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 ULT_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	ULT_SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        ULT_IIC_SCL=0; 
        delay_us(5);
		ULT_IIC_SCL=1;
        receive<<=1;
        if(ULT_READ_SDA)receive++;   
			delay_us(5); 
    }					 
    if (!ack)
        ULT_IIC_NAck();//发送nACK
    else
        ULT_IIC_Ack(); //发送ACK   
    return receive;
}

u8 takeRangeReading(u8 Slave_Address)
{
	ULT_IIC_Start();       
    ULT_IIC_Send_Byte(Slave_Address);   //发送设备地址+写信号
 	ULT_IIC_Wait_Ack();  
	ULT_IIC_Send_Byte(0x51);    				//内部寄存器地址，
 	ULT_IIC_Wait_Ack();
	ULT_IIC_Stop();   	
	return SET;
}

u8 requestRange(u8 Slave_Address,uint16_t *distance)
{
	u8 REG_data[2]={0,0};
	ULT_IIC_Start();        
	ULT_IIC_Send_Byte(Slave_Address);    		//发送设备地址+写信号
	ULT_IIC_Wait_Ack(); 
  	delay_us(5);  
	REG_data[0]=ULT_IIC_Read_Byte(1);       	//读出寄存器数据
	REG_data[1]=ULT_IIC_Read_Byte(0);       	//读出寄存器数据
	ULT_IIC_Stop();                    		//停止信号
	*distance=REG_data[0]<<8|REG_data[1];
	return SET;
}
u8 changeAddress(u8 oldAddress, u8 newAddress)
{
	ULT_IIC_Start(); 
    ULT_IIC_Send_Byte(oldAddress);   //发送设备地址+写信号
	ULT_IIC_Wait_Ack();
	delay_us(50);
	ULT_IIC_Send_Byte(0xaa);    			//指令
	ULT_IIC_Wait_Ack();
	ULT_IIC_Send_Byte(0xa5);    			//指令
	ULT_IIC_Wait_Ack();
	ULT_IIC_Send_Byte(newAddress);    //新地址
 	ULT_IIC_Wait_Ack();
	ULT_IIC_Stop();   	
	return SET;
}

u8 find_addr(u8 add)
{
	u8  ack;
	ULT_IIC_Start();          
    ULT_IIC_Send_Byte(add);   //发送设备地址+写信号
	ack=ULT_IIC_Wait_Ack();
	ULT_IIC_Stop();  
	return ack;
}
