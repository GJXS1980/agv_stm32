#include "IIC2.h"
#include "delay.h"

//��ʼ��IIC
void ULT_IIC_Init_t(void)
{					     
	RCC->AHB1ENR|=1<<1;    //ʹ��PORTHʱ��	   	  
	GPIO_Set(GPIOB,PIN4|PIN5,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);
	ULT_IIC_SCL_T=1;
	ULT_IIC_SDA_T=1;
}
//����IIC��ʼ�ź�
void ULT_IIC_Start_t(void)
{
	ULT_SDA_OUT_T();     //sda�����
	ULT_IIC_SDA_T=1;	  	  
	ULT_IIC_SCL_T=1;
	delay_us(5);
 	ULT_IIC_SDA_T=0;//START:when CLK is high,DATA change form high to low 
	delay_us(5);
	ULT_IIC_SCL_T=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void ULT_IIC_Stop_t(void)
{
	ULT_SDA_OUT_T();//sda�����
	ULT_IIC_SCL_T=0;
	ULT_IIC_SDA_T=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(5);
	ULT_IIC_SCL_T=1; 
	ULT_IIC_SDA_T=1;//����I2C���߽����ź�
	delay_us(5);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 ULT_IIC_Wait_Ack_t(void)
{
	u8 ucErrTime=0;
	ULT_SDA_IN_T();      //SDA����Ϊ����  
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
	ULT_IIC_SCL_T=0;//ʱ�����0 
	delay_us(5);		

	return 0;  
} 
//����ACKӦ��
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
//������ACKӦ��		    
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
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void ULT_IIC_Send_Byte_t(u8 txd)
{                        
    u8 t;   
	ULT_SDA_OUT_T(); 	    
    ULT_IIC_SCL_T=0;//����ʱ�ӿ�ʼ���ݴ���
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
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 ULT_IIC_Read_Byte_t(unsigned char ack)
{
	unsigned char i,receive=0;
	ULT_SDA_IN_T();//SDA����Ϊ����
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
        ULT_IIC_NAck_t();//����nACK
    else
        ULT_IIC_Ack_t(); //����ACK   
    return receive;
}

u8 takeRangeReading_t(u8 Slave_Address)
{
	ULT_IIC_Start_t();       
    ULT_IIC_Send_Byte_t(Slave_Address);   //�����豸��ַ+д�ź�
 	ULT_IIC_Wait_Ack_t();  
	ULT_IIC_Send_Byte_t(0x51);    				//�ڲ��Ĵ�����ַ��
 	ULT_IIC_Wait_Ack_t();
	ULT_IIC_Stop_t();   	
	return SET;
}

u8 requestRange_t(u8 Slave_Address,uint16_t *distance)
{
	u8 REG_data[2]={0,0};
	ULT_IIC_Start_t();        
	ULT_IIC_Send_Byte_t(Slave_Address);    		//�����豸��ַ+д�ź�
	ULT_IIC_Wait_Ack_t(); 
  	delay_us(5);  
	REG_data[0]=ULT_IIC_Read_Byte_t(1);       	//�����Ĵ�������
	REG_data[1]=ULT_IIC_Read_Byte_t(0);       	//�����Ĵ�������
	ULT_IIC_Stop_t();                    		//ֹͣ�ź�
	*distance=REG_data[0]<<8|REG_data[1];
	return SET;
}
u8 changeAddress_t(u8 oldAddress, u8 newAddress)
{
	
	ULT_IIC_Start_t(); 
    ULT_IIC_Send_Byte_t(oldAddress);   //�����豸��ַ+д�ź�
	ULT_IIC_Wait_Ack_t();
	delay_us(50);
	ULT_IIC_Send_Byte_t(0xaa);    			//ָ��
	ULT_IIC_Wait_Ack_t();
	ULT_IIC_Send_Byte_t(0xa5);    			//ָ��
	ULT_IIC_Wait_Ack_t();
	ULT_IIC_Send_Byte_t(newAddress);    //�µ�ַ
 	ULT_IIC_Wait_Ack_t();
	ULT_IIC_Stop_t();   	
	return SET;
}

u8 find_addr_t(u8 add)
{
	u8  ack;
	ULT_IIC_Start_t();          
    ULT_IIC_Send_Byte_t(add);   //�����豸��ַ+д�ź�
	ack=ULT_IIC_Wait_Ack_t();
	ULT_IIC_Stop_t();  
	return ack;
}
