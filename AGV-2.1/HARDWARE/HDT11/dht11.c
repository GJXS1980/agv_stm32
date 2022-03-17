#include "dht11.h"
#include "delay.h"
#include <string.h>

void DHT11_INIT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

}	

void GPIO_MODE(GPIOMode_TypeDef GPIO_Mode)
{
	
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
}	


void DHT11_Start(void)
{

	GPIO_MODE(GPIO_Mode_OUT);		//切换为输出模式
	PBout(10)=0;
	delay_ms(18);								//至少18ms
}	

int32_t DHT11_DATA(u8 *DATA)
{
	uint32_t t=0;
	int32_t i=0,j=0;
	uint8_t  d=0;
	
	PBout(10)=1;
	//20us~40us
	delay_us(30);
	
	//切换为输入模式
	GPIO_MODE(GPIO_Mode_IN);
	
	//等待低电平出现
	t=0;
	while(PBin(10))
	{
		t++;
		delay_us(1);
	
		if(t>=1000)
			return -1;
	}
	
	//检测低电平的持续时间
	t=0;	
	while(PBin(10)==0)
	{
		t++;
		delay_us(1);
	
		if(t>=1000)
			return -2;	
	
	}
	
	
	//检测高电平的持续时间
	t=0;	
	while(PBin(10)) //80us 
	{
		t++;
		delay_us(1);
	
		if(t>=1000)
			return -3;	
	
	}

	//完成5个字节接收
	for(j=0; j<5; j++)
	{
		d=0;
		
		//做好一个字节的接收
		for(i=7; i>=0; i--)
		{
			//等待50us低电平持续完毕
			t=0;	
			while(PBin(10)==0)
			{
				t++;
				delay_us(1);
			
				if(t>=1000)
					return -4;	
			
			}

			//延时40us~60us
			delay_us(40);
			
			
			//检测当前引脚的电平状态
			if(PBin(10))
			{
				//将d变量进行每个bit位赋值，最高有效位优先
				d|=1<<i;
				
				//等待高电平持续完毕
				t=0;	
				while(PBin(10)) 
				{
					t++;
					delay_us(1);
				
					if(t>=1000)
						return -5;	
				}			
			}
		}
		
		DATA[j] = d;
	
	}
	//检验和的判断
	if(DATA[4] !=((DATA[0]+DATA[1]+DATA[2]+DATA[3])&0x00FF))
	{
			memset((void*)DATA,0,4);
			return -6;	
	}
	
	return 0;
	
}


