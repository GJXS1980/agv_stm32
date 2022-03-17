#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "IIC.h"
#include "IIC2.h"

int main()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);															//设置系统中断优先级分组4
	delay_init(168);																														//初始化延时函数
	uart_init(115200);  
	ULT_IIC_Init_t();
	ULT_IIC_Init();
	while(1)
	{
		changeAddress(0XE0,0X40);  //左超声波
		delay_ms(100);	
		
		changeAddress_t(0X40,0XE0); //右超声波
		delay_ms(100);	
					
	}

}


