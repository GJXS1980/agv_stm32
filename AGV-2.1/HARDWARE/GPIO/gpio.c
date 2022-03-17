#include "gpio.h"
/**************************************************************************
函数功能：蜂鸣器控制
入口参数：无
返回  值：无
**************************************************************************/
void Beep_Init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOE,&GPIO_InitStructure);
	
		GPIOE->BSRRH = GPIO_Pin_4;													//上电关闭
}

/**************************************************************************
函数功能：继电器控制 灯光控制
入口参数：无
返回  值：无
**************************************************************************/

void GPIO_control(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC,ENABLE);
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_6|GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_Init(GPIOC,&GPIO_InitStructure);

	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
		GPIOC->BSRRL = GPIO_Pin_7;														//上电立即吸合
		//GPIOC->BSRRL = GPIO_Pin_5;	
		GPIOC->BSRRH = GPIO_Pin_4|GPIO_Pin_6;	
	
}	



