#include "stm32f4xx.h"
#include "tim9.h"
#include "iwdg.h"

void TIM9_INIT(void)
{
	// 4kHz
	u16 arr = 1500 - 1;
	u16 psc = 8400 - 1;
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);
	
	TIM_TimeBaseStructure.TIM_Prescaler = psc;					//��Ƶϵ��Ϊ8400-1	84M/8400 = 10000HZ		1/10000 = 0.0001S
	TIM_TimeBaseStructure.TIM_Period = arr;						//Ԥװ��ֵΪ10-1  	10*0.0001S = 0.001S = 1ms
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM9,&TIM_TimeBaseStructure);
	
	TIM_ITConfig(TIM9,TIM_IT_Update,ENABLE);					//�������
	
	NVIC_InitStruct.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 8;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
	TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Enable);  //ʹ��TIM9��CCR2�ϵ�Ԥװ�ؼĴ���
 
	TIM_ARRPreloadConfig(TIM9,ENABLE);//ARPEʹ�� 
	
	TIM_Cmd(TIM9, DISABLE);  //ʹ��TIM9 
}


void TIM1_BRK_TIM9_IRQHandler(void)
{

	if(TIM_GetITStatus(TIM9,TIM_IT_Update) == SET)
	{			
//		IWDG_Feed();
		
		TIM_ClearFlag(TIM9,TIM_IT_Update);
	}
	
}


