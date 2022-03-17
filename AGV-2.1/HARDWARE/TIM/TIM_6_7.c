#include "TIM_6_7.h"
#include "usart.h"
#include "sys.h"
#include "stdio.h"

void TIM_6_7_INIT(int arr,int psc)
{	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7 | RCC_APB1Periph_TIM6,ENABLE);			
	
	TIM_TimeBaseStructure.TIM_Prescaler = psc;																		//分频系数为8400-1	84M/8400 = 10000HZ		1/10000 = 0.0001S
	TIM_TimeBaseStructure.TIM_Period = arr;																				//预装载值为10-1  	10*0.0001S = 0.001S = 1ms
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM6,&TIM_TimeBaseStructure);
	
	TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);																			//时间溢出更新
	
	NVIC_InitStruct.NVIC_IRQChannel = TIM6_DAC_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 8;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
	TIM_OC2PreloadConfig(TIM6, TIM_OCPreload_Enable);  													//使能TIM6在CCR2上的预装载寄存器
	TIM_ARRPreloadConfig(TIM6,ENABLE);																						//ARPE使能 
	
	
	TIM_TimeBaseStructure.TIM_Prescaler = psc;																		//分频系数为8400-1	84M/8400 = 10000HZ		1/10000 = 0.0001S
	TIM_TimeBaseStructure.TIM_Period = arr + 200;																	//预装载值为10-1  	10*0.0001S = 0.001S = 1ms
	TIM_TimeBaseStructure.TIM_CounterMode = TIM7_IRQn;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM7,&TIM_TimeBaseStructure);
	
	TIM_ClearITPendingBit(TIM7,TIM_IT_Update);
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);																		//时间溢出更新
	
	NVIC_InitStruct.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 8;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
	//TIM_OC2PreloadConfig(TIM7, TIM_OCPreload_Enable);  											//使能TIM7在CCR2上的预装载寄存器
	TIM_ARRPreloadConfig(TIM7,ENABLE);																				//ARPE使能 
	
	TIM_Cmd(TIM6, ENABLE); 
	TIM_Cmd(TIM7, DISABLE); 																									//关闭TIM_6_7 
}

void TIM6_DAC_IRQHandler(void)
{

	if(TIM_GetITStatus(TIM6, TIM_IT_Update ) == SET)
	{
		TIM_ClearITPendingBit(TIM6,TIM_IT_Update);

	}
}

void TIM7_IRQHandler(void)
{

	if(TIM_GetITStatus(TIM7, TIM_IT_Update ) == SET)
	{
	
		TIM_ClearITPendingBit(TIM7,TIM_IT_Update);

	}
}


