#include "urf02.h"
#include "usart.h"
#include "delay.h"

#define ARR 25000-1
#define PSC 84-1

#define URF1_MCU_IN GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)
#define URF1_MCU_OUT_HIGH GPIO_SetBits(GPIOB,GPIO_Pin_0)
#define URF1_MCU_OUT_LOW GPIO_ResetBits(GPIOB,GPIO_Pin_0)

volatile u8 counting = 0;
volatile u8 over_time= 0;
volatile float URF_DIST = 0;

void EXTI1_IRQHandler(void)
{
  // urf1
  if (EXTI_GetITStatus(EXTI_Line1))
  {
    if (URF1_MCU_IN)
		{
      TIM_Cmd(TIM12, ENABLE);
      counting = 1;
    }
    else if ((counting == 1) && (URF1_MCU_IN == 0)) 
		{
      TIM_Cmd(TIM12, DISABLE);
      counting = 0;
      if (over_time) 
			{
        //printf( ">5m\r\n" );
        URF_DIST = 500.0f;
        over_time = 0;
      }
      else
      {
        URF_DIST = (float)TIM_GetCounter(TIM12)/1000 * 340/20;  //单位 CM
      }
    }
    else
    {
      TIM_Cmd(TIM12, DISABLE);
      counting = 0;
      over_time = 0;
    }
    EXTI_ClearITPendingBit(EXTI_Line1);
  }
  
}

// ufr1 timer
void TIM12_Init( void )
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
	TIM_TimeBaseStructure.TIM_Prescaler = PSC; // 时钟预分频数为 84-1，则驱动计数器的时钟(84M) CK_INT / (84)=1M
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; // 向上计数
	TIM_TimeBaseStructure.TIM_Period = ARR; // freq = psc / arr
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; // 时钟不分频
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; // 重复计数设置 
	TIM_TimeBaseInit(TIM12,&TIM_TimeBaseStructure);
  TIM_ClearFlag(TIM12, TIM_FLAG_Update); // 清中断标志位
  TIM_ARRPreloadConfig( TIM12,ENABLE );
	TIM_Cmd( TIM12, DISABLE );
  TIM_SetCounter(TIM12, 0);
}

void TIM12_IRQ_Config(void) 
{
	TIM_ITConfig( TIM12, TIM_IT_Update, ENABLE );
}

void TIM12_NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM8_BRK_TIM12_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NVIC_InitStructure );
}

void TIM8_BRK_TIM12_IRQHandler (void)
{
  if ( TIM_GetITStatus( TIM12, TIM_IT_Update ) != RESET ) 
	{
    over_time = 1;
    TIM_ClearITPendingBit( TIM12, TIM_IT_Update );
  }
}

void URF_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	// 传感器 trig
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	URF1_MCU_OUT_LOW;

	// 传感器 echo
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource1);


	EXTI_InitStructure.EXTI_Line = EXTI_Line1;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; 				//任意边沿电平触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
 
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM12_Init();
	TIM12_NVIC_Config();
	TIM12_IRQ_Config();

}

void Read_URF(void)
{
      TIM_Cmd( TIM12, DISABLE );
      TIM_ClearFlag(TIM12, TIM_FLAG_Update);
      TIM_SetCounter(TIM12, 0);
      URF1_MCU_OUT_HIGH;
      delay_us(10);
      URF1_MCU_OUT_LOW;
}



