#include "infrared.h"
#include "usart.h"
#include "string.h"
#include "outputdata.h"

volatile u8 	left_ir_data[3]={0};
volatile u8 	right_ir_data[3]={0};
volatile u32 	Dval[2]={0};

void TIM_6_INIT(u32 arr,u32 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	//	NVIC_InitTypeDef NVIC_InitStruct;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);			
	TIM_TimeBaseStructure.TIM_Prescaler = psc;		//分频系数为8400-1	84M/8400 = 10000HZ		1/10000 = 0.0001S
	TIM_TimeBaseStructure.TIM_Period = arr;			//预装载值为10-1  	10*0.0001S = 0.001S = 1ms
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM6,&TIM_TimeBaseStructure);
	TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
	TIM_ITConfig(TIM6,TIM_IT_Update,DISABLE);		//不触发中断
	
//	NVIC_InitStruct.NVIC_IRQChannel = TIM6_DAC_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 8;
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStruct);
	
	TIM_OC2PreloadConfig(TIM6, TIM_OCPreload_Enable);  	//使能TIM6在CCR2上的预装载寄存器
	TIM_ARRPreloadConfig(TIM6,ENABLE);					//ARPE使能 
	TIM_Cmd(TIM6, ENABLE); 

}	

void TIM_7_INIT(u32 arr,u32 psc)
{

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);			
	
	TIM_TimeBaseStructure.TIM_Prescaler = psc;	//分频系数为8400-1	84M/8400 = 10000HZ		1/10000 = 0.0001S
	TIM_TimeBaseStructure.TIM_Period = arr;		//预装载值为10-1  	10*0.0001S = 0.001S = 1ms
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM7,&TIM_TimeBaseStructure);
	
	TIM_ClearITPendingBit(TIM7,TIM_IT_Update);
	TIM_ITConfig(TIM7,TIM_IT_Update,DISABLE);	//不触发中断
	
	TIM_OC2PreloadConfig(TIM7, TIM_OCPreload_Enable);  //使能TIM7在CCR2上的预装载寄存器
	TIM_ARRPreloadConfig(TIM7,ENABLE);		//ARPE使能 
	
	TIM_Cmd(TIM7, ENABLE);
}	

void IR_Init(u32 arr,u16 psc)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM3_ICInitStructure;
			
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 
			
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
			
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3); 
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM3); 
			
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  	
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
	TIM_TimeBaseStructure.TIM_Period=arr;   		
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure); 
					
	//初始化TIM3输入捕获参数
	TIM3_ICInitStructure.TIM_Channel = TIM_Channel_1; 
	TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;		//	上升沿捕获
	TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 	//	映射到TI1上
	TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 	//	配置输入分频,不分频 
	TIM3_ICInitStructure.TIM_ICFilter = 0x03;	//	IC1F=0003 8个定时器时钟周期滤波
	TIM_ICInit(TIM3, &TIM3_ICInitStructure);
			
	TIM3_ICInitStructure.TIM_Channel = TIM_Channel_2; 
	TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//	上升沿捕获
	TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //	映射到TI1上
	TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //	配置输入分频,不分频 
	TIM3_ICInitStructure.TIM_ICFilter = 0x03;	//	IC1F=0003 8个定时器时钟周期滤波
	TIM_ICInit(TIM3, &TIM3_ICInitStructure);
			
	TIM_ITConfig(TIM3,TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2,ENABLE);	//	允许更新中断 ,允许CC1IE捕获中断	
	TIM_Cmd(TIM3,ENABLE ); 	 	//	使能定时器3
			
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=6;	//	抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		//	子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//	IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//	初始化NVIC寄存器
}

static int index_l = -1;
static int index_r = -1;

void TIM3_IRQHandler(void)			
{
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{		
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //	清除中断标志位
	}	
	
	if(TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
	{
		if(PAin(6))
		{
			TIM_OC1PolarityConfig(TIM3,TIM_ICPolarity_Falling);	//	CC1P=1 设置为下降沿捕获
			TIM6->CNT =0;   	//	清空定时器值				
		}	
		else 
		{
			Dval[0]=TIM6->CNT;							
			TIM_OC1PolarityConfig(TIM3,TIM_ICPolarity_Rising); //	CC1P=0	设置为上升沿捕获					
			//printf("%d \r\n",Dval[0]);
					
			if((Dval[0]>4400)&&(Dval[0]<4900))	//	起始标志
			{
				//printf("data[%d]:%d\r\n",index_l,left_ir_data[index_l]);
				index_l++;
				if(index_l>2)
					index_l = 0;
				left_ir_data[index_l] = 0;
			}	
			else if(( Dval[0]>350 )&& (Dval[0]<850))				
			{
				left_ir_data[index_l]<<=1;		
				left_ir_data[index_l]|=0;														
			}
			else if((Dval[0]>1400) && (Dval[0]<1850 ))			
			{
				left_ir_data[index_l]<<=1;	
				left_ir_data[index_l]|=1;																								
			}					
		}
			
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1); //清除中断标志位
	}
		
	if(TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)
	{
		if(PAin(7))
		{
			TIM_OC2PolarityConfig(TIM3,TIM_ICPolarity_Falling);	//CC1P=1 设置为下降沿捕获
			TIM7->CNT =0;   //清空定时器值			
		}	
		else 
		{
			Dval[1]=TIM7->CNT;							
			TIM_OC2PolarityConfig(TIM3,TIM_ICPolarity_Rising); //CC1P=0	设置为上升沿捕获
			//printf("%d \r\n",Dval[1]);	
			if((Dval[1]>4400)&&(Dval[1]<4900))	//起始标志
			{
				index_r++;	    //循环数组
				if(index_r>2)	
					index_r = 0;
				right_ir_data[index_r] = 0;									
			}	
			else if(Dval[1]>350&&Dval[1]<850)				
			{
				right_ir_data[index_r]<<=1;		
				right_ir_data[index_r]|=0;														
			}
			else if(Dval[1]>1400&&Dval[1]<1850)			
			{
				right_ir_data[index_r]<<=1;	
				right_ir_data[index_r]|=1;																								
			}
						
		}
			
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC2); //清除中断标志位
	}		
		
}

u8 get_ir_data(u8 ir_num)
{
	u8 IR_TEMP[2]={0};
	u8 i = 0;
	u8 value = 0;

	if(ir_num == 0)
	{
		for(i=0;i<3;i++)
		{
			//printf("left[%d]:%d\r\n",i,left_ir_data[i]);
			if(left_ir_data[i] == 2 )  
				IR_TEMP[0]+=1; 	//	左
			else if(left_ir_data[i] == 80 || left_ir_data[i] == 160)
				IR_TEMP[1]+=1;	//	右
		}	

		//	只收到左红外数据，为1
		if(IR_TEMP[0] && !IR_TEMP[1])
		{
			value = 1;
		}	
		//	只收到右红外数据，为2
		else if(IR_TEMP[1] && !IR_TEMP[0])
		{
			value = 2;
		}	
		//	同时收到左右红外数据，为3
		else if(IR_TEMP[0] && IR_TEMP[1])
		{		
			value = 3;
		}	
		memset((void*)left_ir_data,0,3);	
		return value;						
	}	
	// 右边红外数据	
	else if(ir_num == 1)
	{	
		for(i=0;i<3;i++)
		{
			//printf("right[%d]:%d\r\n",i,right_ir_data[i]);
			if(right_ir_data[i] == 2)
				IR_TEMP[0]+=1;	//	左
			else if(right_ir_data[i] == 80 || right_ir_data[i] == 160)
				IR_TEMP[1]+=1;	//	右
		}
		//	只收到左红外数据，为1
		if(IR_TEMP[0] && !IR_TEMP[1])
		{
			value = 1;
		}	
		//	只收到右红外数据，为2
		else if(IR_TEMP[1] && !IR_TEMP[0])
		{
			value = 2;
		}	
		//	同时收到左右红外数据，为3
		else if(IR_TEMP[0] && IR_TEMP[1])
		{		
			value = 3;
		}	
		//	复位right_ir_data=[0, 0, 0]
		memset((void*)right_ir_data,0,3);
				
		return value;	
	}

	return 0;
}	

