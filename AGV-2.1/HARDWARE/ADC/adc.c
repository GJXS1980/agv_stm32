#include "adc.h"
#include "delay.h"		 
#include "usart.h"

volatile uint16_t  bat_vol[2] = {0};
volatile u8 BAT_Check[2] = {0};

//***************************************方法1**********************************************

void  BAT_DMA_Init(void)
{    
	 DMA_InitTypeDef  DMA_InitStructure;
	
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);					//DMA2时钟使能 
			
	  while (DMA_GetCmdStatus(DMA2_Stream0) != DISABLE){}					//等待DMA可配置 
		
	  DMA_InitStructure.DMA_Channel = DMA_Channel_0;  						//通道选择
	  DMA_InitStructure.DMA_PeripheralBaseAddr = ((uint32_t)&ADC1->DR);		//DMA外设地址
	  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&bat_vol;			//DMA 存储器0地址
	  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;				//存储器到外设模式
	  DMA_InitStructure.DMA_BufferSize = 2;													//数据传输量 
	  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//外设非增量模式
	  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;				//存储器增量模式
	  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;						// 使用循环模式 
	  DMA_InitStructure.DMA_Priority = DMA_Priority_High;					//中等优先级
	  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;  
	  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;			//存储器突发单次传输
	  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;	//外设突发单次传输
	  DMA_Init(DMA2_Stream0, &DMA_InitStructure);							//初始化DMA Stream
		  
	  DMA_Cmd(DMA2_Stream0,ENABLE);	 
	
	  BAT_ADC_Init();		  
}


void BAT_ADC_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 , ENABLE);	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
		
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1复位
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//复位结束
	
	
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;											// 独立ADC模式
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;										// 时钟为fpclk x分频	84/4 = 21MHz
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;				// 禁止DMA直接访问模式	
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles; 	// 采样时间间隔	
	ADC_CommonInit(&ADC_CommonInitStructure);

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;												// ADC 分辨率
	ADC_InitStructure.ADC_ScanConvMode = ENABLE; 																	// 多通道采集	
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; 														// 连续转换	
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;		//禁止外部边沿触发
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;					//外部触发通道，本例子使用软件触发，此值随便赋值即可
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;												//数据右对齐	
	ADC_InitStructure.ADC_NbrOfConversion = 2;         														//转换通道 个                           
	ADC_Init(ADC1, &ADC_InitStructure);
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 1, ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 2, ADC_SampleTime_112Cycles); 	

	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);															// 使能DMA请求 after last transfer (Single-ADC mode)
	ADC_DMACmd(ADC1, ENABLE);																											// 使能ADC DM
	ADC_Cmd(ADC1, ENABLE);  																											// 使能ADC

	ADC_SoftwareStartConv(ADC1);	

}	

void DMA_ADC_Count(u8 times)
{
  float	BAT_ADC[2] = {0.0f};
		
		BAT_ADC[0] = bat_vol[0];
		BAT_ADC[1] = bat_vol[1];
	
		BAT_ADC[0] = BAT_ADC[0]*3.3f/4096;
		BAT_ADC[0] = BAT_ADC[0]*(200.0f +47.0f +10.0f)/10.0f;
		//printf("adc[0]: %f \r\n",BAT_ADC[0]); 
	
		if((BAT_ADC[0] - 45.0f) > 9.7f)		//将总电压分为4096份，占其中的多少份（尺子原理）
			BAT_ADC[0]=1.0f;
		else if((BAT_ADC[0] - 45.1f) < 0.0f)
			BAT_ADC[0]=0.0f;
		else 
			BAT_ADC[0]=(BAT_ADC[0] - 45.0f)/9.7f;

			BAT_Check[0] = BAT_ADC[0]*100; 
		
		BAT_ADC[1]=BAT_ADC[1]*3.33f/4096;
		//printf("adc[1]: %f \r\n",BAT_ADC[1]);

		BAT_ADC[1]>1.6f? (BAT_Check[1]=1):(BAT_Check[1]=0);
	
}	

//***************************************方法2**********************************************
void  Adc_Init(void)
{    
	GPIO_InitTypeDef  GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1|RCC_APB2Periph_ADC2, ENABLE);

	//读取电量 读取电池状态
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	//一个通道不准 原因不明
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1复位			
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//复位结束
	
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC2,ENABLE);	  //ADC2复位
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC2,DISABLE);	//复位结束
	
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
	ADC_CommonInit(&ADC_CommonInitStructure);
	
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; // 4096
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_Init(ADC1, &ADC_InitStructure);
	
	ADC_Init(ADC2, &ADC_InitStructure);
	
	ADC_Cmd(ADC1, ENABLE);//开启AD转换器	
	ADC_Cmd(ADC2, ENABLE);//开启AD转换器	
	
}				  

u16 Get_Adc(u8 ch)   
{
	  	
	 //设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles );	//ADC1,ADC通道,480个周期,提高采样时间可以提高精确度			    
  
	ADC_SoftwareStartConv(ADC1);		//使能指定的ADC1的软件转换启动功能	
	 	
	while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);  //等待ADC1转换完成
			
	ADC_ClearFlag(ADC1,ADC_FLAG_EOC);											//清空标志位

	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
}

u16 Get_Adc2(u8 ch)   
{
	  	
	 //设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(ADC2, ch, 1, ADC_SampleTime_480Cycles );	//ADC1,ADC通道,480个周期,提高采样时间可以提高精确度			    
  
	ADC_SoftwareStartConv(ADC2);		//使能指定的ADC1的软件转换启动功能	
	 	
	while(ADC_GetFlagStatus(ADC2,ADC_FLAG_EOC)==RESET);  //等待ADC1转换完成
			
	ADC_ClearFlag(ADC2,ADC_FLAG_EOC);											//清空标志位

	return ADC_GetConversionValue(ADC2);	//返回最近一次ADC1规则组的转换结果
	
}

float Get_Adc_Average(u8 ch,u8 times)
{
	u32 temp_val=0; u8 t;
	float	BAT_ADC = 0.0f;
	
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
	}
	temp_val=temp_val/times;
	
	BAT_ADC=temp_val*3.30f/4096;
	
	BAT_ADC = BAT_ADC*(200.0f +47.0f +10.0f)/10.0f;
	
/*
//	printf("BAT_ADC: %f \r\n",BAT_ADC[0]);
		
	 if((BAT_ADC[0] - 45.0f) > 9.7f)		//将总电压分为4096份，占其中的多少份（尺子原理）  54.7满电
		BAT_ADC[0]=1.0f;
	 else if((BAT_ADC[0] - 45.0f) < 0.0f)
		BAT_ADC[0]=0.0f;
	 else 
		BAT_ADC[0]=(BAT_ADC[0] - 45.0f)/9.7f;
*/	 
	
	return BAT_ADC;
} 

float Get_Power_State(u8 ch,u8 times)
{
	u8 t;
	float current = 0.0f;
	float  temp_val=0.0f;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc2(ch);
	}
	temp_val=temp_val/times;
	temp_val=temp_val*3.3f/4096;  //转换到出电流
 	
//     U1 = 2.5+ 0.1A 	
//     U2 = U1*4.7/(4.7+3)	= (2.5+0.1A) * 4.7/7.7
// --> (2.5 + 0.1A)  = U2*7.7/4.7
//        A = 	(U2*7.7/4.7 -2.5)*10
	
	current = (temp_val*7.7f/4.7f - 2.5f)*10.0f;
  if(current>10) current = 10; 	
	
	//printf("adc:%f  A:%f \r\n",temp_val,current);
	return current;
		
}



