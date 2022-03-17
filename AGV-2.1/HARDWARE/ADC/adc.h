#ifndef __ADC_H
#define __ADC_H	
#include "sys.h" 

//extern volatile u16 bat_vol;
extern  volatile uint16_t bat_vol[2];

void BAT_DMA_Init(void);
void BAT_ADC_Init(void);
void DMA_ADC_Count(u8 times);

void Adc_Init(void);
u16  Get_Adc(u8 ch);
u16 Get_Adc2(u8 ch);
float	 	Get_Adc_Average(u8 ch,u8 times);
float 	Get_Power_State(u8 ch,u8 times);

extern volatile u8 BAT_Check[2];

#endif 
