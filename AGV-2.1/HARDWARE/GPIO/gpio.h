#ifndef __BEEP_H
#define __BEEP_H	 
#include "sys.h"
void Beep_Init(void);
void GPIO_control(void);

#define STOP GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)

#endif
