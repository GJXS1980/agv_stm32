#ifndef __INFRARED_H
#define __INFRARED_H	 
#include "sys.h"
#include <stdio.h>

void IR_Init(u32 arr,u16 psc);
void TIM_6_INIT(u32 arr,u32 psc);
void TIM_7_INIT(u32 arr,u32 psc);
extern volatile u8 Infrared[2]; 
extern volatile u8 	left_ir_data[3];
extern volatile u8 	right_ir_data[3];
extern u8 get_ir_data(u8 ir_num);

#endif
