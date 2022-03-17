#ifndef __DHT11_H
#define __DHT11_H	
#include "sys.h" 

void DHT11_INIT(void);
int32_t DHT11_DATA(u8 *temp);
void GPIO_MODE(GPIOMode_TypeDef GPIO_Mode);
void DHT11_Start(void);

#endif 

