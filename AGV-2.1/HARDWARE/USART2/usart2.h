#ifndef __USART2_H
#define __USART2_H	 
#include "sys.h" 

void Usart2_Init(unsigned int baud);
void Usart_SendString(USART_TypeDef *USARTx, unsigned char *str, unsigned short len);

void UsartPrintf(USART_TypeDef *USARTx, char *fmt,...);
uint8_t usart2_sendData(uint8_t *data, uint16_t len);
void Usart_SendString(USART_TypeDef *USARTx, unsigned char *str, unsigned short len);

void usart2_getRxData(uint8_t *buf, uint16_t len);
void Usart2_Clear(void);
extern unsigned char usart2_buf[128];
extern unsigned short usart2_cnt;
#endif

















