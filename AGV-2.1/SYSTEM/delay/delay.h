#ifndef __DELAY_H
#define __DELAY_H 			   
#include <sys.h>	  
u32 getSysTickCnt(void);	
void delay_init(u8 SYSCLK);
void delay_us(u32 nus);
void delay_ms(u32 nms);
void delay_xms(u32 nms);

#define CONTROL_DELAY		1000 //换算成实际时间是10秒
#define RATE_1_HZ		  1
#define RATE_2_HZ		  2
#define RATE_5_HZ		  5
#define RATE_10_HZ		10
#define RATE_20_HZ		20
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_200_HZ 	200
#define RATE_300_HZ 	300
#define RATE_500_HZ 	500
#define RATE_600_HZ 	600
#define RATE_700_HZ 	700
#define RATE_800_HZ 	800
#define RATE_900_HZ 	900
#define RATE_1000_HZ 	1000
#endif





























