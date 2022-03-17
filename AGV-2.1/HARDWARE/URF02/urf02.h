#ifndef __URF02_H
#define __URF02_H
#include "sys.h"
void URF_Init( void );
void Read_URF(void);
extern volatile u8 counting;
extern volatile u8 over_time;
extern volatile float URF_DIST;
#endif
