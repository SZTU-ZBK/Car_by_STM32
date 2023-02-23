#ifndef _ENCODER_H
#define _ENCODER_H

#include "./SYSTEM/sys/sys.h"
void Encoder_Init_TIM3(void);
void Encoder_Init_TIM2(void);
void tim6_init(uint16_t arr, uint16_t psc);
void tim7_init(uint16_t arr, uint16_t psc);
#endif