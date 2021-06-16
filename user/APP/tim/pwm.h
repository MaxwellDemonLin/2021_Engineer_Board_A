#ifndef PWM_H
#define PWM_H
#include "stm32f4xx.h"

void TIM2_Init(uint16_t period, uint16_t prescaler);
void TIM4_Init(uint16_t period, uint16_t prescaler);
void TIM5_Init(uint16_t period, uint16_t prescaler);
void TIM8_Init(uint16_t period, uint16_t prescaler);
//void TIM3_Init(uint16_t arr, uint16_t psc);
void TIM7_Init(void);

#endif
