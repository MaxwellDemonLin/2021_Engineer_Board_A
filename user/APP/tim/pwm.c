//
// Created by caesarw on 1/3/21.
//

#include "pwm.h"

uint8_t Run_Control_Task = 0;

void TIM2_Init(uint16_t period, uint16_t prescaler) {

    /* -------------- Configure GPIO_PA0 - PA3 (CH1 - CH4)  ---------------------------------------*/
    {
        GPIO_InitTypeDef GPIO_InitStructure;
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);         // PA0
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);         // PA1
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);         // PA2
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);         // PA3
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
    }

    {
        RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM2, ENABLE);
        RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM2, DISABLE);
    }

    {
        TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
        TIM_TimeBaseInitStructure.TIM_Period = period - 1;
        TIM_TimeBaseInitStructure.TIM_Prescaler = prescaler - 1;
        TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
        TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
        TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
    }

    /* -------------- Configure TIM5_CH0 - CH3 ---------------------------------------*/

    {
        TIM_OCInitTypeDef TIM_OCInitStructure;
        TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
        TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
        TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
        TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
        TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
        TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
        TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
        TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
        TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
        TIM_OCInitStructure.TIM_Pulse = 1000;
        TIM_OC1Init(TIM2, &TIM_OCInitStructure);
        TIM_OC2Init(TIM2, &TIM_OCInitStructure);
        TIM_OC3Init(TIM2, &TIM_OCInitStructure);
        TIM_OC4Init(TIM2, &TIM_OCInitStructure);
        TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
        TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
        TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
        TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
    }

    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_CtrlPWMOutputs(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

void TIM4_Init(uint16_t period, uint16_t prescaler) {

    /* -------------- Configure GPIO_PD12 - PD15 (CH1 - CH4)  ---------------------------------------*/
    {
        GPIO_InitTypeDef GPIO_InitStructure;
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
        GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);        // PD12
        GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);        // PD13
        GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);        // PD14
        GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);        // PD15
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_Init(GPIOD, &GPIO_InitStructure);
    }

    {
        RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM4, ENABLE);
        RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM4, DISABLE);
    }

    {
        TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
        TIM_TimeBaseInitStructure.TIM_Period = period - 1;
        TIM_TimeBaseInitStructure.TIM_Prescaler = prescaler - 1;
        TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
        TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
        TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);
    }

    /* -------------- Configure TIM4_CH1 - CH4 ---------------------------------------*/

    {
        TIM_OCInitTypeDef TIM_OCInitStructure;
        TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
        TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
        TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
        TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
        TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
        TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
        TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
        TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
        TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
        TIM_OCInitStructure.TIM_Pulse = 1000;
        TIM_OC1Init(TIM4, &TIM_OCInitStructure);
        TIM_OC2Init(TIM4, &TIM_OCInitStructure);
        TIM_OC3Init(TIM4, &TIM_OCInitStructure);
        TIM_OC4Init(TIM4, &TIM_OCInitStructure);
        TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
        TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
        TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
        TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
    }

    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_CtrlPWMOutputs(TIM4, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}

void TIM5_Init(uint16_t period, uint16_t prescaler) {

    /* -------------- Configure GPIO_PH10 - PH12 & GPIO_PI0 (CH1 - CH4)  ---------------------------------------*/
    {
        GPIO_InitTypeDef GPIO_InitStructure;
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
        GPIO_PinAFConfig(GPIOH, GPIO_PinSource10, GPIO_AF_TIM5);        // PH10
        GPIO_PinAFConfig(GPIOH, GPIO_PinSource11, GPIO_AF_TIM5);        // PH11
        GPIO_PinAFConfig(GPIOH, GPIO_PinSource12, GPIO_AF_TIM5);        // PH12
        GPIO_PinAFConfig(GPIOI, GPIO_PinSource0, GPIO_AF_TIM5);         // PI0
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_Init(GPIOH, &GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
        GPIO_Init(GPIOI, &GPIO_InitStructure);
    }

    {
        RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM5, ENABLE);
        RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM5, DISABLE);
    }

    {
        TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
        TIM_TimeBaseInitStructure.TIM_Period = period - 1;
        TIM_TimeBaseInitStructure.TIM_Prescaler = prescaler - 1;
        TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
        TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
        TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructure);
    }

    /* -------------- Configure TIM5_CH0 - CH3 ---------------------------------------*/

    {
        TIM_OCInitTypeDef TIM_OCInitStructure;
        TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);
        TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);
        TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);
        TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);
        TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
        TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
        TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
        TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
        TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
        TIM_OCInitStructure.TIM_Pulse = 1000;
        TIM_OC1Init(TIM5, &TIM_OCInitStructure);
        TIM_OC2Init(TIM5, &TIM_OCInitStructure);
        TIM_OC3Init(TIM5, &TIM_OCInitStructure);
        TIM_OC4Init(TIM5, &TIM_OCInitStructure);
        TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);
        TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);
        TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);
        TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);
    }

    TIM_ARRPreloadConfig(TIM5, ENABLE);
    TIM_CtrlPWMOutputs(TIM5, ENABLE);
    TIM_Cmd(TIM5, ENABLE);
}

void TIM8_Init(uint16_t period, uint16_t prescaler) {

    /* -------------- Configure GPIO_PI5 - PI7 & GPIO_PI2 (CH1 - CH4)  ---------------------------------------*/
    {
        GPIO_InitTypeDef GPIO_InitStructure;
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
        GPIO_PinAFConfig(GPIOI, GPIO_PinSource5, GPIO_AF_TIM8);         // PI5
        GPIO_PinAFConfig(GPIOI, GPIO_PinSource6, GPIO_AF_TIM8);         // PI6
        GPIO_PinAFConfig(GPIOI, GPIO_PinSource7, GPIO_AF_TIM8);         // PI7
        GPIO_PinAFConfig(GPIOI, GPIO_PinSource2, GPIO_AF_TIM8);         // PI2
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_2;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_Init(GPIOI, &GPIO_InitStructure);
    }

    {
        RCC_APB2PeriphResetCmd(RCC_APB2Periph_TIM8, ENABLE);
        RCC_APB2PeriphResetCmd(RCC_APB2Periph_TIM8, DISABLE);
    }

    {
        TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
        TIM_TimeBaseInitStructure.TIM_Period = period - 1;
        TIM_TimeBaseInitStructure.TIM_Prescaler = prescaler - 1;
        TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
        TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
        TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);
        TIM_TimeBaseInit(TIM8, &TIM_TimeBaseInitStructure);
    }

    /* -------------- Configure TIM5_CH0 - CH3 ---------------------------------------*/

    {
        TIM_OCInitTypeDef TIM_OCInitStructure;
        TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
        TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
        TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
        TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);
        TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
        TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
        TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
        TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
        TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
        TIM_OCInitStructure.TIM_Pulse = 1000;
        TIM_OC1Init(TIM8, &TIM_OCInitStructure);
        TIM_OC2Init(TIM8, &TIM_OCInitStructure);
        TIM_OC3Init(TIM8, &TIM_OCInitStructure);
        TIM_OC4Init(TIM8, &TIM_OCInitStructure);
        TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
        TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
        TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
        TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);
    }

    TIM_ARRPreloadConfig(TIM8, ENABLE);
    TIM_CtrlPWMOutputs(TIM8, ENABLE);
    TIM_Cmd(TIM8, ENABLE);
}

//
//void TIM3_Init(uint16_t arr, uint16_t psc)
//{
//    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
//    GPIO_InitTypeDef GPIO_InitStructure;
//    TIM_OCInitTypeDef TIM_OCInitStructure;

//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

//    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);

//    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM3, ENABLE);
//    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM3, DISABLE);

//    TIM_TimeBaseInitStructure.TIM_Period = arr - 1;
//    TIM_TimeBaseInitStructure.TIM_Prescaler = psc - 1;
//    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
//    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

//    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

//    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

//    /* TIM3 */
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
//    GPIO_Init(GPIOB, &GPIO_InitStructure);

//    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
//    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
//    TIM_OC2Init(TIM3, &TIM_OCInitStructure);

//    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

//    TIM_ARRPreloadConfig(TIM3, ENABLE);

//    TIM_Cmd(TIM3, ENABLE);
//}


void TIM7_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);		
    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn; 	
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;	 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE); 

  TIM_TimeBaseStructure.TIM_Period = 50-1;       
  TIM_TimeBaseStructure.TIM_Prescaler = 9000-1;	

	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
	
	TIM_ClearFlag(TIM7, TIM_FLAG_Update);

	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);

	TIM_Cmd(TIM7, ENABLE);	
}

void TIM7_IRQHandler(void)
{
	if ( TIM_GetITStatus( TIM7, TIM_IT_Update) != RESET ) 
	{
		TIM_ClearITPendingBit(TIM7 , TIM_IT_Update);
		Run_Control_Task = 1;
	}		 	
}
