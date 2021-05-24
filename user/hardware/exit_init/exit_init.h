#ifndef EXIT_INIT_H
#define EXIT_INIT_H
#include "main.h"

extern void GPIOB_Exti8_GPIO_Init(void);
extern void EXTI_Key_Config(void);
//引脚定义
/*******************************************************/
#define KEY1_INT_GPIO_PORT                GPIOA
#define KEY1_INT_GPIO_CLK                 RCC_AHB1Periph_GPIOA
#define KEY1_INT_GPIO_PIN                 GPIO_Pin_4
#define KEY1_INT_EXTI_PORTSOURCE          EXTI_PortSourceGPIOA
#define KEY1_INT_EXTI_PINSOURCE           EXTI_PinSource4
#define KEY1_INT_EXTI_LINE                EXTI_Line4
#define KEY1_INT_EXTI_IRQ                 EXTI0_IRQn

#define KEY1_IRQHandler                   EXTI0_IRQHandler

#define KEY2_INT_GPIO_PORT                GPIOC
#define KEY2_INT_GPIO_CLK                 RCC_AHB1Periph_GPIOC
#define KEY2_INT_GPIO_PIN                 GPIO_Pin_13
#define KEY2_INT_EXTI_PORTSOURCE          EXTI_PortSourceGPIOC
#define KEY2_INT_EXTI_PINSOURCE           EXTI_PinSource13
#define KEY2_INT_EXTI_LINE                EXTI_Line13
#define KEY2_INT_EXTI_IRQ                 EXTI1_IRQn

#define KEY2_IRQHandler                   EXTI1_IRQHandler

/*******************************************************/
#endif
