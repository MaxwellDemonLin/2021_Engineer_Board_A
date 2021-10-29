#ifndef __CYLINDER_H
#define	__CYLINDER_H

#include "stm32f4xx.h"
#include "Remote_Control.h"
#include "claw_task.h"
#include "lifter.h"

#define Cylinder1_PIN                  GPIO_Pin_0                 
#define Cylinder1_GPIO_PORT            GPIOB                      
#define Cylinder1_GPIO_CLK             RCC_AHB1Periph_GPIOB

#define Cylinder2_PIN                  GPIO_Pin_1                
#define Cylinder2_GPIO_PORT            GPIOC                      
#define Cylinder2_GPIO_CLK             RCC_AHB1Periph_GPIOC

#define Cylinder3_PIN                  GPIO_Pin_2               
#define Cylinder3_GPIO_PORT            GPIOC                      
#define Cylinder3_GPIO_CLK             RCC_AHB1Periph_GPIOC

#define ON  0
#define OFF 1

#define	digitalHi(p,i)			 {p->BSRRL=i;}		//设置为高电平
#define digitalLo(p,i)			 {p->BSRRH=i;}		//输出低电平
#define digitalToggle(p,i)	 {p->ODR ^=i;}		//输出反转状态

#define Cylinder1_Open			digitalHi(Cylinder1_GPIO_PORT,Cylinder1_PIN)
#define Cylinder1_Close				digitalLo(Cylinder1_GPIO_PORT,Cylinder1_PIN)
#define Cylinder1_TOGGLE		digitalToggle(Cylinder1_GPIO_PORT,Cylinder1_PIN)


#define Cylinder2_Open			digitalHi(Cylinder2_GPIO_PORT,Cylinder2_PIN)
#define Cylinder2_Close				digitalLo(Cylinder2_GPIO_PORT,Cylinder2_PIN)
#define Cylinder2_TOGGLE		digitalToggle(Cylinder2_GPIO_PORT,Cylinder2_PIN)


#define Cylinder3_Open			digitalHi(Cylinder3_GPIO_PORT,Cylinder3_PIN)
#define Cylinder3_Close				digitalLo(Cylinder3_GPIO_PORT,Cylinder3_PIN)
#define Cylinder3_TOGGLE		digitalToggle(Cylinder3_GPIO_PORT,Cylinder3_PIN)

typedef struct
{
    uint8_t Cylinder1;
    uint8_t Cylinder2;
    uint8_t Cylinder3;
		const RC_ctrl_t *cylinder_rc_ctrl;
		const Claw_control_e *claw;
		const Lift_control_e *lift;
} Cylinder_switch_t;

void Cylinder_GPIO_Config(void);
void Cylinder_switch_control(void);
const Cylinder_switch_t *get_Cylinder_switch_Measure_Point(void);
void Cylinder_task(void *pvParameters);

#endif /* __CYLINDER_H */
