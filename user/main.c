/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       main.c/h
  * @brief      stm32��ʼ���Լ���ʼ����freeRTOS��h�ļ��������ȫ�ֺ궨���Լ�
  *             typedef һЩ������������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#include "main.h"

#include "stm32f4xx.h"

#include "adc.h"
#include "buzzer.h"
#include "can.h"
#include "delay.h"
#include "flash.h"
#include "fric.h"
#include "laser.h"
#include "led.h"
#include "power_ctrl.h"
#include "rc.h"
#include "rng.h"
#include "sys.h"
#include "timer.h"
#include "exit_init.h"
#include "Cylinder_switch.h"   
#include "pwm.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "CAN_Receive.h"
#include "remote_control.h"
#include "start_task.h"

void BSP_init(void);

int main(void)
{
   BSP_init();
    delay_ms(100);
    startTask();
    vTaskStartScheduler();
		
    while (1)
    {

    }
}

//�ĸ�24v ��� ���ο��� ��� 709us
#define POWER_CTRL_ONE_BY_ONE_TIME 709

void BSP_init(void)
{
    //�ж��� 4
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    //��ʼ���δ�ʱ��
    delay_init(configTICK_RATE_HZ);
    //��ˮ�ƣ����̵Ƴ�ʼ��
    led_configuration();
    //stm32 �����¶ȴ�������ʼ��
    temperature_ADC_init();
#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
    //stm32 �������������ʼ��
    RNG_init();
#endif
    //24������ƿ� ��ʼ��
    power_ctrl_configuration();
    //Ħ���ֵ��PWM��ʼ��
    fric_PWM_configuration();
    //��������ʼ��
//    buzzer_init(30000, 90);
    //����IO��ʼ��
    laser_configuration();
		Cylinder_GPIO_Config();
    //��ʱ��6 ��ʼ��
    TIM6_Init(60000, 90);
	 	TIM2_Init(20000, 90);
		TIM_SetCompare4(TIM2, 2400);
//		EXTI_Key_Config();
    //CAN�ӿڳ�ʼ��
    CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
    CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);

    //24v ��� �����ϵ�
    for (uint8_t i = POWER1_CTRL_SWITCH; i < POWER4_CTRL_SWITCH + 1; i++)
    {
        power_ctrl_on(i);
        delay_us(POWER_CTRL_ONE_BY_ONE_TIME);
    }
    //ң������ʼ��
    remote_control_init();
}
