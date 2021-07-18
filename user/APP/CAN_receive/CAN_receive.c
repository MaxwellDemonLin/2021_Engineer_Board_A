/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       can_receive.c/h
  * @brief      ���can�豸�����շ����������ļ���ͨ��can�ж���ɽ���?1?7
  * @note       ���ļ�����freeRTOS����
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���?1?7
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "CAN_Receive.h"

#include "stm32f4xx.h"
#include "rng.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "Detect_Task.h"
static void get_motor_measure_ecd (motor_measure_t *motor_measure,CanRxMsg *rx_message);

#define get_motor_measure(ptr, rx_message)                                                     \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);     \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]); \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
    }


#define get_gimbal_motor_measuer(ptr, rx_message)                                              \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]); \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]);     \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
    }


static void CAN2_hook(CanRxMsg *rx_message);
static void CAN1_hook(CanRxMsg *rx_message);

static motor_measure_t motor_yaw, motor_pit, motor_chassis[4],motor_lifter[2],motor_rescue[2],motor_claw[2];

static CanTxMsg GIMBAL_TxMessage;

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
static uint8_t delay_time = 100;
#endif

void CAN1_RX0_IRQHandler(void)
{
    static CanRxMsg rx1_message;

    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &rx1_message);
        CAN1_hook(&rx1_message);
    }
}


void CAN2_RX0_IRQHandler(void)
{
    static CanRxMsg rx2_message;
    if (CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
        CAN_Receive(CAN2, CAN_FIFO0, &rx2_message);
        CAN2_hook(&rx2_message);
    }
}

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
void GIMBAL_lose_slove(void)
{
        delay_time = RNG_get_random_range(13,239);
}
#endif

void CAN_CMD_GIMBAL(int16_t yaw, int16_t pitch)
{
    GIMBAL_TxMessage.StdId = 0x200;
    GIMBAL_TxMessage.IDE = CAN_ID_STD;
    GIMBAL_TxMessage.RTR = CAN_RTR_DATA;
    GIMBAL_TxMessage.DLC = 0x08;
    GIMBAL_TxMessage.Data[0] = (yaw >> 8);
    GIMBAL_TxMessage.Data[1] = yaw;
    GIMBAL_TxMessage.Data[2] = (pitch >> 8);
    GIMBAL_TxMessage.Data[3] = pitch;

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE

    TIM6->CNT = 0;
    TIM6->ARR = delay_time ;

    TIM_Cmd(TIM6,ENABLE);
#else
    CAN_Transmit( GIMBAL_CAN,  &GIMBAL_TxMessage );
#endif

}

void TIM6_DAC_IRQHandler(void)
{
    if( TIM_GetITStatus( TIM6, TIM_IT_Update )!= RESET )
    {

        TIM_ClearFlag( TIM6, TIM_IT_Update );
#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
        CAN_Transmit( GIMBAL_CAN,  &GIMBAL_TxMessage );
#endif
        TIM_Cmd(TIM6,DISABLE);
    }
}

void CAN_CMD_CHASSIS_RESET_ID(void)
{

    CanTxMsg TxMessage;
    TxMessage.StdId = 0x700;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = 0;
    TxMessage.Data[1] = 0;
    TxMessage.Data[2] = 0;
    TxMessage.Data[3] = 0;
    TxMessage.Data[4] = 0;
    TxMessage.Data[5] = 0;
    TxMessage.Data[6] = 0;
    TxMessage.Data[7] = 0;

    CAN_Transmit(CAN2, &TxMessage);
}

//���͵��̵����������?1?7
void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN_CHASSIS_ALL_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = motor1 >> 8;
    TxMessage.Data[1] = motor1;
    TxMessage.Data[2] = motor2 >> 8;
    TxMessage.Data[3] = motor2;
    TxMessage.Data[4] = motor3 >> 8;
    TxMessage.Data[5] = motor3;
    TxMessage.Data[6] = motor4 >> 8;
    TxMessage.Data[7] = motor4;

    CAN_Transmit(CHASSIS_CAN, &TxMessage);
}
//�����������ͺ���
void CAN_CMD_LIFTER_RESCUE(int16_t motor1, int16_t motor2, int16_t motor3 , int16_t motor4)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = 0x1FF;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = motor1 >> 8;
    TxMessage.Data[1] = motor1;
    TxMessage.Data[2] = motor2 >> 8;
    TxMessage.Data[3] = motor2;
		TxMessage.Data[4] = motor3>>8;
		TxMessage.Data[5] = motor3;
		TxMessage.Data[6] = motor4>>8;
		TxMessage.Data[7] = motor4;
    CAN_Transmit(CAN1, &TxMessage);
}
void CAN_CMD_CLAW(int16_t motor1, int16_t motor2)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = 0x1FF;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = motor1 >> 8;
    TxMessage.Data[1] = motor1;
    TxMessage.Data[2] = motor2 >> 8;
    TxMessage.Data[3] = motor2;
    CAN_Transmit(CAN2, &TxMessage);
}
//����yaw���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����?1?7
const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void)
{
    return &motor_yaw;
}
//����pitch���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����?1?7
const motor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void)
{
    return &motor_pit;
}
//���ص��̵��������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����?1?7
const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}
//�����������������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����?1?7
const motor_measure_t *get_Lifter_Motor_Measure_Point(uint8_t i)
{
    return &motor_lifter[(i)];
}
//���ؾ�Ԯ���������ַ��ͨ��ָ��ķ�ʽ��ȡԭʼ����
const motor_measure_t *get_Rescue_Motor_Measure_Point(uint8_t i)
{
    return &motor_rescue[(i)];
}
//���ػ�еצ���������ַ��ͨ��ָ��ķ�ʽ��ȡԭʼ����
const motor_measure_t *get_Claw_Motor_Measure_Point(uint8_t i)
{
    return &motor_claw[(i)];
}
//CAN1 ���մ������� 
static void CAN1_hook(CanRxMsg *rx_message)
{
    switch (rx_message->StdId)
    {
    case CAN_3508_M1_ID:
    case CAN_3508_M2_ID:
    case CAN_3508_M3_ID:
    case CAN_3508_M4_ID:
    {
        static uint8_t i = 0;
        i = rx_message->StdId - CAN_3508_M1_ID;
        get_motor_measure(&motor_chassis[i], rx_message);
        DetectHook(ChassisMotor1TOE + i);
        break;
    }

		case CAN_LIFTER_M1_ID:
    {
        get_motor_measure_ecd(&motor_lifter[0], rx_message);
        DetectHook(LifterMotor1TOE);
        break;
    }
    case CAN_LIFTER_M2_ID:
    {
        get_motor_measure_ecd(&motor_lifter[1], rx_message);
        DetectHook(LifterMotor2TOE);
        break;
    }
		case CAN_RESCUE_M1_ID:
		{
				get_motor_measure_ecd(&motor_rescue[0], rx_message);
        DetectHook(ClawMotor2TOE);
        break;
		}
		case CAN_RESCUE_M2_ID:
		{
				get_motor_measure_ecd(&motor_rescue[1], rx_message);
        DetectHook(ClawMotor2TOE);
        break;
		}
		
    default:
    {
        break;
    }
    }
}
//CAN2 ���մ������� 
static void CAN2_hook(CanRxMsg *rx_message)
{
    switch (rx_message->StdId)
    {
    case CAN_PIT_MOTOR_ID:
    {
        get_motor_measure_ecd(&motor_pit, rx_message);
        DetectHook(PitchGimbalMotorTOE);
        break;
    }
    case CAN_YAW_MOTOR_ID:
    {

        get_motor_measure_ecd(&motor_yaw, rx_message);
        DetectHook(YawGimbalMotorTOE);
        break;
    }

    case CAN_CLAW_M1_ID:
    {
        get_motor_measure_ecd(&motor_claw[0], rx_message);
        DetectHook(ClawMotor1TOE);
        break;
    }
    case CAN_CLAW_M2_ID:
    {
        get_motor_measure_ecd(&motor_claw[1], rx_message);
        DetectHook(ClawMotor2TOE);
        break;
    }
    default:
    {
        break;
    }
    }
}

void get_motor_measure_ecd (motor_measure_t *motor_measure,CanRxMsg *rx_message)
{
	motor_measure->last_ecd=motor_measure->ecd;
	motor_measure->ecd=(uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);
	motor_measure->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);
	motor_measure->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]);
	motor_measure->temperate = (rx_message)->Data[6];
	 if (motor_measure->ecd - motor_measure->last_ecd > Half_ecd_range)
	{
			motor_measure->count--;
	}
	else if (motor_measure->ecd - motor_measure->last_ecd < -Half_ecd_range)
	{
			motor_measure->count++;
	}
}