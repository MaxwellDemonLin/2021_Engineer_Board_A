/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       can_receive.c/h
  * @brief      ���can�豸�����շ����������ļ���ͨ��can�ж���ɽ���
  * @note       ���ļ�����freeRTOS����
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

#ifndef CANTASK_H
#define CANTASK_H
#include "main.h"

#define CHASSIS_CAN CAN1
#define RESCUE_CAN  CAN1

#define GIMBAL_CAN  CAN1
#define LIFTER_CAN  CAN1
#define CLAW_CAN    CAN1

/* CAN send and receive ID */
typedef enum
{
  CAN_CHASSIS_ALL_ID = 0x200,
  CAN_3508_M1_ID = 0x201,  //CAN1
  CAN_3508_M2_ID = 0x202,
  CAN_3508_M3_ID = 0x203,
  CAN_3508_M4_ID = 0x204,

  CAN_RESCUE_ALL_ID = 0x1FF,//CAN1
  CAN_RESCUE_M1_ID = 0x205,
  CAN_RESCUE_M2_ID = 0x206,

  CAN_YAW_MOTOR_ID = 0x205,  //CAN2
  CAN_PIT_MOTOR_ID = 0x206,
  CAN_GIMBAL_ALL_ID = 0x1FF,

  CAN_GRAB_ALL_ID = 0x1FF,//CAN1
  CAN_LIFTER_M1_ID = 0x205,
  CAN_LIFTER_M2_ID = 0x206,


  CAN_CLAW_M1_ID = 0x203,
  CAN_CLAW_M2_ID = 0x204,

} can_msg_id_e;

//rm���ͳһ���ݽṹ��
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
    int16_t count;
} motor_measure_t;

extern void CAN_CMD_CHASSIS_RESET_ID(void);
//编码器范围 
#define Half_ecd_range 4096
#define ecd_range 8191
//������̨�����������revΪ�����ֽ�
extern void CAN_CMD_GIMBAL(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);
//���͵��̵����������
extern void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
//�������������������
extern void CAN_CMD_LIFTER(int16_t motor1, int16_t motor2);
//����צ�ӵ���������� 
extern void CAN_CMD_RESCUE(int16_t motor1, int16_t motor2);
//���;�Ԯ�����������
extern void CAN_CMD_CLAW(int16_t motor1, int16_t motor2);
//����yaw���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void);
//����pitch���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const motor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void);
//����trigger���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const motor_measure_t *get_Trigger_Motor_Measure_Point(void);
//���ص��̵��������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����,i�ķ�Χ��0-3����Ӧ0x201-0x204,
extern const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i);
//�����������������ַ��ͨ��ָ��ķ�ʽ��ȡԭʼ��ַ��i�ķ�Χ��0-1����Ӧ0x201-0x202��
extern const motor_measure_t *get_Lifter_Motor_Measure_Point(uint8_t i);
//����צ�ӵ��������ַ��ͨ��ָ��ķ�ʽ��ȡԭʼ��ַ��i�ķ�Χ��0-1.��Ӧ0x203-0x204
extern const motor_measure_t *get_Claw_Motor_Measure_Point(uint8_t i);
//���ؾ�Ԯ���������ַ��ͨ��ָ��ķ�ʽ��ȡԭʼ��ַ��i�ķ�Χ��0-1����Ӧ0x205-0x206.
extern const motor_measure_t *get_Rescue_Motor_Measure_Point(uint8_t i);

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
extern void GIMBAL_lose_slove(void);
#endif

#endif
