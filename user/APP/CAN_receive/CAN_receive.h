/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       can_receive.c/h
  * @brief      ���can�豸�����շ����������ļ���ͨ��can�ж���ɽ���?
  * @note       ���ļ�����freeRTOS����
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���?
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
#define RESCUE_CAN  CAN2

#define GIMBAL_CAN  CAN1
#define LIFTER_CAN  CAN1
#define CLAW_CAN    CAN2

/* CAN send and receive ID */
typedef enum
{
  CAN_CHASSIS_ALL_ID = 0x200,
  CAN_3508_M1_ID = 0x201,  //CAN1
  CAN_3508_M2_ID = 0x202,
  CAN_3508_M3_ID = 0x203,
  CAN_3508_M4_ID = 0x204,

  CAN_GRAB_ALL_ID = 0x1FF,//CAN1
  CAN_LIFTER_M1_ID = 0x205,
  CAN_LIFTER_M2_ID = 0x206,




  CAN_RESCUE_ALL_ID = 0x1FF,//CAN2
  CAN_RESCUE_M1_ID = 0x205,
  CAN_RESCUE_M2_ID = 0x206,

  CAN_YAW_MOTOR_ID = 0x201,  //CAN2
  CAN_PIT_MOTOR_ID = 0x202,
  CAN_GIMBAL_ALL_ID = 0x200,

  CAN_CLAW_M1_ID = 0x203,    //CAN2
  CAN_CLAW_M2_ID = 0x204,

} can_msg_id_e;


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
//编码器范�? 
#define Half_ecd_range 4096
#define ecd_range 8191

extern void CAN_CMD_GIMBAL(int16_t yaw, int16_t pitch);

extern void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

extern void CAN_CMD_LIFTER(int16_t motor1, int16_t motor2);

extern void CAN_CMD_RESCUE(int16_t motor1, int16_t motor2);

extern void CAN_CMD_CLAW(int16_t motor1, int16_t motor2);

extern const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void);

extern const motor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void);

extern const motor_measure_t *get_Trigger_Motor_Measure_Point(void);

extern const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i);

extern const motor_measure_t *get_Lifter_Motor_Measure_Point(uint8_t i);

extern const motor_measure_t *get_Claw_Motor_Measure_Point(uint8_t i);

extern const motor_measure_t *get_Rescue_Motor_Measure_Point(uint8_t i);

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
extern void GIMBAL_lose_slove(void);
#endif

#endif
