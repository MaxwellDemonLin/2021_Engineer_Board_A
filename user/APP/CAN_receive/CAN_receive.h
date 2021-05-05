/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       can_receive.c/h
  * @brief      完成can设备数据收发函数，该文件是通过can中断完成接收
  * @note       该文件不是freeRTOS任务
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
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

#define CHASSIS_CAN CAN2
#define RESCUE_CAN  CAN2

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
  CAN_TRIGGER_MOTOR_ID = 0x207,
  CAN_GIMBAL_ALL_ID = 0x1FF,

  CAN_GRAB_ALL_ID = 0x200,//CAN2
  CAN_LIFTER_M1_ID = 0x201,
  CAN_LIFTER_M2_ID = 0x202,
  CAN_CLAW_M1_ID = 0x203,
  CAN_CLAW_M2_ID = 0x204,

} can_msg_id_e;

//rm电机统一数据结构体
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

extern void CAN_CMD_CHASSIS_RESET_ID(void);

//发送云台控制命令，其中rev为保留字节
extern void CAN_CMD_GIMBAL(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);
//发送底盘电机控制命令
extern void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
//发送升降电机控制命令
extern void CAN_CMD_LIFTER(int16_t motor1, int16_t motor2);
//发送爪子电机控制命令 
extern void CAN_CMD_RESCUE(int16_t motor1, int16_t motor2);
//发送救援电机控制命令
extern void CAN_CMD_CLAW(int16_t motor1, int16_t motor2);
//返回yaw电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void);
//返回pitch电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void);
//返回trigger电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Trigger_Motor_Measure_Point(void);
//返回底盘电机变量地址，通过指针方式获取原始数据,i的范围是0-3，对应0x201-0x204,
extern const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i);
//返回升降电机变量地址，通过指针的方式获取原始地址，i的范围是0-1，对应0x201-0x202，
extern const motor_measure_t *get_Lifter_Motor_Measure_Point(uint8_t i);
//返回爪子电机变量地址，通过指针的方式获取原始地址，i的范围是0-1.对应0x203-0x204
extern const motor_measure_t *get_Claw_Motor_Measure_Point(uint8_t i);
//返回救援电机变量地址，通过指针的方式获取原始地址，i的范围是0-1，对应0x205-0x206.
extern const motor_measure_t *get_Rescue_Motor_Measure_Point(uint8_t i);

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
extern void GIMBAL_lose_slove(void);
#endif

#endif
