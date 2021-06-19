#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "CAN_receive.h"
#include "main.h"
#include "remote_control.h"

#include "pid.h"


#define MIN_YAW_ECD_OFFSET   30000
#define MAX_YAW_ECD_OFFSET   30000

#define MIN_PITCH_ECD_OFFSET 30000
#define MAX_PITCH_ECD_OFFSET 30000

typedef struct
{
    const motor_measure_t *gimbal_motor_measure;
    PidTypeDef gimbal_ecd_pid;
    PidTypeDef gimbal_speed_pid;
    int32_t ecd_sum;
    int32_t ecd_sum_set;
    int32_t mid_ecd;
} Gimbal_Motor_t;
typedef struct
{
    const RC_ctrl_t *gimbal_rc_ctrl;
    Gimbal_Motor_t gimbal_yaw_motor;
    Gimbal_Motor_t gimbal_pitch_motor;
    uint8_t cali_flag;

} Gimbal_Control_t;


#endif