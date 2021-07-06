#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "CAN_receive.h"
#include "main.h"
#include "remote_control.h"

#include "pid.h"



#define GIMBAL_ECD_KP 0.03
#define GIMBAL_ECD_KI 0.0
#define GIMBAL_ECD_KD 0
#define GIMBAL_ECD_MAX_OUT 1800
#define GIMBAL_ECD_MAX_IOUT 5


#define GIMBAL_SPEED_KP 15
#define GIMBAL_SPEED_KI 0
#define GIMBAL_SPEED_KD 0
#define GIMBAL_SPEED_MAX_OUT 10000
#define GIMBAL_SPEED_MAX_IOUT 0

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
    int32_t given_current;
} Gimbal_Motor_t;
typedef enum
{
    NO_FORCE,
    FIXED,
    FREE,
}GIMBAL_mode_e;
typedef struct
{
    const RC_ctrl_t *gimbal_rc_ctrl;
    Gimbal_Motor_t gimbal_yaw_motor;
    Gimbal_Motor_t gimbal_pitch_motor;
    uint8_t cali_flag;
    GIMBAL_mode_e GIMBAL_mode;

} Gimbal_Control_t;

void Gimbal_Task(void *pvParameters);
#endif