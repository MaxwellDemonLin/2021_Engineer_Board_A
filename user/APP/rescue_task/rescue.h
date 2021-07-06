#ifndef RESCUEER_H
#define RESCUEER_H

#include "main.h"
#include "CAN_receive.h"
#include "remote_control.h"
#include "pid.h"
void Rescue_task(void *pvParameters);

#define OPEN_COUNT
#define RESCUE_KEY           KEY_PRESSED_OFFSET_R
//速度PID
#define RESCUE_SPEED_KP 14
#define RESCUE_SPEED_KI 0.05
#define RESCUE_SPEED_KD 0
#define RESCUE_SPEED_MAX_OUT 8000
#define RESCUE_SPEED_MAX_IOUT 1200
//编码器范围 
#define Half_ecd_range 4096
#define ecd_range 8191
//圈数PID
#define RESCUE_COUNT_KP 0.2
#define RESCUE_COUNT_KI 0
#define RESCUE_COUNT_KD 200
#define RESCUE_COUNT_MAX_OUT 10000
#define RESCUE_COUNT_MAX_IOUT 20000

//校准电流
#define RESCUE_CALI_CURRENT 1500
#define CALI_TIME 2000
typedef enum
{
    CLOSE,
    OPEN,
} Claw_mode_e;
typedef struct
{
    const RC_ctrl_t *rescue_RC; 
    const motor_measure_t *rescue_motor_measure[2];  
    uint8_t close_flag[2];
    int32_t motor_sum_ecd[2];
    PidTypeDef rescue_count_pid[2]; 
    PidTypeDef rescue_speed_pid[2];
    PidTypeDef rescue_ecd_pid[2];
    int16_t give_current[2];
    fp32 speed[2];
    fp32 speed_set;
    Claw_mode_e Claw_mode[2];
		uint8_t cali_step;
    int16_t given_current[2];
    int32_t open_ecd_set[2];
    int32_t close_ecd_set[2];

} rescue_control_e;






#endif
