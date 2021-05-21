#ifndef RESCUEER_H
#define RESCUEER_H

#include "main.h"
#include "CAN_receive.h"
#include "remote_control.h"
#include "pid.h"

#define OPEN_COUNT
//速度PID
#define RESCUE_SPEED_KP 3000
#define RESCUE_SPEED_KI 0
#define RESCUE_SPEED_KD 0
#define RESCUE_SPEED_MAX_OUT 20000
#define RESCUE_SPEED_MAX_IOUT 0
//编码器范围 
#define Half_ecd_range 4096
#define ecd_range 8191
//圈数PID
#define RESCUE_COUNT_KP 20
#define RESCUE_COUNT_KI 0.01
#define RESCUE_COUNT_KD 0
#define RESCUE_COUNT_MAX_OUT 20000
#define RESCUE_COUNT_MAX_IOUT 20000
//校准电流
#define RESCUE_CALI_CURRENT 3000
#define CALI_TIME 2000
typedef enum
{
    CLOSE,
    OPEN,
} Claw_mode_e;
typedef struct
{
    uint16_t *open_ecd_set;
    uint16_t *close_ecd_set;
} rescue_ecd_set_t;
typedef struct
{
    uint16_t *open_ecd_set;
    uint16_t *close_ecd_set;
} rescue_count_set_t;
typedef struct
{
    rescue_count_set_t *rescue_count_set;
    rescue_ecd_set_t *rescue_ecd_set;
} rescue_position_set_t;
typedef struct
{
    const RC_ctrl_t *rescue_RC; 
    const motor_measure_t *rescue_motor_measure[2];  
    rescue_position_set_t *rescue_position_set[2];
    PidTypeDef rescue_count_pid[2]; 
    PidTypeDef rescue_speed_pid[2];
    int16_t give_current[2];
    fp32 speed[2];
    fp32 speed_set;

    uint16_t ecd_set[2];
    Claw_mode_e Claw_mode;
    int16_t given_current[2];
    int8_t ecd_count[2];
    uint16_t motor_count_set;
    uint16_t motor_count[2];

} rescue_control_e;






#endif