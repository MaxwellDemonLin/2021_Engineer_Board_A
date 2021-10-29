#ifndef CLAWER_H
#define CLAWER_H

#include "main.h"
#include "CAN_receive.h"
#include "remote_control.h"
#include "pid.h"

void Claw_task(void *pvParameters);

#define RAW_FORWARD_KEY KEY_PRESSED_OFFSET_G
#define RAW_BACKWARS_KEY KEY_PRESSED_OFFSET_B
//遥控器和高度转化比例
#define CLAW_HEIGHT_RC_SEN 0.1
//键盘原生模式上升下降
#define KEY_CHANGE_VALUE 50
//上升高度
#define FORWARD_HORIZONTAL 80000
#define BACKWARD_HORIONT 10000
#define VERTICAL_DOWN 10000
//编码器范�?
#define Half_ecd_range 4096
#define ecd_range 8191
//速度PID
#define CLAW_SPEED_KP 18
#define CLAW_SPEED_KI 1.0
#define CLAW_SPEED_KD 0
#define CLAW_SPEED_MAX_OUT 8000
#define CLAW_SPEED_MAX_IOUT 0

//圈数PID
#define CLAW_COUNT_KP 0.30
#define CLAW_COUNT_KI 0
#define CLAW_COUNT_KD 0
#define CLAW_COUNT_MAX_OUT 2000
#define CLAW_COUNT_MAX_IOUT 5
//校准电流
#define CLAW_CALI_CURRENT_DOWN -2000
#define CLAW_CALI_CURRENT_UP 2500
#define CALI_TIME 600
//死区限制
#define rc_deadline_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }
#define EXCHANGE_ADD 10000
#define UP_ECD_ADD 120000


#define OPEN_COUNT
#define RESCUE_KEY           KEY_PRESSED_OFFSET_R

#define RESCUE_SPEED_KP 16
#define RESCUE_SPEED_KI 0.05
#define RESCUE_SPEED_KD 0
#define RESCUE_SPEED_MAX_OUT 15000
#define RESCUE_SPEED_MAX_IOUT 1200

#define Half_ecd_range 4096
#define ecd_range 8191

#define RESCUE_COUNT_KP 0.3
#define RESCUE_COUNT_KI 0
#define RESCUE_COUNT_KD 400
#define RESCUE_COUNT_MAX_OUT 10000
#define RESCUE_COUNT_MAX_IOUT 20000


#define RESCUE_CALI_CURRENT 1500
#define RESCUE_CALI_TIME 3000

typedef enum
{
    CLAW_RAW,
    CLAW_LARGE_RESOURCE_ISLAND,
    CLAW_SMALL_RESOURCE_ISLAND,
    CLAW_EXCHANGE,
    CLAW_LOOT,
    CLAW_NO_FORCE,
    CLAW_DOWN,
    CLAW_CALI,
    CLAW_THROUGH,
} claw_mode_e;

typedef struct
{
    const RC_ctrl_t *claw_RC;
    const motor_measure_t *claw_motor_measure[2];
    PidTypeDef claw_height_pid[2];
    PidTypeDef claw_speed_pid[2];
    int16_t give_current[2];
    int32_t ecd_sum_set[2];
    int16_t given_current[2];
    int32_t motor_count_set[2];
    int32_t down_sum_ecd[2];
    int32_t exchange_sum_ecd[2];
    int32_t up_sum_ecd[2];
    int32_t motor_sum_ecd[2];
    uint8_t cali_step;
    uint8_t cali_flag;
    claw_mode_e claw_mode;
} Claw_control_e;
const Claw_control_e *get_claw_measure(void);



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
    fp32 speed[2];
    fp32 speed_set;
    Claw_mode_e Claw_mode[2];
		uint8_t cali_step;
    int16_t given_current[2];
    int32_t open_ecd_set[2];
    int32_t close_ecd_set[2];

} rescue_control_e;
#endif
