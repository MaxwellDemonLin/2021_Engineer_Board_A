#ifndef CLAWER_H
#define CLAWER_H

#include "main.h"
#include "CAN_receive.h"
#include "remote_control.h"
#include "pid.h"

void Claw_task(void *pvParameters);

#define RAW_FORWARD_KEY   KEY_PRESSED_OFFSET_G
#define RAW_BACKWARS_KEY KEY_PRESSED_OFFSET_B
//遥控器和高度转化比例
#define CLAW_HEIGHT_RC_SEN 0.5f
//键盘原生模式上升下降
#define KEY_CHANGE_VALUE  50
//上升高度
#define LOOT_HEIGHT_ECD           10000
#define LARGE_ISLAND_HEIGHT_ECD   16000
#define SMALL_ISLAND_HEIGHT_ECD   10000
#define EXCHANGE_HEIGHT_ECD       10000
//编码器范围 
#define Half_ecd_range 4096
#define ecd_range 8191
//速度PID
#define CLAW_SPEED_KP 18
#define CLAW_SPEED_KI 0
#define CLAW_SPEED_KD 0
#define CLAW_SPEED_MAX_OUT 8000
#define CLAW_SPEED_MAX_IOUT 0

//圈数PID
#define CLAW_COUNT_KP 0.06
#define CLAW_COUNT_KI 0.0
#define CLAW_COUNT_KD 0
#define CLAW_COUNT_MAX_OUT 600
#define CLAW_COUNT_MAX_IOUT 5
//校准电流
#define CLAW_CALI_CURRENT_DOWN   2000 
#define CLAW_CALI_CURRENT_UP		 2500
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
#define EXCHANGE_ADD  10000
#define UP_ECD_ADD   30000
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
}claw_mode_e;


typedef struct
{
    const RC_ctrl_t *claw_RC; 
    const motor_measure_t *claw_motor_measure[2];   
    PidTypeDef claw_height_pid[2]; 
    PidTypeDef claw_speed_pid[2];
    int16_t give_current[2];
    fp32 speed[2];
    fp32 speed_set;
    fp32 height[2];
    int32_t ecd_sum_set[2];
    int16_t given_current[2];
    int8_t ecd_count[2];
    int32_t motor_count_set[2];
    int32_t down_sum_ecd[2];
    int32_t exchange_sum_ecd[2];
    int32_t up_sum_ecd[2];
    int32_t motor_sum_ecd[2];
    uint8_t cali_step; 
    uint8_t cali_flag;
    claw_mode_e claw_mode;
} Claw_control_e;

#endif
