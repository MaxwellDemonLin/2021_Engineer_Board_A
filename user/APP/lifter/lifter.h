#ifndef LIFTER_H
#define LIFTER_H

#include "main.h"
#include "CAN_receive.h"
#include "remote_control.h"
#include "pid.h"

void Lift_task(void *pvParameters);

#define WHEEL_PERIMETER  0.43f  //cm
//底盘摇摆按键
//#define RAW_KEY           KEY_PRESSED_OFFSET_R
#define LARGE_ISLAND_KEY  KEY_PRESSED_OFFSET_Z
#define SMALL_ISLAND_KEY  KEY_PRESSED_OFFSET_X
#define EXCHANGE_KEY      KEY_PRESSED_OFFSET_E
#define LOOT_KEY          KEY_PRESSED_OFFSET_B
#define DOWN_KEY          KEY_PRESSED_OFFSET_C

//遥控器和高度转化比例
#define LIFT_HEIGHT_RC_SEN 0.1f

//上升高度
#define LOOT_HEIGHT           35.0f
#define LARGE_ISLAND_HEIGHT   30.0f
#define SMALL_ISLAND_HEIGHT   25.0f
#define EXCHANGE_HEIGHT       20.0f
//编码器范围 
#define Half_ecd_range 4096
#define ecd_range 8191
//速度PID
#define LIFT_SPEED_KP 3000
#define LIFT_SPEED_KI 0
#define LIFT_SPEED_KD 0
#define LIFT_SPEED_MAX_OUT 20000
#define LIFT_SPEED_MAX_IOUT 0

//圈数PID
#define LIFT_COUNT_KP 20
#define LIFT_COUNT_KI 0.01
#define LIFT_COUNT_KD 0
#define LIFT_COUNT_MAX_OUT 20000
#define LIFT_COUNT_MAX_IOUT 20000

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


typedef enum
{
   LIFT_RAW,
   LIFT_LARGE_RESOURCE_ISLAND,
   LIFT_SMALL_RESOURCE_ISLAND,
   LIFT_EXCHANGE,
   LIFT_LOOT,
   LIFT_NO_FORCE,
   LIFT_DOWN,
} lift_mode_e;

typedef struct
{
    const RC_ctrl_t *lift_RC; 
    const motor_measure_t *lift_motor_measure[2];   
    lift_mode_e lift_mode;
    PidTypeDef lift_height_pid[2]; 
    PidTypeDef lift_speed_pid[2];
    int16_t give_current[2];
    fp32 speed[2];
    fp32 speed_set;
    fp32 height[2];
    fp32 height_set;
    int16_t given_current[2];
    int8_t ecd_count[2];
    uint16_t motor_count_set;
    uint16_t motor_count[2];
} Lift_control_e;

typedef enum
{
   RAW,
   LARGE_RESOURCE_ISLAND,
   SMALL_RESOURCE_ISLAND,
   EXCHANGE,
   LOOT,
} engineering_mode_e;
#endif
