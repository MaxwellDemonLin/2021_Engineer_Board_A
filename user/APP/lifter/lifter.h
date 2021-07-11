#ifndef LIFTER_H
#define LIFTER_H

#include "main.h"
#include "CAN_receive.h"
#include "remote_control.h"
#include "pid.h"

void Lift_task(void *pvParameters);

#define WHEEL_PERIMETER  0.43f  //cm

//#define RAW_KEY           KEY_PRESSED_OFFSET_R
#define LARGE_ISLAND_KEY  KEY_PRESSED_OFFSET_Z
#define SMALL_ISLAND_KEY  KEY_PRESSED_OFFSET_X
#define EXCHANGE_KEY      KEY_PRESSED_OFFSET_E
#define LOOT_KEY          KEY_PRESSED_OFFSET_B
#define DOWN_KEY          KEY_PRESSED_OFFSET_C
#define RAW_UP_KEY        KEY_PRESSED_OFFSET_SHIFT
#define RAW_DOWN_KEY      KEY_PRESSED_OFFSET_CTRL 

#define LIFT_HEIGHT_RC_SEN 1.6f

#define KEY_CHANGE_VALUE  150

#define LOOT_HEIGHT_ECD           10000
#define LARGE_ISLAND_HEIGHT_ECD   16000
#define SMALL_ISLAND_HEIGHT_ECD   10000
#define EXCHANGE_HEIGHT_ECD       10000

#define Half_ecd_range 4096
#define ecd_range 8191

#define LIFT_SPEED_KP 20
#define LIFT_SPEED_KI 0.1
#define LIFT_SPEED_KD 0
#define LIFT_SPEED_MAX_OUT 10000
#define LIFT_SPEED_MAX_IOUT 0


#define LIFT_COUNT_KP 0.3
#define LIFT_COUNT_KI 0.0
#define LIFT_COUNT_KD 0
#define LIFT_COUNT_MAX_OUT 2500
#define LIFT_COUNT_MAX_IOUT 5



#define LIFT_CALI_CURRENT_DOWN   500 
#define LIFT_CALI_CURRENT_UP		 500
#define CALI_TIME 600

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
   LIFT_CALI,
}lift_mode_e;


typedef struct
{
    const RC_ctrl_t *lift_RC; 
    const motor_measure_t *lift_motor_measure[2];   
    PidTypeDef lift_height_pid[2]; 
    PidTypeDef lift_speed_pid[2];
    int16_t give_current[2];
    fp32 speed[2];
    fp32 speed_set;
    fp32 height[2];
    int32_t ecd_sum_set[2];
    int16_t given_current[2];
    int8_t ecd_count[2];
    int32_t motor_count_set[2];
    int32_t down_sum_ecd[2];
    int32_t up_sum_ecd[2];
    int32_t motor_sum_ecd[2];
    uint8_t cali_step; 
    uint8_t cali_flag;
    lift_mode_e lift_mode;
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
