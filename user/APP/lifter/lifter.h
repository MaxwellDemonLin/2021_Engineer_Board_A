#ifndef LIFTER_H
#define LIFTER_H

#include "main.h"
#include "CAN_receive.h"
#include "remote_control.h"
#include "pid.h"

void Lift_Rescue_task(void *pvParameters);
#define WHEEL_PERIMETER  0.43f  //cm

//#define RAW_KEY           KEY_PRESSED_OFFSET_R
#define LARGE_ISLAND_KEY  KEY_PRESSED_OFFSET_Z
#define SMALL_ISLAND_KEY  KEY_PRESSED_OFFSET_X
#define EXCHANGE_KEY      KEY_PRESSED_OFFSET_E
#define LOOT_KEY          KEY_PRESSED_OFFSET_B
#define DOWN_KEY          KEY_PRESSED_OFFSET_C
#define RAW_UP_KEY        KEY_PRESSED_OFFSET_SHIFT
#define RAW_DOWN_KEY      KEY_PRESSED_OFFSET_CTRL 

#define LIFT_HEIGHT_RC_SEN 0.5f

#define KEY_CHANGE_VALUE  100

#define LOOT_HEIGHT_ECD           10000
#define LARGE_ISLAND_HEIGHT_ECD   57670
#define SMALL_ISLAND_HEIGHT_ECD   10000
#define EXCHANGE_HEIGHT_ECD       302671

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

#define FULL_RANGE 380000

#define LIFT_CALI_CURRENT_DOWN   2500 
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


#define OPEN_COUNT
#define RESCUE_KEY           KEY_PRESSED_OFFSET_R

#define RESCUE_SPEED_KP 14
#define RESCUE_SPEED_KI 0.05
#define RESCUE_SPEED_KD 0
#define RESCUE_SPEED_MAX_OUT 8000
#define RESCUE_SPEED_MAX_IOUT 1200

#define Half_ecd_range 4096
#define ecd_range 8191

#define RESCUE_COUNT_KP 0.05
#define RESCUE_COUNT_KI 0
#define RESCUE_COUNT_KD 200
#define RESCUE_COUNT_MAX_OUT 10000
#define RESCUE_COUNT_MAX_IOUT 20000


#define RESCUE_CALI_CURRENT 1500
#define RESCUE_CALI_TIME 6000
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
