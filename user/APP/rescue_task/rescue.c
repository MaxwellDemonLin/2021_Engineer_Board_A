#include "rescue.h"
#include "remote_control.h"
#include "pid.h"
#include "CAN_receive.h"
rescue_control_e rescue_control;

#define GIMBAL_CALI_GYRO_JUDGE(gyro, cmd_time, angle_set, angle, ecd_set, ecd, step) \
    {                                                                                \
        if ((gyro) < GIMBAL_CALI_GYRO_LIMIT)                                         \
        {                                                                            \
            (cmd_time)++;                                                            \
            if ((cmd_time) > GIMBAL_CALI_STEP_TIME)                                  \
            {                                                                        \
                (cmd_time) = 0;                                                      \
                (angle_set) = (angle);                                               \
                (ecd_set) = (ecd);                                                   \
                (step)++;                                                            \
            }                                                                        \
        }                                                                            \
    }
void Rescue_init(rescue_control_e *rescue_init);
static void Rescue_data_update(rescue_control_e *rescue_update);
static void Rescue_cali(rescue_control_e *rescue_cail);
void Rescue_task(void *pvParameters)
{
    Rescue_init(&rescue_control);
    while (1)
    {

    }
}
void Rescue_init(rescue_control_e *rescue_init)
{
    fp32 rescue_speed_pid[3] = {RESCUE_SPEED_KP, RESCUE_SPEED_KI, RESCUE_SPEED_KD};
    fp32 rescue_count_pid[3] = {RESCUE_COUNT_KP, RESCUE_COUNT_KI, RESCUE_COUNT_KD};
    rescue_init->rescue_RC = get_remote_control_point();
    rescue_init->rescue_motor_measure[0] = get_Rescue_Motor_Measure_Point(0);
    rescue_init->rescue_motor_measure[1] = get_Rescue_Motor_Measure_Point(0);

    PID_Init(&rescue_init->rescue_speed_pid[0], PID_DELTA, rescue_speed_pid, RESCUE_SPEED_MAX_OUT, RESCUE_SPEED_MAX_IOUT);
    PID_Init(&rescue_init->rescue_speed_pid[1], PID_DELTA, rescue_speed_pid, RESCUE_SPEED_MAX_OUT, RESCUE_SPEED_MAX_IOUT);

    PID_Init(&rescue_init->rescue_count_pid[0], PID_DELTA, rescue_count_pid, RESCUE_COUNT_MAX_IOUT, RESCUE_COUNT_MAX_IOUT);
    PID_Init(&rescue_init->rescue_count_pid[1], PID_DELTA, rescue_count_pid, RESCUE_COUNT_MAX_IOUT, RESCUE_COUNT_MAX_IOUT);
    Rescue_task(rescue_init);
}


static void Rescue_cali(rescue_control_e *rescue_cail)
{
    static uint16_t cali_time = 0;
    while(1)
    {
        CAN_CMD_RESCUE(RESCUE_CALI_CURRENT,RESCUE_CALI_CURRENT);
        if(rescue_cail->rescue_motor_measure[0]->ecd==rescue_cail->rescue_motor_measure[0]->last_ecd)
        {
            cali_time++;
        }
        if(cali_time>CALI_TIME)
        {
            break;
        }
    }
}

static void Rescue_data_update(rescue_control_e *rescue_update)
{
    if (rescue_update->rescue_motor_measure[0]->ecd - rescue_update->rescue_motor_measure[0]->last_ecd > Half_ecd_range)
    {
        rescue_update->motor_count[0]--;
    }
    else if (rescue_update->rescue_motor_measure[0]->ecd - rescue_update->rescue_motor_measure[0]->last_ecd < -Half_ecd_range)
    {
        rescue_update->motor_count[0]++;
    }


    if (rescue_update->rescue_motor_measure[1]->ecd - rescue_update->rescue_motor_measure[1]->last_ecd > Half_ecd_range)
    {
        rescue_update->motor_count[1]--;
    }
    else if (rescue_update->rescue_motor_measure[1]->ecd - rescue_update->rescue_motor_measure[1]->last_ecd < -Half_ecd_range)
    {
        rescue_update->motor_count[1]++;
    }
}
