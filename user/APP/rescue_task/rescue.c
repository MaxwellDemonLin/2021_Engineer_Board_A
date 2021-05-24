#include "rescue.h"
#include "remote_control.h"
#include "pid.h"
#include "CAN_receive.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
rescue_control_e rescue_control;

void Rescue_init(rescue_control_e *rescue_init);
static void Rescue_data_update(rescue_control_e *rescue_update);
static void Rescue_cali(rescue_control_e *rescue_cail);
static void Rescue_mode_set(rescue_control_e *rescue_mode_set);
static void Rescue_control_PID(rescue_control_e *rescue_calc);
void Rescue_task(void *pvParameters)
{
    Rescue_init(&rescue_control);
    while (1)
    {
        Rescue_cali(&rescue_control);
        Rescue_data_update(&rescue_control);
        if(rescue_control.cali_step==2)
        {
        Rescue_mode_set(&rescue_control);
        Rescue_control_PID(&rescue_control);
        CAN_CMD_RESCUE(rescue_control.give_current[0], rescue_control.give_current[1]);
        }
        vTaskDelay(2);
    }
}
void Rescue_init(rescue_control_e *rescue_init)
{
    fp32 rescue_speed_pid[3] = {RESCUE_SPEED_KP, RESCUE_SPEED_KI, RESCUE_SPEED_KD};
    fp32 rescue_count_pid[3] = {RESCUE_COUNT_KP, RESCUE_COUNT_KI, RESCUE_COUNT_KD};
    fp32 rescue_ecd_pid[3] = {RESCUE_ECD_KP, RESCUE_ECD_KI, RESCUE_ECD_KD};

    rescue_init->rescue_RC = get_remote_control_point();
    rescue_init->rescue_motor_measure[0] = get_Rescue_Motor_Measure_Point(0);
    rescue_init->rescue_motor_measure[1] = get_Rescue_Motor_Measure_Point(0);
    rescue_init->cali_step = 0;

    PID_Init(&rescue_init->rescue_speed_pid[0], PID_DELTA, rescue_speed_pid, RESCUE_SPEED_MAX_OUT, RESCUE_SPEED_MAX_IOUT);
    PID_Init(&rescue_init->rescue_speed_pid[1], PID_DELTA, rescue_speed_pid, RESCUE_SPEED_MAX_OUT, RESCUE_SPEED_MAX_IOUT);

    PID_Init(&rescue_init->rescue_count_pid[0], PID_DELTA, rescue_count_pid, RESCUE_COUNT_MAX_IOUT, RESCUE_COUNT_MAX_IOUT);
    PID_Init(&rescue_init->rescue_count_pid[1], PID_DELTA, rescue_count_pid, RESCUE_COUNT_MAX_IOUT, RESCUE_COUNT_MAX_IOUT);

    PID_Init(&rescue_init->rescue_count_pid[0], PID_DELTA, rescue_ecd_pid, RESCUE_SPEED_MAX_OUT, RESCUE_SPEED_MAX_IOUT);
    PID_Init(&rescue_init->rescue_count_pid[1], PID_DELTA, rescue_ecd_pid, RESCUE_SPEED_MAX_OUT, RESCUE_SPEED_MAX_IOUT);
    rescue_init->Claw_mode[0]=OPEN;
    rescue_init->Claw_mode[1]=OPEN;
}

static void Rescue_cali(rescue_control_e *rescue_cali)
{
    static uint16_t cali_time = 0;
    if (rescue_cali->cali_step == 0)
    {
        CAN_CMD_RESCUE(-RESCUE_CALI_CURRENT, -RESCUE_CALI_CURRENT);
        if (rescue_cali->rescue_motor_measure[0]->ecd == rescue_cali->rescue_motor_measure[0]->last_ecd)
        {
            cali_time++;
        }
        if (cali_time > CALI_TIME)
        {
            rescue_cali->open_ecd_set[0] = rescue_cali->motor_sum_ecd[0];
            rescue_cali->open_ecd_set[1] = rescue_cali->motor_sum_ecd[1];
            rescue_cali->cali_step++;
            cali_time = 0;
        }
    }
    if (rescue_cali->cali_step == 1)
    {
        CAN_CMD_RESCUE(RESCUE_CALI_CURRENT, RESCUE_CALI_CURRENT);
        if (rescue_cali->rescue_motor_measure[0]->ecd == rescue_cali->rescue_motor_measure[0]->last_ecd)
        {
            cali_time++;
        }
        if (cali_time > CALI_TIME)
        {
            rescue_cali->close_ecd_set[0] = rescue_cali->motor_sum_ecd[0];
            rescue_cali->close_ecd_set[1] = rescue_cali->motor_sum_ecd[1];
            rescue_cali->cali_step++;
            cali_time = 0;
        }
    }
}

static void Rescue_data_update(rescue_control_e *rescue_update)
{
    uint8_t i = 0;
    for (i = 0; i <= 2; i++)
    {
        if (rescue_update->rescue_motor_measure[i]->count >= 0)
        {
            rescue_update->motor_sum_ecd[i] = rescue_update->rescue_motor_measure[i]->count * 8192 + rescue_update->rescue_motor_measure[i]->ecd;
        }
        if (rescue_update->rescue_motor_measure[i]->count < 0)
        {
            rescue_update->motor_sum_ecd[i] = rescue_update->rescue_motor_measure[i]->count * 8192 - rescue_update->rescue_motor_measure[i]->ecd;
        }
    }
}

static void Rescue_mode_set(rescue_control_e *rescue_mode_set)
{
    static uint8_t key_rescue_flag = 0;
    static uint8_t key_rescue_time = 0;
    if (key_rescue_time)
    {
        key_rescue_time--;
    }
    if (rescue_mode_set->Claw_mode[0] == OPEN && rescue_mode_set->close_flag[0] == 1)
    {
        rescue_mode_set->Claw_mode[0] = CLOSE;
    }
    if (rescue_mode_set->Claw_mode[1] == OPEN && rescue_mode_set->close_flag[1] == 1)
    {
        rescue_mode_set->Claw_mode[1] = CLOSE;
    }
    if (!key_rescue_time)
    {
        if (key_rescue_flag)
        {
            if (rescue_mode_set->rescue_RC->key.v && RESCUE_KEY)
            {
                rescue_mode_set->Claw_mode[0] = OPEN;
                rescue_mode_set->Claw_mode[1] = OPEN;
                key_rescue_time = 200;
                key_rescue_flag = 0;
            }
        }
        else if (!key_rescue_flag)
        {
            if (rescue_mode_set->rescue_RC->key.v && RESCUE_KEY)
            {
                rescue_mode_set->Claw_mode[0] = CLOSE;
                rescue_mode_set->Claw_mode[1] = CLOSE;
                key_rescue_time = 200;
                key_rescue_flag = 1;
            }
        }
    }
}
static void Rescue_control_PID(rescue_control_e *rescue_calc)
{
    uint8_t i = 0;
    for (i = 0; i <= 1; i++)
    {
        if (rescue_calc->Claw_mode[i] == OPEN)
        {
            PID_Calc(rescue_calc->rescue_count_pid, (fp32)rescue_calc->open_ecd_set[i], rescue_calc->motor_sum_ecd[i]);
            rescue_calc->give_current[i]=PID_Calc(rescue_calc->rescue_speed_pid, rescue_calc->rescue_motor_measure[i]->speed_rpm, rescue_calc->rescue_count_pid->out);
        }
        else if (rescue_calc->Claw_mode[i] == CLOSE)
        {
            PID_Calc(rescue_calc->rescue_count_pid, (fp32)rescue_calc->close_ecd_set[i], rescue_calc->motor_sum_ecd[i]);
            rescue_calc->give_current[i]=PID_Calc(rescue_calc->rescue_speed_pid, rescue_calc->rescue_motor_measure[i]->speed_rpm, rescue_calc->rescue_count_pid->out);
        }
    }
}
