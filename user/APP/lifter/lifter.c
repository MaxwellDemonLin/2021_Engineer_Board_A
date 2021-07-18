#include "lifter.h"
#include "remote_control.h"
#include "pid.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "math.h"

Lift_control_e lift_control;
rescue_control_e rescue_control;
void Lift_task_init(Lift_control_e *lift_control_init);
void lift_rc_to_control_vector(int32_t *ecd, Lift_control_e *lift_rc_to_vector);
static void lift_set_value(Lift_control_e *set_value);
static void lift_data_update(Lift_control_e *lift_update);
static void lift_control_PID(Lift_control_e *lift_control);
static void Lift_set_mode(Lift_control_e *lift_control_mode_set);
static void Lift_set_mode(Lift_control_e *lift_control_mode_set);
//static void lift_set_height(Lift_control_e *lift_control_set_control);
static void Lift_cali(Lift_control_e *Lift_cali);

void Rescue_init(rescue_control_e *rescue_init);
static void Rescue_data_update(rescue_control_e *rescue_update);
static void Rescue_cali(rescue_control_e *rescue_cail);
static void Rescue_mode_set(rescue_control_e *rescue_mode_set);
static void Rescue_control_PID(rescue_control_e *rescue_calc);

void Lift_Rescue_task(void *pvParameters)
{
    Lift_task_init(&lift_control);
    Rescue_init(&rescue_control);
    lift_data_update(&lift_control);
    Rescue_data_update(&rescue_control);
    //					vTaskDelayUntil(100);
    while (1)
    {
        Lift_set_mode(&lift_control);
        lift_set_value(&lift_control);
        lift_data_update(&lift_control);
        lift_control_PID(&lift_control);

        Rescue_mode_set(&rescue_control);
        Rescue_data_update(&rescue_control);
        Rescue_control_PID(&rescue_control);

        // CAN_CMD_LIFTER_RESCUE(lift_control.given_current[0],lift_control.given_current[1] , rescue_control.given_current[0], rescue_control.given_current[1]);
        CAN_CMD_LIFTER_RESCUE(0, 0, rescue_control.given_current[0], rescue_control.given_current[1]);
        vTaskDelay(1);
    }
}
void Lift_task_init(Lift_control_e *lift_control_init)
{
    fp32 lift_speed_pid[3] = {LIFT_SPEED_KP, LIFT_SPEED_KI, LIFT_SPEED_KD};
    fp32 lift_count_pid[3] = {LIFT_COUNT_KP, LIFT_COUNT_KI, LIFT_COUNT_KD};
    PID_Init(&lift_control_init->lift_speed_pid[0], PID_POSITION, lift_speed_pid, LIFT_SPEED_MAX_OUT, LIFT_SPEED_MAX_IOUT);
    PID_Init(&lift_control_init->lift_speed_pid[1], PID_POSITION, lift_speed_pid, LIFT_SPEED_MAX_OUT, LIFT_SPEED_MAX_IOUT);
    PID_Init(&lift_control_init->lift_height_pid[0], PID_POSITION, lift_count_pid, LIFT_COUNT_MAX_OUT, LIFT_COUNT_MAX_IOUT);
    PID_Init(&lift_control_init->lift_height_pid[1], PID_POSITION, lift_count_pid, LIFT_COUNT_MAX_OUT, LIFT_COUNT_MAX_IOUT);
    lift_control_init->lift_RC = get_remote_control_point();
    lift_control_init->lift_motor_measure[0] = get_Lifter_Motor_Measure_Point(0);

    lift_control_init->lift_motor_measure[1] = get_Lifter_Motor_Measure_Point(1);
    lift_control_init->lift_mode = LIFT_EXCHANGE;
}

static void Lift_set_mode(Lift_control_e *lift_control_mode_set)
{
    int32_t ecd_sum_set;
    int32_t ecd_add;
    if (lift_control_mode_set->cali_step == 0)
    {
        lift_control_mode_set->lift_mode = LIFT_CALI;
        return;
    }
    if ((switch_is_down(lift_control_mode_set->lift_RC->rc.s[0])))
    {
        lift_control.lift_mode = LIFT_NO_FORCE;
    }
    if ((switch_is_mid(lift_control_mode_set->lift_RC->rc.s[0])))
    {
        lift_control.lift_mode = LIFT_RAW;
    }
    /* static uint16_t cali_time = 0;
    if (time)
    {
        time--;
    }
    if (switch_is_up(lift_control_mode_set->lift_RC->rc.s[0]))
    {
        if (!time)
        {

            if ((lift_control_mode_set->lift_RC->key.v & EXCHANGE_KEY))
            {
                lift_control_mode_set->lift_mode = LIFT_EXCHANGE;
                lift_control_mode_set->ecd_sum_set[0] = lift_control_mode_set->down_sum_ecd[0] + EXCHANGE_HEIGHT_ECD;
                lift_control_mode_set->ecd_sum_set[1] = lift_control_mode_set->down_sum_ecd[1] - EXCHANGE_HEIGHT_ECD;
                time = 500;
            }
            else if ((lift_control_mode_set->lift_RC->key.v & LOOT_KEY))
            {
                lift_control_mode_set->lift_mode = LIFT_LOOT;
                lift_control_mode_set->ecd_sum_set[0] = lift_control_mode_set->down_sum_ecd[0] + LOOT_HEIGHT_ECD;
                lift_control_mode_set->ecd_sum_set[1] = lift_control_mode_set->down_sum_ecd[1] - LOOT_HEIGHT_ECD;
                time = 500;
            }
            else if ((lift_control_mode_set->lift_RC->key.v & LARGE_ISLAND_KEY))
            {
                lift_control_mode_set->lift_mode = LIFT_LARGE_RESOURCE_ISLAND;
                lift_control_mode_set->ecd_sum_set[0] = lift_control_mode_set->down_sum_ecd[0] + LARGE_ISLAND_HEIGHT_ECD;
                lift_control_mode_set->ecd_sum_set[1] = lift_control_mode_set->down_sum_ecd[1] - LARGE_ISLAND_HEIGHT_ECD;
                time = 500;
            }
            else if ((lift_control_mode_set->lift_RC->key.v & SMALL_ISLAND_KEY))
            {
                lift_control_mode_set->lift_mode = LIFT_SMALL_RESOURCE_ISLAND;
                lift_control_mode_set->ecd_sum_set[0] = lift_control_mode_set->down_sum_ecd[0] + SMALL_ISLAND_HEIGHT_ECD;
                lift_control_mode_set->ecd_sum_set[1] = lift_control_mode_set->down_sum_ecd[1] - SMALL_ISLAND_HEIGHT_ECD;
                time = 500;
            }
            else if (lift_control_mode_set->lift_RC->key.v & DOWN_KEY)
            {
                lift_control_mode_set->lift_mode = LIFT_DOWN;
                lift_control_mode_set->ecd_sum_set[0] = lift_control_mode_set->down_sum_ecd[0];
                lift_control_mode_set->ecd_sum_set[1] = lift_control_mode_set->down_sum_ecd[1];
                time = 500;
            }

            if (lift_control_mode_set->lift_RC->key.v & RAW_UP_KEY)
            {
                lift_control_mode_set->ecd_sum_set[0] += KEY_CHANGE_VALUE;
                lift_control_mode_set->ecd_sum_set[1] -= KEY_CHANGE_VALUE;
            }
            else if (lift_control_mode_set->lift_RC->key.v & RAW_DOWN_KEY)
            {
                lift_control_mode_set->ecd_sum_set[0] -= KEY_CHANGE_VALUE;
                lift_control_mode_set->ecd_sum_set[1] += KEY_CHANGE_VALUE;
            }

            if (lift_control_mode_set->lift_RC->key.v & RAW_DOWN_KEY && lift_control_mode_set->lift_RC->key.v & RAW_UP_KEY)
            {
                cali_time++;
            }
            if (!(lift_control_mode_set->lift_RC->key.v & RAW_DOWN_KEY && lift_control_mode_set->lift_RC->key.v & RAW_UP_KEY))
            {
                cali_time = 0;
            }
            if (cali_time == 1000)
            {
                lift_control_mode_set->cali_step = 0;
                lift_control_mode_set->lift_mode = LIFT_CALI;
                cali_time = 0;
            }
        }
    }
    if (switch_is_mid(lift_control_mode_set->lift_RC->rc.s[0]))
    {
        lift_control_mode_set->lift_mode = LIFT_RAW;
        lift_rc_to_control_vector(&ecd_add, lift_control_mode_set);

        lift_control_mode_set->ecd_sum_set[0] += ecd_add;
        lift_control_mode_set->ecd_sum_set[1] -= ecd_add;
        if (lift_control_mode_set->lift_RC->key.v & RAW_UP_KEY)
        {
            lift_control_mode_set->ecd_sum_set[0] += KEY_CHANGE_VALUE;
            lift_control_mode_set->ecd_sum_set[1] -= KEY_CHANGE_VALUE;
        }
        else if (lift_control_mode_set->lift_RC->key.v & RAW_DOWN_KEY)
        {
            lift_control_mode_set->ecd_sum_set[0] -= KEY_CHANGE_VALUE;
            lift_control_mode_set->ecd_sum_set[1] += KEY_CHANGE_VALUE;
        }
    }
    if (switch_is_down(lift_control_mode_set->lift_RC->rc.s[0]))
    {
        lift_control_mode_set->lift_mode = LIFT_NO_FORCE;
        lift_control_mode_set->ecd_sum_set[0] = lift_control_mode_set->down_sum_ecd[0]; //为了防止有力无力切换时上升发生抖动，因此将设定值设置为0
        lift_control_mode_set->ecd_sum_set[1] = lift_control_mode_set->down_sum_ecd[1];
    }

    if (lift_control_mode_set->ecd_sum_set[0] < lift_control_mode_set->down_sum_ecd[0])
    {
        lift_control_mode_set->ecd_sum_set[0] = lift_control_mode_set->down_sum_ecd[0];
        lift_control_mode_set->ecd_sum_set[1] = lift_control_mode_set->down_sum_ecd[1];
    }

    if (lift_control_mode_set->ecd_sum_set[0] > lift_control_mode_set->down_sum_ecd[0] + FULL_RANGE)
    {
        lift_control_mode_set->ecd_sum_set[0] = lift_control_mode_set->down_sum_ecd[0] + FULL_RANGE;
        lift_control_mode_set->ecd_sum_set[1] = lift_control_mode_set->down_sum_ecd[1] - FULL_RANGE;
    }*/
}
static void lift_set_value(Lift_control_e *set_value)
{
    int32_t ecd_add;
    if (set_value->lift_mode == LIFT_CALI)
    {
        Lift_cali(set_value);
        return;
    }
    if (set_value->lift_mode == LIFT_RAW)
    {
        lift_rc_to_control_vector(&ecd_add, set_value);
        set_value->ecd_sum_set[0] += ecd_add;
        set_value->ecd_sum_set[1] -= ecd_add;

        if (set_value->lift_RC->key.v & RAW_UP_KEY && !set_value->lift_RC->mouse.press_r)
        {
            set_value->ecd_sum_set[0] += KEY_CHANGE_VALUE;
            set_value->ecd_sum_set[1] -= KEY_CHANGE_VALUE;
        }
        else if (set_value->lift_RC->key.v & RAW_DOWN_KEY && !set_value->lift_RC->mouse.press_r)
        {
            set_value->ecd_sum_set[0] -= KEY_CHANGE_VALUE;
            set_value->ecd_sum_set[1] += KEY_CHANGE_VALUE;
        }


        if ((set_value->lift_RC->key.v & EXCHANGE_KEY) && set_value->lift_RC->mouse.press_r)
        {
            set_value->lift_mode = LIFT_EXCHANGE;
            set_value->ecd_sum_set[0] = set_value->down_sum_ecd[0] + EXCHANGE_HEIGHT_ECD;
            set_value->ecd_sum_set[1] = set_value->down_sum_ecd[1] - EXCHANGE_HEIGHT_ECD;
        }
        else if ((set_value->lift_RC->key.v & LARGE_ISLAND_KEY) && set_value->lift_RC->mouse.press_r)
        {
            set_value->lift_mode = LIFT_LARGE_RESOURCE_ISLAND;
            set_value->ecd_sum_set[0] = set_value->down_sum_ecd[0] + LARGE_ISLAND_HEIGHT_ECD;
            set_value->ecd_sum_set[1] = set_value->down_sum_ecd[1] - LARGE_ISLAND_HEIGHT_ECD;
        }
        

        if (set_value->ecd_sum_set[0] < set_value->down_sum_ecd[0])
        {
            set_value->ecd_sum_set[0] = set_value->down_sum_ecd[0];
            set_value->ecd_sum_set[1] = set_value->down_sum_ecd[1];
        }

        if (set_value->ecd_sum_set[0] > set_value->down_sum_ecd[0] + FULL_RANGE)
        {
            set_value->ecd_sum_set[0] = set_value->down_sum_ecd[0] + FULL_RANGE;
            set_value->ecd_sum_set[1] = set_value->down_sum_ecd[1] - FULL_RANGE;
        }
    }
}
static void lift_rc_to_control_vector(int32_t *ecd, Lift_control_e *lift_rc_to_vector)
{
    fp32 ecd_channel;

    rc_deadline_limit(lift_rc_to_vector->lift_RC->rc.ch[3], ecd_channel, 10);

    *ecd = ecd_channel * LIFT_HEIGHT_RC_SEN;
}
static void lift_control_PID(Lift_control_e *lift_control)
{
    if (lift_control->lift_mode == LIFT_CALI)
    {
        return;
    }

    PID_Calc(&lift_control->lift_height_pid[0], lift_control->motor_sum_ecd[0], lift_control->ecd_sum_set[0]);
    PID_Calc(&lift_control->lift_height_pid[1], lift_control->motor_sum_ecd[1], lift_control->ecd_sum_set[1]);

    lift_control->given_current[0] = (int16_t)PID_Calc(&lift_control->lift_speed_pid[0], lift_control->lift_motor_measure[0]->speed_rpm, lift_control->lift_height_pid[0].out);
    lift_control->given_current[1] = (int16_t)PID_Calc(&lift_control->lift_speed_pid[1], lift_control->lift_motor_measure[1]->speed_rpm, lift_control->lift_height_pid[1].out);
}
static void lift_data_update(Lift_control_e *lift_update)
{

    uint8_t i = 0;
    for (i = 0; i <= 1; i++)
    {
        if (lift_update->lift_motor_measure[i]->count >= 0)
        {
            lift_update->motor_sum_ecd[i] = lift_update->lift_motor_measure[i]->count * ecd_range + lift_update->lift_motor_measure[i]->ecd;
        }
        if (lift_update->lift_motor_measure[i]->count < 0)
        {
            lift_update->motor_sum_ecd[i] = lift_update->lift_motor_measure[i]->count * ecd_range + lift_update->lift_motor_measure[i]->ecd;
        }
    }
}

static void Lift_cali(Lift_control_e *Lift_cali)
{
    static uint16_t cali_time = 0;
    Lift_cali->given_current[0] = -500;
    Lift_cali->given_current[1] = 500;
    if (Lift_cali->lift_motor_measure[0]->ecd == Lift_cali->lift_motor_measure[0]->last_ecd)
    {
        cali_time++;
    }
    if (cali_time > CALI_TIME)
    {
        Lift_cali->down_sum_ecd[0] = Lift_cali->motor_sum_ecd[0] + 20000;
        Lift_cali->down_sum_ecd[1] = Lift_cali->motor_sum_ecd[1] - 20000;
        Lift_cali->ecd_sum_set[0] = Lift_cali->motor_sum_ecd[0];
        Lift_cali->ecd_sum_set[1] = Lift_cali->motor_sum_ecd[1];
        Lift_cali->cali_step++;
        cali_time = 0;
        Lift_cali->lift_mode = LIFT_RAW;
    }
}

void Rescue_init(rescue_control_e *rescue_init)
{
    fp32 rescue_speed_pid[3] = {RESCUE_SPEED_KP, RESCUE_SPEED_KI, RESCUE_SPEED_KD};
    fp32 rescue_count_pid[3] = {RESCUE_COUNT_KP, RESCUE_COUNT_KI, RESCUE_COUNT_KD};
    rescue_init->rescue_RC = get_remote_control_point();
    rescue_init->rescue_motor_measure[0] = get_Rescue_Motor_Measure_Point(0);
    rescue_init->rescue_motor_measure[1] = get_Rescue_Motor_Measure_Point(1);

    rescue_init->cali_step = 0;

    PID_Init(&rescue_init->rescue_speed_pid[0], PID_POSITION, rescue_speed_pid, RESCUE_SPEED_MAX_OUT, RESCUE_SPEED_MAX_IOUT);
    PID_Init(&rescue_init->rescue_speed_pid[1], PID_POSITION, rescue_speed_pid, RESCUE_SPEED_MAX_OUT, RESCUE_SPEED_MAX_IOUT);

    PID_Init(&rescue_init->rescue_count_pid[0], PID_POSITION, rescue_count_pid, RESCUE_SPEED_MAX_OUT, RESCUE_SPEED_MAX_IOUT);
    PID_Init(&rescue_init->rescue_count_pid[1], PID_POSITION, rescue_count_pid, RESCUE_SPEED_MAX_OUT, RESCUE_SPEED_MAX_IOUT);
    rescue_init->rescue_count_pid[0].derivative_output_filter_coefficient = exp(-0.5 * 1E-3);
    rescue_init->rescue_count_pid[0].proportion_output_filter_coefficient = exp(-50 * 1E-3);

    rescue_init->rescue_count_pid[1].derivative_output_filter_coefficient = exp(-0.5 * 1E-3);
    rescue_init->rescue_count_pid[1].proportion_output_filter_coefficient = exp(-50 * 1E-3);

    rescue_init->rescue_speed_pid[0].derivative_output_filter_coefficient = exp(-0.05 * 1E-3);
    rescue_init->rescue_speed_pid[0].proportion_output_filter_coefficient = exp(-400 * 1E-3);

    rescue_init->rescue_speed_pid[1].derivative_output_filter_coefficient = exp(-0.05 * 1E-3);
    rescue_init->rescue_speed_pid[1].proportion_output_filter_coefficient = exp(-400 * 1E-3);

    rescue_init->Claw_mode[0] = OPEN;
    rescue_init->Claw_mode[1] = OPEN;
}

static void Rescue_cali(rescue_control_e *rescue_cali)
{
    static uint16_t cali_time = 0;
    rescue_cali->given_current[0] = 1500;
    rescue_cali->given_current[1] = -1500;
    if (rescue_cali->cali_step == 0)
    {
        if (rescue_cali->rescue_motor_measure[0]->ecd == rescue_cali->rescue_motor_measure[0]->last_ecd)
        {
            cali_time++;
        }
        if (cali_time > RESCUE_CALI_TIME)
        {
            rescue_cali->open_ecd_set[0] = rescue_cali->motor_sum_ecd[0] - 2000;
            rescue_cali->open_ecd_set[1] = rescue_cali->motor_sum_ecd[1] + 2000;

            rescue_cali->close_ecd_set[0] = rescue_cali->motor_sum_ecd[0] - 80000;
            rescue_cali->close_ecd_set[1] = rescue_cali->motor_sum_ecd[1] + 80000;
            rescue_cali->cali_step++;
            cali_time = 0;
        }
    }
}

static void Rescue_data_update(rescue_control_e *rescue_update)
{
    uint8_t i = 0;
    for (i = 0; i <= 1; i++)
    {
        if (rescue_update->rescue_motor_measure[i]->count >= 0)
        {
            rescue_update->motor_sum_ecd[i] = rescue_update->rescue_motor_measure[i]->count * 8191 + rescue_update->rescue_motor_measure[i]->ecd;
        }
        if (rescue_update->rescue_motor_measure[i]->count < 0)
        {
            rescue_update->motor_sum_ecd[i] = (rescue_update->rescue_motor_measure[i]->count + 1) * 8191 - (8191 - rescue_update->rescue_motor_measure[i]->ecd);
        }
    }
}

static void Rescue_mode_set(rescue_control_e *rescue_mode_set)
{
    static uint8_t key_rescue_flag = 0;
    static uint8_t key_rescue_time = 0;
    if (rescue_mode_set->cali_step == 0)
    {
        Rescue_cali(rescue_mode_set);
        return;
    }

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
            if (rescue_mode_set->rescue_RC->key.v & RESCUE_KEY && !(rescue_mode_set->rescue_RC->mouse.press_r))
            {
                rescue_mode_set->Claw_mode[0] = OPEN;
                rescue_mode_set->Claw_mode[1] = OPEN;
                key_rescue_time = 200;
                key_rescue_flag = 0;
            }
        }
        else if (!key_rescue_flag)
        {
            if (rescue_mode_set->rescue_RC->key.v & RESCUE_KEY && !(rescue_mode_set->rescue_RC->mouse.press_r))
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
    if (rescue_calc->cali_step == 0)
    {
        return;
    }
    uint8_t i = 0;
    for (i = 0; i <= 1; i++)
    {
        if (rescue_calc->Claw_mode[i] == OPEN)
        {
            PID_Calc(&rescue_calc->rescue_count_pid[i], rescue_calc->motor_sum_ecd[i], rescue_calc->open_ecd_set[i]);
            rescue_calc->given_current[i] = PID_Calc(&rescue_calc->rescue_speed_pid[i], (fp32)rescue_calc->rescue_motor_measure[i]->speed_rpm, rescue_calc->rescue_count_pid[i].out);
        }
        else if (rescue_calc->Claw_mode[i] == CLOSE)
        {
            PID_Calc(&rescue_calc->rescue_count_pid[i], rescue_calc->motor_sum_ecd[i], rescue_calc->close_ecd_set[i]);
            rescue_calc->given_current[i] = PID_Calc(&rescue_calc->rescue_speed_pid[i], (fp32)rescue_calc->rescue_motor_measure[i]->speed_rpm, rescue_calc->rescue_count_pid[i].out);
        }
    }
}
