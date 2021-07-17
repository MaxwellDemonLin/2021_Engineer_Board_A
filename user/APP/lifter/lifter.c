#include "lifter.h"
#include "remote_control.h"
#include "pid.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
Lift_control_e lift_control;
void Lift_task_init(Lift_control_e *lift_control_init); //初始化函�?
void lift_rc_to_control_vector(int32_t *ecd, Lift_control_e *lift_rc_to_vector);
static void lift_data_update(Lift_control_e *lift_update);
static void lift_control_PID(Lift_control_e *lift_control);
static void Lift_set_mode(Lift_control_e *lift_control_mode_set);
static void Lift_set_mode(Lift_control_e *lift_control_mode_set);
//static void lift_set_height(Lift_control_e *lift_control_set_control);
static void Lift_cali(Lift_control_e *Lift_cali);
void Lift_task(void *pvParameters)
{

    Lift_task_init(&lift_control);
    while (1)
    {
        Lift_cali(&lift_control);
        lift_data_update(&lift_control);
        if (lift_control.cali_step == 1)
        {
            Lift_set_mode(&lift_control);
            lift_control_PID(&lift_control);
            if (lift_control.lift_mode != LIFT_NO_FORCE)
            {
               CAN_CMD_LIFTER(1200 + lift_control.given_current[0], -1200 + lift_control.given_current[1]);
						//		CAN_CMD_LIFTER(500,500);
            }
            else if (lift_control.lift_mode == LIFT_NO_FORCE)
            {
                CAN_CMD_LIFTER(0, 0);
            }
        }
        vTaskDelay(2);
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

    //计算遥控器的原始输入信号
    static uint16_t time = 500;
    static uint16_t cali_time = 0;
    if (time)
    {
        time--;
    }
    if (switch_is_up(lift_control_mode_set->lift_RC->rc.s[0]))
    {
        if (!time)
        { //判断是否要摇�?

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
		}
}
//设置上升高度

/*static void lift_set_height(Lift_control_e *lift_control_set_control)
{

    int32_t ecd_sum_set;
    int32_t ecd_add;
    if (lift_control_set_control->lift_mode == LIFT_RAW)
    {
        lift_rc_to_control_vector(&ecd_add, lift_control_set_control);
        lift_control_set_control->ecd_sum_set[0] += ecd_add;
        lift_control_set_control->ecd_sum_set[1] -= ecd_add;
    }
    if (lift_control_set_control->lift_mode == LIFT_LOOT)
    {
        lift_control_set_control->ecd_sum_set[0] = lift_control_set_control->down_sum_ecd[0] + LOOT_HEIGHT_ECD;
        lift_control_set_control->ecd_sum_set[0] = lift_control_set_control->down_sum_ecd[1] - LOOT_HEIGHT_ECD;
    }
    else if (lift_control_set_control->lift_mode == LIFT_LARGE_RESOURCE_ISLAND)
    {
        lift_control_set_control->ecd_sum_set[0] = lift_control_set_control->down_sum_ecd[0] + LARGE_ISLAND_HEIGHT_ECD;
        lift_control_set_control->ecd_sum_set[0] = lift_control_set_control->down_sum_ecd[1] - LARGE_ISLAND_HEIGHT_ECD;
    }
    else if (lift_control_set_control->lift_mode == LIFT_SMALL_RESOURCE_ISLAND)
    {
        lift_control_set_control->ecd_sum_set[0] = lift_control_set_control->down_sum_ecd[0] + SMALL_ISLAND_HEIGHT_ECD;
        lift_control_set_control->ecd_sum_set[0] = lift_control_set_control->down_sum_ecd[1] - SMALL_ISLAND_HEIGHT_ECD;
    }
    else if (lift_control_set_control->lift_mode == LIFT_EXCHANGE)
    {
        lift_control_set_control->ecd_sum_set[0] = lift_control_set_control->down_sum_ecd[0] + EXCHANGE_HEIGHT_ECD;
        lift_control_set_control->ecd_sum_set[0] = lift_control_set_control->down_sum_ecd[1] - EXCHANGE_HEIGHT_ECD;
    }
    else if (lift_control_set_control->lift_mode == LIFT_DOWN)
    {
        lift_control_set_control->ecd_sum_set[0] = lift_control_set_control->down_sum_ecd[0];
        lift_control_set_control->ecd_sum_set[0] = lift_control_set_control->down_sum_ecd[1];
    }
    else if (lift_control_set_control->lift_mode == LIFT_NO_FORCE)
    {
        ecd_sum_set = 0;
    }
}*/
static void lift_rc_to_control_vector(int32_t *ecd, Lift_control_e *lift_rc_to_vector)
{
    fp32 ecd_channel;

    rc_deadline_limit(lift_rc_to_vector->lift_RC->rc.ch[3], ecd_channel, 10);

    *ecd = ecd_channel * LIFT_HEIGHT_RC_SEN;
}
static void lift_control_PID(Lift_control_e *lift_control)
{
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
    if (Lift_cali->cali_step == 0)
    {
        CAN_CMD_LIFTER(-LIFT_CALI_CURRENT_DOWN, LIFT_CALI_CURRENT_DOWN);
        if (Lift_cali->lift_motor_measure[0]->ecd == Lift_cali->lift_motor_measure[0]->last_ecd)
        {
            cali_time++;
        }
        if (cali_time > CALI_TIME)
        {
            Lift_cali->down_sum_ecd[0] = Lift_cali->motor_sum_ecd[0]+20000;
            Lift_cali->down_sum_ecd[1] = Lift_cali->motor_sum_ecd[1]-20000;
            Lift_cali->ecd_sum_set[0] = Lift_cali->motor_sum_ecd[0];
            Lift_cali->ecd_sum_set[1] = Lift_cali->motor_sum_ecd[1];
            Lift_cali->cali_step++;
            cali_time = 0;
        }
    }
}
