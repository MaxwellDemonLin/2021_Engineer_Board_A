#include "lifter.h"
#include "remote_control.h"
#include "pid.h"
Lift_control_e lift_control;
void Lift_task_init(Lift_control_e *lift_control_init); //初始化函数
void lift_rc_to_control_vector(fp32 *height, Lift_control_e *lift_rc_to_vector);
static void lift_data_update(Lift_control_e *lift_update);
static void lift_control_PID(Lift_control_e *lift_control);
static void Lift_set_mode(Lift_control_e *lift_control_mode_set);
static void Lift_set_mode(Lift_control_e *lift_control_mode_set);
static void lift_set_height(Lift_control_e *lift_control_set_control);
void Lift_task(void *pvParameters)
{
    Lift_task_init(&lift_control);
    while (1)
    {
        Lift_set_mode(&lift_control);
        lift_set_height(&lift_control);
        lift_data_update(&lift_control);
        lift_control_PID(&lift_control);

        CAN_CMD_LIFTER(lift_control.given_current[0],lift_control.given_current[1]);
    }
}
void Lift_task_init(Lift_control_e *lift_control_init)
{
    lift_control_init->lift_RC = get_remote_control_point();
    lift_control_init->lift_motor_measure[0] = get_Lifter_Motor_Measure_Point(0);

    lift_control_init->lift_motor_measure[1] = get_Lifter_Motor_Measure_Point(1);
    lift_control_init->height_set = 0;
}
static void Lift_set_mode(Lift_control_e *lift_control_mode_set)
{
    //计算遥控器的原始输入信号
    static uint16_t time = 500;
    if (time)
    {
        time--;
    }
    if (switch_is_mid(lift_control_mode_set->lift_RC->rc.s[0]))
    {
        if (!time)
        { //判断是否要摇摆

            if ((lift_control_mode_set->lift_RC->key.v & EXCHANGE_KEY))
            {
                lift_control_mode_set->lift_mode = LIFT_EXCHANGE;
                time = 500;
            }
            else if ((lift_control_mode_set->lift_RC->key.v & LOOT_KEY))
            {
                lift_control_mode_set->lift_mode = LIFT_LOOT;
                time = 500;
            }
            else if ((lift_control_mode_set->lift_RC->key.v & LARGE_ISLAND_KEY))
            {
                lift_control_mode_set->lift_mode = LIFT_LARGE_RESOURCE_ISLAND;
                time = 500;
            }
            else if ((lift_control_mode_set->lift_RC->key.v & SMALL_ISLAND_KEY))
            {
                lift_control_mode_set->lift_mode = LIFT_SMALL_RESOURCE_ISLAND;
                time = 500;
            }
            else if(lift_control_mode_set->lift_RC->key.v & DOWN_KEY)
            {
                lift_control_mode_set->lift_mode = LIFT_DOWN;
            }
        }
        if (lift_control_mode_set->ecd_count[1] == lift_control_mode_set->motor_count_set && lift_control_mode_set->ecd_count[0] == lift_control_mode_set->motor_count_set)
        {
            lift_control_mode_set->lift_mode = LIFT_RAW;
        }
    }
    if (switch_is_down(lift_control_mode_set->lift_RC->rc.s[0]))
    {
        lift_control_mode_set->lift_mode = LIFT_NO_FORCE;
    }
}
//设置上升高度

static void lift_set_height(Lift_control_e *lift_control_set_control)
{
    fp32 height_set;
    if (lift_control_set_control->lift_mode == LIFT_RAW)
    {
        lift_rc_to_control_vector(&height_set, lift_control_set_control);
    }
    else if (lift_control_set_control->lift_mode == LIFT_LOOT)
    {
        height_set = LOOT_HEIGHT;
    }
    else if (lift_control_set_control->lift_mode == LIFT_LARGE_RESOURCE_ISLAND)
    {
        height_set = LARGE_ISLAND_HEIGHT;
    }
    else if (lift_control_set_control->lift_mode == LIFT_SMALL_RESOURCE_ISLAND)
    {
        height_set = SMALL_ISLAND_HEIGHT;
    }
    else if (lift_control_set_control->lift_mode == LIFT_EXCHANGE)
    {
        height_set = EXCHANGE_HEIGHT;
    }
    else if (lift_control_set_control->lift_mode == LIFT_NO_FORCE)
    {
        height_set = 0;
    }
    lift_control_set_control->height_set = height_set;
}
static void lift_rc_to_control_vector(fp32 *height, Lift_control_e *lift_rc_to_vector)
{
    fp32 height_channel;
    fp32 height_channel_add;
    //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadline_limit(lift_rc_to_vector->lift_RC->rc.ch[1], height_channel, 10);

    height_channel_add = -height_channel * LIFT_HEIGHT_RC_SEN;
    *height = *height + height_channel_add;
}
static void lift_control_PID(Lift_control_e *lift_control)
{
    lift_control->motor_count_set = (lift_control->height_set / WHEEL_PERIMETER) * 19;
    PID_Calc(&lift_control->lift_height_pid[0], lift_control->ecd_count[0], lift_control->motor_count_set);
    PID_Calc(&lift_control->lift_height_pid[1], lift_control->ecd_count[1], lift_control->motor_count_set);

    lift_control->given_current[0]=PID_Calc(&lift_control->lift_speed_pid[0], lift_control->lift_motor_measure[0]->speed_rpm, lift_control->lift_height_pid[0].out);
    lift_control->given_current[1]=PID_Calc(&lift_control->lift_speed_pid[1], lift_control->lift_motor_measure[1]->speed_rpm, lift_control->lift_height_pid[1].out);

}
static void lift_data_update(Lift_control_e *lift_update)
{
    if (lift_update->lift_motor_measure[0]->ecd - lift_update->lift_motor_measure[0]->last_ecd > Half_ecd_range)
    {
        lift_update->motor_count[0]--;
    }
    else if (lift_update->lift_motor_measure[0]->ecd - lift_update->lift_motor_measure[0]->last_ecd < -Half_ecd_range)
    {
        lift_update->motor_count[0]++;
    }

    if (lift_update->lift_motor_measure[1]->ecd - lift_update->lift_motor_measure[1]->last_ecd > Half_ecd_range)
    {
        lift_update->motor_count[1]--;
    }
    else if (lift_update->lift_motor_measure[1]->ecd - lift_update->lift_motor_measure[1]->last_ecd < -Half_ecd_range)
    {
        lift_update->motor_count[1]++;
    }
}

