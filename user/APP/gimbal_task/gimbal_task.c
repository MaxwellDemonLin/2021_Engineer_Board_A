#include "gimbal_task.h"
//云台控制所有相关数据
Gimbal_Control_t gimbal_control;

void GIMBAL_Init(Gimbal_Control_t *gimbal_init)
{
    gimbal_init->gimbal_pitch_motor.gimbal_motor_measure = get_Pitch_Gimbal_Motor_Measure_Point();
    gimbal_init->gimbal_yaw_motor.gimbal_motor_measure = get_Yaw_Gimbal_Motor_Measure_Point();
    gimbal_init->gimbal_rc_ctrl = get_remote_control_point();
}

void GIMBAL_data_update(Gimbal_Control_t *gimbal_update)
{
    if (gimbal_update->gimbal_pitch_motor.gimbal_motor_measure->count >= 0)
    {
        gimbal_update->gimbal_pitch_motor.ecd_sum = gimbal_update->gimbal_pitch_motor.gimbal_motor_measure->count * 8191 + gimbal_update->gimbal_pitch_motor.gimbal_motor_measure->ecd;
    }
    else if (gimbal_update->gimbal_pitch_motor.gimbal_motor_measure->count < 0)
    {
        gimbal_update->gimbal_pitch_motor.ecd_sum = gimbal_update->gimbal_pitch_motor.gimbal_motor_measure->count * 8191 - gimbal_update->gimbal_pitch_motor.gimbal_motor_measure->ecd;
    }

    if (gimbal_update->gimbal_yaw_motor.gimbal_motor_measure->count >= 0)
    {
        gimbal_update->gimbal_yaw_motor.ecd_sum = gimbal_update->gimbal_yaw_motor.gimbal_motor_measure->count * 8191 + gimbal_update->gimbal_yaw_motor.gimbal_motor_measure->ecd;
    }
    else if (gimbal_update->gimbal_yaw_motor.gimbal_motor_measure->count < 0)
    {
        gimbal_update->gimbal_yaw_motor.ecd_sum = gimbal_update->gimbal_yaw_motor.gimbal_motor_measure->count * 8191 - gimbal_update->gimbal_yaw_motor.gimbal_motor_measure->ecd;
    }
}
void GIMBAL_set_control(Gimbal_Control_t *gimbal_set_control)
{
    fp32 rc_add_yaw;
    fp32 rc_add_pitch;
    rc_add_yaw = gimbal_set_control->gimbal_rc_ctrl->mouse.x * 0.002;
    rc_add_pitch = gimbal_set_control->gimbal_rc_ctrl->mouse.y * 0.002;



    gimbal_set_control->gimbal_yaw_motor.ecd_sum_set+=r

    if (gimbal_set_control->cali_flag == 1)
    {
        if (gimbal_set_control->gimbal_yaw_motor.ecd_sum + rc_add_yaw > gimbal_set_control->gimbal_yaw_motor.mid_ecd + MAX_YAW_ECD_OFFSET)
        {
            gimbal_set_control->gimbal_yaw_motor.ecd_sum_set = gimbal_set_control->gimbal_yaw_motor.mid_ecd + MAX_YAW_ECD_OFFSET;
        }

        if (gimbal_set_control->gimbal_yaw_motor.ecd_sum + rc_add_yaw < gimbal_set_control->gimbal_yaw_motor.mid_ecd - MIN_YAW_ECD_OFFSET)
        {
            gimbal_set_control->gimbal_yaw_motor.ecd_sum_set = gimbal_set_control->gimbal_yaw_motor.mid_ecd - MIN_YAW_ECD_OFFSET;
        }

        if (gimbal_set_control->gimbal_pitch_motor.ecd_sum + rc_add_pitch > gimbal_set_control->gimbal_pitch_motor.mid_ecd + MAX_PITCH_ECD_OFFSET)
        {
            gimbal_set_control->gimbal_pitch_motor.ecd_sum_set = gimbal_set_control->gimbal_pitch_motor.mid_ecd + MAX_PITCH_ECD_OFFSET;
        }

        if (gimbal_set_control->gimbal_pitch_motor.ecd_sum + rc_add_pitch < gimbal_set_control->gimbal_pitch_motor.mid_ecd - MIN_PITCH_ECD_OFFSET)
        {
            gimbal_set_control->gimbal_pitch_motor.ecd_sum_set = gimbal_set_control->gimbal_pitch_motor.mid_ecd - MIN_PITCH_ECD_OFFSET;
        }
    }
}

void GIMBAL_Cali_control(Gimbal_Control_t *gimbal_cali_control)
{
    static int16_t time = 0;
    if (time)
    {
        time--;
    }
    if (!time)
    {
        if (gimbal_cali_control->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_V)
        {
            gimbal_cali_control->gimbal_yaw_motor.mid_ecd = gimbal_cali_control->gimbal_yaw_motor.ecd_sum;
            gimbal_cali_control->gimbal_pitch_motor.mid_ecd = gimbal_cali_control->gimbal_pitch_motor.ecd_sum;
            time = 200;
            gimbal_cali_control->cali_flag = 1;
        }
    }
}