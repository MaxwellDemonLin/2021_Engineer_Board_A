#include "gimbal_task.h"
#include "pid.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "CAN_receive.h"
#include "math.h"
//云台控制所有相关数据
Gimbal_Control_t gimbal_control;

void GIMBAL_Init(Gimbal_Control_t *gimbal_init);
void GIMBAL_data_update(Gimbal_Control_t *gimbal_update);
void GIMBAL_set_control(Gimbal_Control_t *gimbal_set_control);
static void GIMBAL_PID_Cali(Gimbal_Control_t *gimbal_PID_cali);
void Gimbal_Task(void *pvParameters)
{
    GIMBAL_Init(&gimbal_control);
    while (1)
    {
        GIMBAL_data_update(&gimbal_control);
        GIMBAL_set_control(&gimbal_control);
        GIMBAL_PID_Cali(&gimbal_control);
        if(gimbal_control.GIMBAL_mode!=NO_FORCE)
        {
            CAN_CMD_GIMBAL(gimbal_control.gimbal_yaw_motor.given_current,gimbal_control.gimbal_pitch_motor.given_current);
        }
        else if(gimbal_control.GIMBAL_mode==NO_FORCE)
        {
            CAN_CMD_GIMBAL(0,0);
        }
        
    }
    
}
void GIMBAL_Init(Gimbal_Control_t *gimbal_init)
{
    
    fp32 gimbal_speed_pid[3] = {GIMBAL_SPEED_KP, GIMBAL_SPEED_KI, GIMBAL_SPEED_KD};
    fp32 gimbal_ecd_pid[3] = {GIMBAL_ECD_KP, GIMBAL_ECD_KI, GIMBAL_ECD_KD};
    PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_ecd_pid,PID_POSITION,gimbal_ecd_pid,GIMBAL_ECD_MAX_OUT,GIMBAL_ECD_MAX_IOUT);
    PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_ecd_pid,PID_POSITION,gimbal_ecd_pid,GIMBAL_ECD_MAX_OUT,GIMBAL_ECD_MAX_IOUT);
    PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_speed_pid,PID_POSITION,gimbal_speed_pid,GIMBAL_SPEED_MAX_OUT,GIMBAL_SPEED_MAX_IOUT);
    PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_speed_pid,PID_POSITION,gimbal_speed_pid,GIMBAL_SPEED_MAX_OUT,GIMBAL_SPEED_MAX_IOUT);
    gimbal_init->gimbal_pitch_motor.gimbal_motor_measure = get_Pitch_Gimbal_Motor_Measure_Point();
    gimbal_init->gimbal_yaw_motor.gimbal_motor_measure = get_Yaw_Gimbal_Motor_Measure_Point();
    gimbal_init->gimbal_rc_ctrl = get_remote_control_point();
    	gimbal_init->gimbal_pitch_motor.gimbal_ecd_pid.derivative_output_filter_coefficient= exp(-0.5*1E-3);
		gimbal_init->gimbal_pitch_motor.gimbal_ecd_pid.proportion_output_filter_coefficient= exp(-50*1E-3);

		gimbal_init->gimbal_yaw_motor.gimbal_ecd_pid.derivative_output_filter_coefficient= exp(-0.5*1E-3);
		gimbal_init->gimbal_yaw_motor.gimbal_ecd_pid.proportion_output_filter_coefficient= exp(-50*1E-3);	
		

		gimbal_init->gimbal_pitch_motor.gimbal_speed_pid.derivative_output_filter_coefficient= exp(-0.05*1E-3);
		gimbal_init->gimbal_pitch_motor.gimbal_speed_pid.proportion_output_filter_coefficient= exp(-400*1E-3);

		gimbal_init->gimbal_yaw_motor.gimbal_speed_pid.derivative_output_filter_coefficient= exp(-0.05*1E-3);
		gimbal_init->gimbal_yaw_motor.gimbal_speed_pid.proportion_output_filter_coefficient= exp(-400*1E-3);	
}

void GIMBAL_data_update(Gimbal_Control_t *gimbal_update)
{
    if (&gimbal_update->gimbal_pitch_motor.gimbal_motor_measure->ecd >= 0)
    {
        gimbal_update->gimbal_pitch_motor.ecd_sum = gimbal_update->gimbal_pitch_motor.gimbal_motor_measure->ecd * 8191 + gimbal_update->gimbal_pitch_motor.gimbal_motor_measure->ecd;
    }
    else if (&gimbal_update->gimbal_pitch_motor.gimbal_motor_measure->ecd < 0)
    {
        gimbal_update->gimbal_pitch_motor.ecd_sum = gimbal_update->gimbal_pitch_motor.gimbal_motor_measure->ecd * 8191 + gimbal_update->gimbal_pitch_motor.gimbal_motor_measure->ecd;
    }

    if (&gimbal_update->gimbal_yaw_motor.gimbal_motor_measure->ecd >= 0)
    {
        gimbal_update->gimbal_yaw_motor.ecd_sum = gimbal_update->gimbal_yaw_motor.gimbal_motor_measure->ecd * 8191 + gimbal_update->gimbal_yaw_motor.gimbal_motor_measure->ecd;
    }
    else if (&gimbal_update->gimbal_yaw_motor.gimbal_motor_measure->ecd < 0)
    {
        gimbal_update->gimbal_yaw_motor.ecd_sum = gimbal_update->gimbal_yaw_motor.gimbal_motor_measure->ecd * 8191 + gimbal_update->gimbal_yaw_motor.gimbal_motor_measure->ecd;
    }
}
void GIMBAL_set_control(Gimbal_Control_t *gimbal_set_control)
{
    fp32 rc_add_yaw;
    fp32 rc_add_pitch;
    rc_add_yaw = gimbal_set_control->gimbal_rc_ctrl->mouse.x * 0.002;
    rc_add_pitch = gimbal_set_control->gimbal_rc_ctrl->mouse.y * 0.002;

    gimbal_set_control->gimbal_yaw_motor.ecd_sum_set+=rc_add_yaw;
    gimbal_set_control->gimbal_pitch_motor.ecd_sum_set+=rc_add_pitch;

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

static void GIMBAL_PID_Cali(Gimbal_Control_t *gimbal_PID_cali)
{
    int16_t pitch_speed_set;
    int16_t yaw_speed_set;

    pitch_speed_set=PID_Calc(&gimbal_PID_cali->gimbal_pitch_motor.gimbal_ecd_pid,gimbal_PID_cali->gimbal_pitch_motor.ecd_sum,gimbal_PID_cali->gimbal_pitch_motor.ecd_sum_set);
    yaw_speed_set=PID_Calc(&gimbal_PID_cali->gimbal_yaw_motor.gimbal_ecd_pid,gimbal_PID_cali->gimbal_yaw_motor.ecd_sum,gimbal_PID_cali->gimbal_yaw_motor.ecd_sum_set);
    
    gimbal_PID_cali->gimbal_pitch_motor.given_current=(int16_t)PID_Calc(&gimbal_PID_cali->gimbal_pitch_motor.gimbal_speed_pid,gimbal_PID_cali->gimbal_pitch_motor.gimbal_motor_measure->speed_rpm,pitch_speed_set);
    gimbal_PID_cali->gimbal_yaw_motor.given_current=(int16_t)PID_Calc(&gimbal_PID_cali->gimbal_yaw_motor.gimbal_speed_pid,gimbal_PID_cali->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm,yaw_speed_set);
    
}