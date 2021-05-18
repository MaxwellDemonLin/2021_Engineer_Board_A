#include "chassis_task.h"

#include "rc.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "arm_math.h"

#include "CAN_Receive.h"
#include "Detect_Task.h"
#include "pid.h"

#include "Remote_Control.h"
#include "INS_Task.h"
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

chassis_move_t chassis_move;
static void chassis_init(chassis_move_t *chassis_move_init);
static void chassis_set_mode(chassis_move_t *chassis_move_mode);
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4]);
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);
/**
  * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
  * @author         RM
  * @param[in]      vx_set前进的速度 设定值将直接发送到can总线上
  * @param[in]      vy_set左右的速度 设定值将直接发送到can总线上
  * @param[in]      wz_set旋转的速度 设定值将直接发送到can总线上
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
  * @author         RM
  * @param[in]      vx_set前进的速度
  * @param[in]      vy_set左右的速度
  * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          底盘跟随云台的行为状态机下，底盘模式是跟随云台角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
  * @author         RM
  * @param[in]      vx_set前进的速度
  * @param[in]      vy_set左右的速度
  * @param[in]      angle_set底盘与云台控制到的相对角度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          底盘跟随底盘yaw的行为状态机下，底盘模式是跟随底盘角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
  * @author         RM
  * @param[in]      vx_set前进的速度
  * @param[in]      vy_set左右的速度
  * @param[in]      angle_set底盘设置的yaw，范围 -PI到PI
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

static void chassis_engineer_follow_chassis_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
  * @author         RM
  * @param[in]      vx_set前进的速度
  * @param[in]      vy_set左右的速度
  * @param[in]      wz_set底盘设置的旋转速度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
  * @author         RM
  * @param[in]      vx_set前进的速度
  * @param[in]      vy_set左右的速度
  * @param[in]      wz_set底盘设置的旋转速度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
//主任务
void chassis_task(void *pvParameters)
{
    //空闲一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    //底盘初始化
    chassis_init(&chassis_move);

    while (1)
    {
        //遥控器设置状态
        chassis_set_mode(&chassis_move);
        //遥控器状态切换数据保存
        chassis_mode_change_control_transit(&chassis_move);
        //底盘数据更新
        chassis_feedback_update(&chassis_move);
        //底盘控制量设置
        chassis_set_contorl(&chassis_move);
        //底盘控制PID计算
        chassis_control_loop(&chassis_move);

        if (1)
        {
            //当遥控器掉线的时候，为relax状态，底盘电机指令为零，为了保证一定发送为零，故而不采用设置give_current的方法
            if (toe_is_error(DBUSTOE))
            {
                CAN_CMD_CHASSIS(0, 0, 0, 0);
            }
            else
            {
                CAN_CMD_CHASSIS(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
                       							chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
            }
        }
        //系统延时
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
    }
}
static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }

    //底盘速度环pid值
    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
    //底盘旋转环pid值
    const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    uint8_t i;

    //底盘开机状态为停止
    chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
    //获取遥控器指针
    chassis_move_init->chassis_RC = get_remote_control_point();
    //获取陀螺仪姿态角指针
    chassis_move_init->chassis_INS_angle = get_INS_angle_point();
    //获取云台电机数据指针
    //  chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
    //  chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();
    //初始化PID 运动
    for (i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);
        PID_Init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }
    //初始化旋转PID
    PID_Init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
    //用一阶滤波代替斜波函数生成
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);

    //最大 最小速度
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

    //更新一下数据
    chassis_feedback_update(chassis_move_init);
}
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
   if (chassis_move_mode == NULL)
    {
        return;
    }

    //遥控器设置行为模式
    if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW;
    }
    else if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW;
    }
    else if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;
    }
}

static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
    if (chassis_move_transit == NULL)
    {
        return;
    }

    if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode)
    {
        return;
    }

    //切入跟随云台模式
    if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_transit->chassis_relative_angle_set = 0.0f;
    }
    //切入跟随底盘角度模式
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }
    //切入不跟随云台模式
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }

    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }

    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        //更新电机速度，加速度是速度的PID微分
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
        chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
    }

    //更新底盘前进速度 x， 平移速度y，旋转速度wz，坐标系为右手系
    chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

    //计算底盘姿态角度, 如果底盘上有陀螺仪请更改这部分代码
    //chassis_move_update->chassis_yaw = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET) - chassis_move_update->chassis_yaw_motor->relative_angle);
    //chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET) - chassis_move_update->chassis_pitch_motor->relative_angle);
    //chassis_move_update->chassis_roll = *(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);
}
static void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{

    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_zero_force_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {
        chassis_no_move_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
        chassis_infantry_follow_gimbal_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW)
    {
        chassis_engineer_follow_chassis_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        chassis_no_follow_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
}


static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{

    if (chassis_move_control == NULL)
    {
        return;
    }

         //设置速度
        fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
        chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);
		PidTypeDef angle;
		angle.Kp=3;
		angle.Ki=0;
		angle.Kd=0.1;
	
		vx_set=-vx_set;
		vy_set=-vy_set;
		
		if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        //旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
        sin_yaw = arm_sin_f32(chassis_move_control->chassis_yaw_motor->relative_angle);
        cos_yaw = arm_cos_f32(chassis_move_control->chassis_yaw_motor->relative_angle);
        chassis_move_control->vx_set = -(cos_yaw * vx_set + sin_yaw * vy_set);
        chassis_move_control->vy_set = -(-sin_yaw * vx_set + cos_yaw * vy_set);
        //设置控制相对云台角度
        chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);
        //计算旋转PID角速度
        chassis_move_control->wz_set = -PID_Calc(&chassis_move_control->chassis_angle_pid, chassis_move_control->chassis_yaw_motor->relative_angle, chassis_move_control->chassis_relative_angle_set);
        //速度限幅
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
				
	}
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        fp32 delat_angle = 0.0f;
        //放弃跟随云台
        //设置底盘控制的角度
        chassis_move_control->chassis_yaw_set = rad_format(angle_set);
        delat_angle = rad_format(chassis_move_control->chassis_yaw_set - chassis_move_control->chassis_yaw);
        //计算旋转的角速度
        chassis_move_control->wz_set = PID_Calc(&chassis_move_control->chassis_angle_pid, 0.0f, delat_angle);
        //设置底盘运动的速度
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {

        //放弃跟随云台
        //这个模式下，角度设置的为 角速度
        fp32 chassis_wz = angle_set;
        chassis_move_control->wz_set = chassis_wz;
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        chassis_move_control->vx_set = vx_set;
        chassis_move_control->vy_set = vy_set;
        chassis_move_control->wz_set = angle_set;
        chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
        chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
    }
}

static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;
    //麦轮运动分解
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
                                          chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);

    if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        //赋值电流值
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(wheel_speed[i]);
        }
        //raw控制直接返回
        return;
    }

    //计算轮子控制最大速度，并限制其最大速度
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
        temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);
        if (max_vector < temp)
        {
            max_vector = temp;
        }
    }

    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
        }
    }

    //计算无故障pid
	if (!(toe_is_error(ChassisMotor1TOE) || toe_is_error(ChassisMotor2TOE) || toe_is_error(ChassisMotor3TOE) || toe_is_error(ChassisMotor4TOE)))
		{
    for (i = 0; i < 4; i++)
    {
        PID_Calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
    }
		

    //赋值电流值
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
    }
	}
		//故障pid
	else if(!(toe_is_error(ChassisMotor1TOE) && toe_is_error(ChassisMotor2TOE) && toe_is_error(ChassisMotor3TOE) && toe_is_error(ChassisMotor4TOE))) //判断是否掉线
	{
		for (i = 0; i < 4; i++)
    {
				if(chassis_move_control_loop->motor_chassis[i].speed_set)
					{
						PID_Calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
						chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
					}
				else if(!(chassis_move_control_loop->motor_chassis[i].speed_set))
				{
					chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(0);
				}
    }
	}
}

static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{		
		if(toe_is_error(ChassisMotor1TOE))
		{
			
			if(wz_set==0)
			{
			wheel_speed[0] =0;
			wheel_speed[1] =0;
			wheel_speed[2] = vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
			wheel_speed[3] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
				return ;
			}
			if(wz_set!=0)
			{
			wheel_speed[0] =0;
			wheel_speed[1] =vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
			wheel_speed[2] =0;
			wheel_speed[3] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
				return ;
			}
			
		}
		if(toe_is_error(ChassisMotor2TOE))
		{
			
			if(wz_set==0)
			{
			wheel_speed[0] =0;
			wheel_speed[1] =0;
			wheel_speed[2] = vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
			wheel_speed[3] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
				return ;
			}
			if(wz_set!=0)
			{
			wheel_speed[0] = -vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
			wheel_speed[1] =0;
			wheel_speed[2] = vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
			wheel_speed[3] =0;
				return ;
			}
			
		}
		if(toe_is_error(ChassisMotor3TOE))
		{
			
			if(wz_set==0)
			{
		wheel_speed[0] = -vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[1] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[2] = 0;
    wheel_speed[3] =0;
				return ;
			}
			if(wz_set!=0)
			{
			wheel_speed[0] = 0;
    wheel_speed[1] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[2] = 0;
    wheel_speed[3] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
				return ;
			}
			
		}
		if(toe_is_error(ChassisMotor4TOE))
		{
			
			if(wz_set==0)
			{
			wheel_speed[0] = -vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[1] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[2] = 0;
    wheel_speed[3] =0;
				return ;
			}
			if(wz_set!=0)
			{
		wheel_speed[0] = -vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[1] = 0;
    wheel_speed[2] = vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[3] = 0;
				return ;
			}
		}
    //旋转的时候， 由于云台靠前，所以是前面两轮 0 ，1 旋转的速度变慢， 后面两轮 2,3 旋转的速度变快
    wheel_speed[0] = -vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[1] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[2] = vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[3] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
		
	
}
/**
  * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
  * @author         RM
  * @param[in]      vx_set前进的速度 设定值将直接发送到can总线上
  * @param[in]      vy_set左右的速度 设定值将直接发送到can总线上
  * @param[in]      wz_set旋转的速度 设定值将直接发送到can总线上
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_can_set == NULL || vy_can_set == NULL || wz_can_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_can_set = 0.0f;
    *vy_can_set = 0.0f;
    *wz_can_set = 0.0f;
}
/**
  * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
  * @author         RM
  * @param[in]      vx_set前进的速度
  * @param[in]      vy_set左右的速度
  * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_set = 0.0f;
    *vy_set = 0.0f;
    *wz_set = 0.0f;
}

/**
  * @brief          底盘跟随云台的行为状态机下，底盘模式是跟随云台角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
  * @author         RM
  * @param[in]      vx_set前进的速度
  * @param[in]      vy_set左右的速度
  * @param[in]      angle_set底盘与云台控制到的相对角度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
	
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

		
}
/**
  * @brief          底盘跟随底盘yaw的行为状态机下，底盘模式是跟随底盘角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
  * @author         RM
  * @param[in]      vx_set前进的速度
  * @param[in]      vy_set左右的速度
  * @param[in]      angle_set底盘设置的yaw，范围 -PI到PI
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */


static void chassis_engineer_follow_chassis_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

    *angle_set = rad_format(chassis_move_rc_to_vector->chassis_yaw_set - CHASSIS_ANGLE_Z_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL]);
}

/**
  * @brief          底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
  * @author         RM
  * @param[in]      vx_set前进的速度
  * @param[in]      vy_set左右的速度
  * @param[in]      wz_set底盘设置的旋转速度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
    *wz_set = -CHASSIS_WZ_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL];
}





//遥控器的数据处理成底盘的前进vx速度，vy速度
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }
    //遥控器原始通道值
    int16_t vx_channel, vy_channel;
    fp32 vx_set_channel, vy_set_channel;
    //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

    vx_set_channel = -vx_channel * CHASSIS_VX_RC_SEN;
    vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;

    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
    {
        vx_set_channel = -chassis_move_rc_to_vector->vx_max_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
    {
        vx_set_channel =  -chassis_move_rc_to_vector->vx_min_speed;
    }

    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
    {
        vy_set_channel =  chassis_move_rc_to_vector->vy_max_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
    {
        vy_set_channel =  chassis_move_rc_to_vector->vy_min_speed;
    }

    //一阶低通滤波代替斜波作为底盘速度输入
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);

    //停止信号，不需要缓慢加速，直接减速到零
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}