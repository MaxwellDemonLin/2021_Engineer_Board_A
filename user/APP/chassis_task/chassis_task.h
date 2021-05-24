#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "main.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "Remote_Control.h"
#include "user_lib.h"



void chassis_task(void *pvParameters);
//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357

//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 1
//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 0
//在特殊模式下，可以通过遥控器控制旋转
#define CHASSIS_WZ_CHANNEL 2

//选择底盘状态 开关通道号
#define MODE_CHANNEL 0
//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.006f
//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.005f
//跟随底盘yaw模式下，遥控器的yaw遥杆（max 660）增加到车体角度的比例
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN 0.01f

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

#define CHASSIS_RC_DEADLINE 10

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

#define MOTOR_DISTANCE_TO_CENTER 0.2f

//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002
//底盘任务控制频率，尚未使用这个宏
#define CHASSIS_CONTROL_FREQUENCE 500.0f
//底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 16000.0f
//底盘摇摆按键
#define SWING_KEY KEY_PRESSED_OFFSET_CTRL
//底盘前后左右控制按键
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D

//m3508转化成底盘速度(m/s)的比例，做两个宏 是因为可能换电机需要更换比例
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//底盘电机最大速度
#define MAX_WHEEL_SPEED 2.2f
//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 1.7f
//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 1.4f
//底盘设置旋转速度，设置前后左右轮不同设定速度的比例分权 0为在几何中心，不需要补偿
#define CHASSIS_WZ_SET_SCALE 0.0f

//摇摆原地不动摇摆最大角度(rad)
#define SWING_NO_MOVE_ANGLE 0.7f
//摇摆过程底盘运动最大角度(rad)
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f

//底盘电机速度环PID
#define M3505_MOTOR_SPEED_PID_KP 15000.0f
#define M3505_MOTOR_SPEED_PID_KI 10.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//底盘旋转跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 15.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.03f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 6.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f

typedef enum
{
  CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,
  CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,
  CHASSIS_VECTOR_NO_FOLLOW_YAW,
  CHASSIS_VECTOR_RAW,
  CHASSIS_TOP,
  //  CHASSIS_AUTO,
  //  CHASSIS_FOLLOW_YAW,
  //  CHASSIS_ENCODER,
  //  CHASSIS_NO_ACTION,
  //  CHASSIS_RELAX,
} chassis_mode_e;

typedef enum
{
  CHASSIS_ZERO_FORCE,                   //chassis will be like no power,底盘无力, 跟没上电那样
  CHASSIS_NO_MOVE,                      //chassis will be stop,底盘保持不动
  CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW,   //chassis will follow gimbal, usually in infantry,正常步兵底盘跟随云台
  CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW,  //chassis will follow chassis yaw angle, usually in engineer,
                                        //because chassis does have gyro sensor, its yaw angle is calculed by gyro in gimbal and gimbal motor angle,
                                        //if you have a gyro sensor in chassis, please updata yaw, pitch, roll angle in "chassis_feedback_update"  function
                                        //工程底盘角度控制底盘，由于底盘未有陀螺仪，故而角度是减去云台角度而得到，
                                        //如果有底盘陀螺仪请更新底盘的yaw，pitch，roll角度 在chassis_feedback_update函数中
  CHASSIS_NO_FOLLOW_YAW,                //chassis does not follow angle, angle is open-loop,but wheels have closed-loop speed
                                        //底盘不跟随角度，角度是开环的，但轮子是有速度环
  CHASSIS_OPEN                          //the value of remote control will mulitiply a value, get current value that will be sent to can bus
                                        // 遥控器的值乘以比例成电流值 直接发送到can总线上
} chassis_behaviour_e;

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} Chassis_Motor_t;

typedef struct
{
    const motor_measure_t *gimbal_motor_measure;

    fp32 max_relative_angle; //rad
    fp32 min_relative_angle; //rad

    fp32 relative_angle;     //rad
    fp32 relative_angle_set; //rad
		fp32 relative_angle_move_set;
	
		fp32 vision_error_set;   //unit in pixel
		fp32 vision_error;
		fp32 vision_angle;				//pixel
		fp32 vision_angle_set;
	
    fp32 absolute_angle;     //rad
    fp32 absolute_angle_set; //rad
    fp32 motor_gyro;         //rad/s
    fp32 motor_gyro_set;
    fp32 motor_speed;
    fp32 raw_cmd_current;
    fp32 current_set;
    int16_t given_current;

} Gimbal_Motor_t;

typedef struct
{
  const RC_ctrl_t *chassis_RC;               
  const Gimbal_Motor_t *chassis_yaw_motor;   
  const Gimbal_Motor_t *chassis_pitch_motor; 
  const fp32 *chassis_INS_angle;            
  chassis_mode_e chassis_mode;               
  chassis_mode_e last_chassis_mode;          
  Chassis_Motor_t motor_chassis[4];          
  PidTypeDef motor_speed_pid[4];             
  PidTypeDef chassis_angle_pid;             

  first_order_filter_type_t chassis_cmd_slow_set_vx;
  first_order_filter_type_t chassis_cmd_slow_set_vy;

  fp32 vx;                         
  fp32 vy;                         
  fp32 wz;                        
  fp32 vx_set;                     
  fp32 vy_set;                     
  fp32 wz_set;                     
  fp32 chassis_relative_angle;     
  fp32 chassis_relative_angle_set; 
  fp32 chassis_yaw_set;

  fp32 vx_max_speed;  
  fp32 vx_min_speed;  
  fp32 vy_max_speed;  
  fp32 vy_min_speed;  
  fp32 chassis_yaw;   
  fp32 chassis_pitch; 
  fp32 chassis_roll;  


	uint8_t swing_flag;
 
} chassis_move_t;



#endif
