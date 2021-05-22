/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       start_task.c/h
  * @brief      �������񣬽�һ������������������Դ�������������ȼ�,
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "Start_Task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "Detect_Task.h"

#include "User_Task.h"
#include "INS_Task.h"
#include "chassis_task.h"
#include "lifter.h"
#include "rescue.h"
#define INS_TASK_PRIO 20
#define INS_TASK_SIZE 512
static TaskHandle_t INSTask_Handler;

#define GIMBAL_TASK_PRIO 19
#define GIMBAL_STK_SIZE 512
TaskHandle_t GIMBALTask_Handler;

#define Chassis_TASK_PRIO 18
#define Chassis_STK_SIZE 512
TaskHandle_t ChassisTask_Handler;

#define User_TASK_PRIO 4
#define User_STK_SIZE 256
static TaskHandle_t UserTask_Handler;

#define START_TASK_PRIO 1
#define START_STK_SIZE 512
static TaskHandle_t StartTask_Handler;

#define CALIBRATE_TASK_PRIO 5
#define CALIBRATE_STK_SIZE 512
static TaskHandle_t CalibrateTask_Handler;

#define Detect_TASK_PRIO 10
#define Detect_STK_SIZE 512
static TaskHandle_t DetectTask_Handler;

#define Rescue_TASK_PRIO 15
#define Rescue_STK_SIZE 512
static TaskHandle_t RescueTask_Handler;

#define Lift_TASK_PRIO 14
#define Lift_STK_SIZE 512
static TaskHandle_t LiftTask_Handler;
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();



    vTaskDelete(StartTask_Handler); //ɾ����ʼ����
    taskEXIT_CRITICAL();            //�˳��ٽ���
}

void startTask(void)
{

    xTaskCreate((TaskFunction_t)INSTask,
                (const char *)"INSTask",
                (uint16_t)INS_TASK_SIZE,
                (void *)NULL,
                (UBaseType_t)INS_TASK_PRIO,
                (TaskHandle_t *)&INSTask_Handler);
    xTaskCreate((TaskFunction_t)chassis_task,
                (const char *)"ChassisTask",
                (uint16_t)Chassis_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)Chassis_TASK_PRIO,
                (TaskHandle_t *)&ChassisTask_Handler);
    xTaskCreate((TaskFunction_t)UserTask,
                (const char *)"UserTask",
                (uint16_t)User_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)User_TASK_PRIO,
                (TaskHandle_t *)&UserTask_Handler);
    xTaskCreate((TaskFunction_t)DetectTask,
                (const char *)"DetectTask",
                (uint16_t)Detect_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)Detect_TASK_PRIO,
                (TaskHandle_t *)&DetectTask_Handler);
    xTaskCreate((TaskFunction_t)Rescue_task,
                (const char *)"DetectTask",
                (uint16_t)Detect_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)Rescue_TASK_PRIO,
                (TaskHandle_t *)&DetectTask_Handler);
    xTaskCreate((TaskFunction_t)Lift_task,
                (const char *)"DetectTask",
                (uint16_t)Detect_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)Lift_TASK_PRIO,
                (TaskHandle_t *)&DetectTask_Handler);
}
