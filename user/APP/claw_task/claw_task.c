#include "claw_task.h"
#include "remote_control.h"
#include "pid.h"
#include "stm32f4xx.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

Claw_control_e claw_control;
void Claw_task_init(Claw_control_e *claw_control_init); //初始化函�?
void claw_rc_to_control_vector(int32_t *ecd, Claw_control_e *claw_rc_to_vector);
static void claw_data_update(Claw_control_e *claw_update);
static void claw_control_PID(Claw_control_e *claw_control);
static void Claw_set_mode(Claw_control_e *claw_control_mode_set);
static void Claw_set_mode(Claw_control_e *claw_control_mode_set);
//static void claw_set_height(Claw_control_e *claw_control_set_control);
static void Claw_cali(Claw_control_e *Claw_cali);
static void timer(Claw_control_e *Claw_cali);
void Claw_task(void *pvParameters)
{

    Claw_task_init(&claw_control);
    while (1)
    {
        claw_data_update(&claw_control);
        Claw_set_mode(&claw_control);
        claw_control_PID(&claw_control);
	
        if (claw_control.claw_mode != CLAW_NO_FORCE && claw_control.claw_mode != CLAW_CALI)
        {
            CAN_CMD_CLAW(claw_control.given_current[0], claw_control.given_current[1]);
//						CAN_CMD_CLAW(2000, -2000);
        }
  
        else if (claw_control.claw_mode == CLAW_NO_FORCE)
        {
            CAN_CMD_CLAW(0, 0);
        }
	
        vTaskDelay(2);
    }
}
void Claw_task_init(Claw_control_e *claw_control_init)
{
    fp32 claw_speed_pid[3] = {CLAW_SPEED_KP, CLAW_SPEED_KI, CLAW_SPEED_KD};
    fp32 claw_count_pid[3] = {CLAW_COUNT_KP, CLAW_COUNT_KI, CLAW_COUNT_KD};
    PID_Init(&claw_control_init->claw_speed_pid[0], PID_POSITION, claw_speed_pid, CLAW_SPEED_MAX_OUT, CLAW_SPEED_MAX_IOUT);
    PID_Init(&claw_control_init->claw_speed_pid[1], PID_POSITION, claw_speed_pid, CLAW_SPEED_MAX_OUT, CLAW_SPEED_MAX_IOUT);

    PID_Init(&claw_control_init->claw_height_pid[0], PID_POSITION, claw_count_pid, CLAW_COUNT_MAX_OUT, CLAW_COUNT_MAX_IOUT);
    PID_Init(&claw_control_init->claw_height_pid[1], PID_POSITION, claw_count_pid, CLAW_COUNT_MAX_OUT, CLAW_COUNT_MAX_IOUT);
    claw_control_init->claw_RC = get_remote_control_point();
    claw_control_init->claw_motor_measure[0] = get_Claw_Motor_Measure_Point(0);
    claw_control_init->cali_flag = 0;
    claw_control_init->claw_motor_measure[1] = get_Claw_Motor_Measure_Point(1);
    claw_control_init->claw_mode = CLAW_RAW;
    claw_data_update(&claw_control);
    claw_control_init->ecd_sum_set[0] = claw_control_init->motor_sum_ecd[0];
    claw_control_init->ecd_sum_set[1] = claw_control_init->motor_sum_ecd[1];
}

static void Claw_set_mode(Claw_control_e *claw_control_mode_set)
{
    int32_t ecd_add;

    //计算遥控器的原始输入信号
    static uint16_t time = 500;
    static uint16_t cali_time = 0;
    if (time)
    {
        time--;
    }

    if (claw_control_mode_set->claw_mode == CLAW_CALI)
    {
        return;
    }
    if (switch_is_up(claw_control_mode_set->claw_RC->rc.s[0]))
    {
				claw_control_mode_set->claw_mode = CLAW_RAW;
        if (claw_control_mode_set->claw_RC->key.v & RAW_FORWARD_KEY && claw_control_mode_set->claw_RC->key.v & RAW_BACKWARS_KEY)
        {
            cali_time++;
        }
        if (!(claw_control_mode_set->claw_RC->key.v & RAW_FORWARD_KEY && claw_control_mode_set->claw_RC->key.v & RAW_FORWARD_KEY))
        {
            cali_time = 0;
        }
        if (cali_time == 1000)
        {
            claw_control_mode_set->cali_step = 0;
            claw_control_mode_set->claw_mode = CLAW_CALI;
            cali_time = 0;
            return;
        }

        if (!time)
        {

            claw_rc_to_control_vector(&ecd_add, claw_control_mode_set);
            claw_control_mode_set->ecd_sum_set[0] += ecd_add;
            claw_control_mode_set->ecd_sum_set[1] -= ecd_add;

            if (claw_control.cali_step == 1)
            {
                if (claw_control_mode_set->claw_RC->key.v & KEY_PRESSED_OFFSET_G && claw_control_mode_set->claw_RC->mouse.press_r)
                {
                    if (claw_control_mode_set->cali_flag)
                    {
                        claw_control_mode_set->claw_mode = CLAW_EXCHANGE;
                        claw_control_mode_set->ecd_sum_set[0] = claw_control_mode_set->down_sum_ecd[0] - FORWARD_HORIZONTAL;
                        claw_control_mode_set->ecd_sum_set[1] = claw_control_mode_set->down_sum_ecd[1] + FORWARD_HORIZONTAL;
                    }
                    cali_time = 0;
                }
                if (claw_control_mode_set->claw_RC->key.v & KEY_PRESSED_OFFSET_B && claw_control_mode_set->claw_RC->mouse.press_r)
                {
                    if (claw_control_mode_set->cali_flag)
                    {
                        claw_control_mode_set->claw_mode = CLAW_EXCHANGE;
                        claw_control_mode_set->ecd_sum_set[0] = claw_control_mode_set->down_sum_ecd[0] - BACKWARD_HORIONT;
                        claw_control_mode_set->ecd_sum_set[1] = claw_control_mode_set->down_sum_ecd[1] + BACKWARD_HORIONT;
                    }
                    cali_time = 0;
                }
            }
        }
		}
		  if (switch_is_mid(claw_control_mode_set->claw_RC->rc.s[0]))
			{
							claw_control_mode_set->claw_mode = CLAW_RAW;
				 claw_rc_to_control_vector(&ecd_add, claw_control_mode_set);
         claw_control_mode_set->ecd_sum_set[0] += ecd_add;
         claw_control_mode_set->ecd_sum_set[1] -= ecd_add;
			}
        if (switch_is_down(claw_control_mode_set->claw_RC->rc.s[0]))
        {
            claw_control_mode_set->claw_mode = CLAW_NO_FORCE;
          //  claw_control_mode_set->ecd_sum_set[0] = claw_control_mode_set->down_sum_ecd[0]; //为了防止有力无力切换时上升发生抖动，因此将设定值设置为0
           // claw_control_mode_set->ecd_sum_set[1] = claw_control_mode_set->down_sum_ecd[1];
        }
    
}
static void claw_rc_to_control_vector(int32_t *ecd, Claw_control_e *claw_rc_to_vector)
{
    fp32 ecd_channel;
    rc_deadline_limit(claw_rc_to_vector->claw_RC->rc.ch[4], ecd_channel, 10);
    int32_t ecd_add = 0;
    ecd_add = claw_rc_to_vector->claw_RC->rc.ch[4] * CLAW_HEIGHT_RC_SEN;
    if (claw_rc_to_vector->claw_RC->key.v & RAW_BACKWARS_KEY && !claw_rc_to_vector->claw_RC->key.v & RAW_FORWARD_KEY && !claw_rc_to_vector->claw_RC->mouse.press_r)
    {
        ecd_add = 100;
    }
    if (claw_rc_to_vector->claw_RC->key.v & RAW_FORWARD_KEY && !claw_rc_to_vector->claw_RC->key.v & RAW_BACKWARS_KEY&& !claw_rc_to_vector->claw_RC->mouse.press_r)
    {
        ecd_add = -100;
    }
    if (claw_rc_to_vector->cali_flag == 1)
    {
        if (ecd_add > 0)
        {
            if (claw_rc_to_vector->ecd_sum_set[0] + ecd_add > claw_rc_to_vector->down_sum_ecd[0])
            {
                claw_rc_to_vector->ecd_sum_set[0] = claw_rc_to_vector->down_sum_ecd[0];
                claw_rc_to_vector->ecd_sum_set[1] = claw_rc_to_vector->down_sum_ecd[1];
                return;
            }
        }
        if (ecd_add < 0)
        {
            if (claw_rc_to_vector->ecd_sum_set[0] + ecd_add < claw_rc_to_vector->down_sum_ecd[0] - UP_ECD_ADD)
            {
                claw_rc_to_vector->ecd_sum_set[0] = claw_rc_to_vector->down_sum_ecd[0] - UP_ECD_ADD;
                claw_rc_to_vector->ecd_sum_set[1] = claw_rc_to_vector->down_sum_ecd[1] + UP_ECD_ADD;
                return;
            }
        }
        *ecd = ecd_add;
    }
    else
        *ecd = ecd_add;
}
static void claw_control_PID(Claw_control_e *claw_control)
{
    if (claw_control->claw_mode == CLAW_CALI)
    {
        Claw_cali(claw_control);
        return;
    }

    PID_Calc(&claw_control->claw_height_pid[0], claw_control->motor_sum_ecd[0], claw_control->ecd_sum_set[0]);
    PID_Calc(&claw_control->claw_height_pid[1], claw_control->motor_sum_ecd[1], claw_control->ecd_sum_set[1]);

    claw_control->given_current[0] = (int16_t)PID_Calc(&claw_control->claw_speed_pid[0], claw_control->claw_motor_measure[0]->speed_rpm, claw_control->claw_height_pid[0].out);
    claw_control->given_current[1] = (int16_t)PID_Calc(&claw_control->claw_speed_pid[1], claw_control->claw_motor_measure[1]->speed_rpm, claw_control->claw_height_pid[1].out);
}
static void claw_data_update(Claw_control_e *claw_update)
{

    uint8_t i = 0;
    for (i = 0; i <= 1; i++)
    {
        if (claw_update->claw_motor_measure[i]->count >= 0)
        {
            claw_update->motor_sum_ecd[i] = claw_update->claw_motor_measure[i]->count * ecd_range + claw_update->claw_motor_measure[i]->ecd;
        }
        if (claw_update->claw_motor_measure[i]->count < 0)
        {
            claw_update->motor_sum_ecd[i] = claw_update->claw_motor_measure[i]->count * ecd_range + claw_update->claw_motor_measure[i]->ecd;
        }
    }
}

static void Claw_cali(Claw_control_e *Claw_cali)
{
    static uint16_t cali_time = 0;
    if (Claw_cali->cali_step == 0)
    {
        CAN_CMD_CLAW(-CLAW_CALI_CURRENT_DOWN, CLAW_CALI_CURRENT_DOWN);
        if (Claw_cali->claw_motor_measure[0]->ecd == Claw_cali->claw_motor_measure[0]->last_ecd)
        {
            cali_time++;
        }
        if (cali_time > CALI_TIME)
        {
            Claw_cali->down_sum_ecd[0] = Claw_cali->motor_sum_ecd[0] - 2000;
            Claw_cali->down_sum_ecd[1] = Claw_cali->motor_sum_ecd[1] + 2000;
            Claw_cali->ecd_sum_set[0] = Claw_cali->motor_sum_ecd[0];
            Claw_cali->ecd_sum_set[1] = Claw_cali->motor_sum_ecd[1];
            Claw_cali->exchange_sum_ecd[0] = Claw_cali->motor_sum_ecd[0] - EXCHANGE_ADD;
            Claw_cali->exchange_sum_ecd[1] = Claw_cali->motor_sum_ecd[1] + EXCHANGE_ADD;
            Claw_cali->cali_step++;
            Claw_cali->claw_mode = CLAW_RAW;
            Claw_cali->cali_flag = 1;
            cali_time = 0;
        }
    }
}
const Claw_control_e *get_claw_measure(void)
{
    return &claw_control;
}
