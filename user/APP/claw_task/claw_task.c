#include "claw_task.h"
#include "remote_control.h"
#include "pid.h"
#include "stm32f4xx.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"

Claw_control_e claw_control;
rescue_control_e rescue_control;
void Claw_task_init(Claw_control_e *claw_control_init); //初始化函�?
void claw_rc_to_control_vector(int32_t *ecd, Claw_control_e *claw_rc_to_vector);
static void claw_data_update(Claw_control_e *claw_update);
static void claw_control_PID(Claw_control_e *claw_control);
static void Claw_set_mode(Claw_control_e *claw_control_mode_set);
static void Claw_set_mode(Claw_control_e *claw_control_mode_set);
//static void claw_set_height(Claw_control_e *claw_control_set_control);
static void Claw_cali(Claw_control_e *Claw_cali);
static void timer(Claw_control_e *Claw_cali);



void Rescue_init(rescue_control_e *rescue_init);
static void Rescue_data_update(rescue_control_e *rescue_update);
static void Rescue_cali(rescue_control_e *rescue_cail);
static void Rescue_mode_set(rescue_control_e *rescue_mode_set);
static void Rescue_control_PID(rescue_control_e *rescue_calc);

void Claw_task(void *pvParameters)
{

    Claw_task_init(&claw_control);
		Rescue_init(&rescue_control);
		
    while (1)
    {
        claw_data_update(&claw_control);
        Claw_set_mode(&claw_control);
        claw_control_PID(&claw_control);

				
				Rescue_mode_set	(&rescue_control);
				Rescue_data_update(&rescue_control);
				Rescue_control_PID(&rescue_control);
					
				CAN_CMD_CLAW(claw_control.given_current[0], claw_control.given_current[1],rescue_control.given_current[0],rescue_control.given_current[1]);


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
    if (switch_is_mid(claw_control_mode_set->claw_RC->rc.s[0]))
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
								if(claw_control_mode_set->claw_RC->mouse.press_r)
							{
                if ((claw_control_mode_set->claw_RC->key.v & KEY_PRESSED_OFFSET_G)&& (!(claw_control_mode_set->claw_RC->key.v & KEY_PRESSED_OFFSET_B)))
                {
                    if (claw_control_mode_set->cali_flag)
                    {
                        claw_control_mode_set->claw_mode = CLAW_EXCHANGE;
                        claw_control_mode_set->ecd_sum_set[0] = claw_control_mode_set->down_sum_ecd[0] - FORWARD_HORIZONTAL;
                        claw_control_mode_set->ecd_sum_set[1] = claw_control_mode_set->down_sum_ecd[1] + FORWARD_HORIZONTAL;
                    }
                    time = 0;
                }
                if ((claw_control_mode_set->claw_RC->key.v & KEY_PRESSED_OFFSET_B) && (!(claw_control_mode_set->claw_RC->key.v & KEY_PRESSED_OFFSET_G)))
                {
                    if (claw_control_mode_set->cali_flag)
                    {
                        claw_control_mode_set->claw_mode = CLAW_EXCHANGE;
                        claw_control_mode_set->ecd_sum_set[0] = claw_control_mode_set->down_sum_ecd[0] - BACKWARD_HORIONT;
                        claw_control_mode_set->ecd_sum_set[1] = claw_control_mode_set->down_sum_ecd[1] + BACKWARD_HORIONT;
                    }
                    time = 0;
                }
							}
                if(claw_control_mode_set->claw_RC->key.v & KEY_PRESSED_OFFSET_V)
                {
                    
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
    ecd_add =(int32_t)(ecd_channel * CLAW_HEIGHT_RC_SEN);
if( !claw_rc_to_vector->claw_RC->mouse.press_r)
{
    if (claw_rc_to_vector->claw_RC->key.v & RAW_BACKWARS_KEY && !(claw_rc_to_vector->claw_RC->key.v & RAW_FORWARD_KEY ))
    {
        ecd_add = 10;
    }
    if (claw_rc_to_vector->claw_RC->key.v & RAW_FORWARD_KEY && !(claw_rc_to_vector->claw_RC->key.v & RAW_BACKWARS_KEY ))
    {
        ecd_add = -10;
    }
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
     Claw_cali->given_current[0]=CLAW_CALI_CURRENT_UP;	
		 Claw_cali->given_current[1]=-CLAW_CALI_CURRENT_UP;
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




void Rescue_init(rescue_control_e *rescue_init)
{
    fp32 rescue_speed_pid[3] = {RESCUE_SPEED_KP, RESCUE_SPEED_KI, RESCUE_SPEED_KD};
    fp32 rescue_count_pid[3] = {RESCUE_COUNT_KP, RESCUE_COUNT_KI, RESCUE_COUNT_KD};

	//	fp32 rescue_speed_pid1[3] = {8, RESCUE_SPEED_KI, 0};
  //  fp32 rescue_count_pid1[3] = {0.05, RESCUE_COUNT_KI, 0};
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
    rescue_init->rescue_count_pid[1].proportion_output_filter_coefficient = exp(-10 * 1E-3);

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
            rescue_cali->open_ecd_set[0] = rescue_cali->motor_sum_ecd[0] - 8000;
            rescue_cali->open_ecd_set[1] = rescue_cali->motor_sum_ecd[1] + 8000;

            rescue_cali->close_ecd_set[0] = rescue_cali->motor_sum_ecd[0] - 92000;
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

