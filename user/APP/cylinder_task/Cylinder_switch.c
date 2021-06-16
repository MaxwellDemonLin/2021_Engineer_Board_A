
#include "Cylinder_switch.h"
#include "Remote_Control.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

Cylinder_switch_t Cylinder_switch_e;
void Cylinder_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(Cylinder1_GPIO_CLK |
							   Cylinder2_GPIO_CLK | Cylinder3_GPIO_CLK,
						   ENABLE);
	GPIO_InitStructure.GPIO_Pin = Cylinder1_PIN;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;

	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;

	GPIO_Init(Cylinder1_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = Cylinder2_PIN;
	GPIO_Init(Cylinder2_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = Cylinder3_PIN;
	GPIO_Init(Cylinder3_GPIO_PORT, &GPIO_InitStructure);
}

void Cylinder_switch_init(void)
{
	Cylinder_GPIO_Config();
	//Cylinder_switch = get_Cylinder_switch_Measure_Point();

	Cylinder_switch_e.Cylinder1 = 0;
	Cylinder_switch_e.Cylinder2 = 0;
	Cylinder_switch_e.Cylinder3 = 0;
	Cylinder_switch_e.cylinder_rc_ctrl = get_remote_control_point();
	Cylinder1_Close;
	Cylinder2_Close;
	Cylinder3_Close;
}
void Cylinder_switch_control(void)
{
	static uint16_t time = 0;
	if (time)
	{
		time--;
	}
	if (!time)
	{
		if (Cylinder_switch_e.cylinder_rc_ctrl->key.v & KEY_PRESSED_OFFSET_X)
		{
			Cylinder1_TOGGLE;
			Cylinder_switch_e.Cylinder1 = !Cylinder_switch_e.Cylinder1;
			time = 100;
		}
		if (Cylinder_switch_e.cylinder_rc_ctrl->key.v & KEY_PRESSED_OFFSET_C)
		{
			Cylinder2_TOGGLE;
			Cylinder_switch_e.Cylinder2 = !Cylinder_switch_e.Cylinder2;
			time = 100;
		}
		if (Cylinder_switch_e.cylinder_rc_ctrl->key.v & KEY_PRESSED_OFFSET_Z)
		{
			Cylinder3_TOGGLE;
			Cylinder_switch_e.Cylinder3 = !Cylinder_switch_e.Cylinder3;
			time = 100;
		}
	}
}
const Cylinder_switch_t *get_Cylinder_switch_Measure_Point(void)
{
	return &Cylinder_switch_e;
}
void Cylinder_task(void *pvParameters)
{
	Cylinder_switch_init();
	while (1)
	{
		Cylinder_switch_control();
		vTaskDelay(2);
	}
}
