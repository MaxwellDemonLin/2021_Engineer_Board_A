#include "exit_init.h"

#include "stm32f4xx.h"
#include "rescue.h"
extern rescue_control_e rescue_control;

void GPIOB_Exti8_GPIO_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource8);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    EXTI_InitStructure.EXTI_Line = EXTI_Line8;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = MPU_INT_NVIC;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
 /**
  * @brief  配置嵌套向量中断控制器NVIC
  * @param  无
  * @retval 无
  */
static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* 配置NVIC为优先级组1 */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  
  /* 配置中断源：按键1 */
  NVIC_InitStructure.NVIC_IRQChannel = KEY1_INT_EXTI_IRQ;
  /* 配置抢占优先级：1 */
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  /* 配置子优先级：1 */
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  /* 使能中断通道 */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* 配置中断源：按键2，其他使用上面相关配置 */  
  NVIC_InitStructure.NVIC_IRQChannel = KEY2_INT_EXTI_IRQ;
  NVIC_Init(&NVIC_InitStructure);
}


 /**
  * @brief  配置 PA0 为线中断口，并设置中断优先级
  * @param  无
  * @retval 无
  */
void EXTI_Key_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	EXTI_InitTypeDef EXTI_InitStructure;
  
	/*开启按键GPIO口的时钟*/
	RCC_AHB1PeriphClockCmd(KEY1_INT_GPIO_CLK|KEY2_INT_GPIO_CLK ,ENABLE);
  
  /* 使能 SYSCFG 时钟 ，使用GPIO外部中断时必须使能SYSCFG时钟*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  
  /* 配置 NVIC */
  NVIC_Configuration();
  
	/* 选择按键1的引脚 */ 
  GPIO_InitStructure.GPIO_Pin = KEY1_INT_GPIO_PIN;
  /* 设置引脚为输入模式 */ 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	    		
  /* 设置引脚不上拉也不下拉 */
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  /* 使用上面的结构体初始化按键 */
  GPIO_Init(KEY1_INT_GPIO_PORT, &GPIO_InitStructure); 

	/* 连接 EXTI 中断源 到key1引脚 */
  SYSCFG_EXTILineConfig(KEY1_INT_EXTI_PORTSOURCE,KEY1_INT_EXTI_PINSOURCE);

  /* 选择 EXTI 中断源 */
  EXTI_InitStructure.EXTI_Line = KEY1_INT_EXTI_LINE;
  /* 中断模式 */
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  /* 下降沿触发 */
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  /* 使能中断/事件线 */
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  /* 选择按键2的引脚 */ 
  GPIO_InitStructure.GPIO_Pin = KEY2_INT_GPIO_PIN;  
  /* 其他配置与上面相同 */
  GPIO_Init(KEY2_INT_GPIO_PORT, &GPIO_InitStructure);      

	/* 连接 EXTI 中断源 到key2 引脚 */
  SYSCFG_EXTILineConfig(KEY2_INT_EXTI_PORTSOURCE,KEY2_INT_EXTI_PINSOURCE);

  /* 选择 EXTI 中断源 */
  EXTI_InitStructure.EXTI_Line = KEY2_INT_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  /* 上升沿触发 */
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}


void EXTI0_IRQHandler(void)
{
  //确保是否产生了EXTI Line中断
	if(EXTI_GetITStatus(KEY1_INT_EXTI_LINE) != RESET) 
	{
    rescue_control.close_flag[0]=1;
		EXTI_ClearITPendingBit(KEY1_INT_EXTI_LINE);     
	}  

}

void EXTI1_IRQHandler(void)
{
  //确保是否产生了EXTI Line中断
	if(EXTI_GetITStatus(KEY2_INT_EXTI_LINE) != RESET) 
	{
    rescue_control.close_flag[1]=1;
		EXTI_ClearITPendingBit(KEY2_INT_EXTI_LINE);     
	}  
}
