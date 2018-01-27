#include "stm32f4xx.h"

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_it.h"
#include "timer.h"
#include "gpio.h"
#include "usart.h"
#include "misc.h"
#include "can.h"
#include "elmo.h"
#include "math.h"
#include "stdio.h"
#include "arm_math.h"
#include "ctrl.h"
#include "comm.h"

extern DriverType Driver;

void init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	
	    
	USART3_DMA_Init(115200);
	
	CAN_Config(CAN1,1000,GPIOB,GPIO_Pin_8, GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5, GPIO_Pin_6);

	TIM_Init(TIM2,999,83,0,0);					//主周期定时1ms	
  TIM_Cmd(TIM2,DISABLE);	
  
	DriverInit();
	
//	TIM_Delayms(TIM3,1000);
  TIM_Cmd(TIM2,ENABLE);	
	
	MotorOn();
 
}
int main(void)
{
	init();
	while(1)
	{
		CANRespond();
//		Driver.PosCtrl.DesiredPos = 3.0f*8192.0f;
//		TIM_Delayms(TIM3,3000);
//		Driver.PosCtrl.DesiredPos = -3.0f*8192.0f;
//		TIM_Delayms(TIM3,3000);
	}
}

	

