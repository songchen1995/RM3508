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

extern DriverType Driver[8];
extern MotorType Motor[8];

void init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	
	    
	USART3_DMA_Init(921600);
	
	CAN_Config(CAN1,1000,GPIOB,GPIO_Pin_8, GPIO_Pin_9); 
	CAN_Config(CAN2, 500,GPIOB,GPIO_Pin_5, GPIO_Pin_6);

	TIM_Init(TIM2,999,83,0,0);					//主周期定时1ms	
  TIM_Cmd(TIM2,DISABLE);	
  
	DriverInit();
	
  TIM_Cmd(TIM2,ENABLE);	
  
	TIM_Delayms(TIM3,200);
	
	for(int i = 0; i < 8; i++)
	{
		Driver[i].posCtrl.actualPos = 0.0f;
		
		MotorOn(i);
		
		if(Motor[i].type == NONE)
			break;
	}
 
}
int main(void)
{
	init();
	
	while(1)
	{
		CANRespond();
//		VelCtrlTest(300.0f,200);
//  	Driver[1].velCtrl.desiredVel[CMD] = 10.0f;
//		
//		Driver[0].posCtrl.desiredPos = 27.0f*8192.0f;
//		TIM_Delayms(TIM3,1000);
//		Driver[0].posCtrl.desiredPos = -0.0f*8192.0f;
//		TIM_Delayms(TIM3,700);
	}
}

	

