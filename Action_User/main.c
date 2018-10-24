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
#include "tle5012.h"

extern DriverType Driver[8];
extern MotorType Motor[8];

void init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	
	    
	USART3_DMA_Init(921600);
//	SPI
	CAN_Config(CAN1,1000,GPIOB,GPIO_Pin_8, GPIO_Pin_9); 
	CAN_Config(CAN2, 500,GPIOB,GPIO_Pin_5, GPIO_Pin_6);

	SpiInit();
	delay_ms(1);
	TIM_Init(TIM2,999,83,0,0);					//主周期定时1ms	
	
  TIM_Cmd(TIM2,DISABLE);	
  	
	DriverInit();
		
  	
  TIM_Cmd(TIM2,ENABLE);
	
	TIM_Delayms(TIM3,200);
	for(int i = 0; i < 8; i++)
	{
		if(Motor[i].type == NONE)
			break;
		Driver[i].posCtrl.actualPos = 0;
		MotorOn(i);
	}
	
}
int main(void)
{
	init();
//	Driver[0].velCtrl.desiredVel[CMD] = 8.192f * 180;
//	TIM_Delayms(TIM3,1000);
//	ZeroPosInit();//Please Use it before interupts
	TIM_Delayms(TIM3,5000);
//	delay_ms(10);
//  Driver[0].posCtrl.actualPos = Motor[0].pos - 4000.f;
//	Driver[0].posCtrl.desiredPos = 0.f;
	
	while(1)
	{
		
		CANRespond();
		
		Driver[0].posCtrl.desiredPos = 19 * 8192; 
//		TIM_Delayms(TIM3,500);
//		VelCtrlTest(300.0f,200);
//		Driver[2].velCtrl.desiredVel[CMD] = 1.0f;
//		
//		Driver[2].posCtrl.desiredPos = 3.0f*8192.0f;
//		TIM_Delayms(TIM3,2000);
//		Driver[2].posCtrl.desiredPos = -0.0f*8192.0f;
//		TIM_Delayms(TIM3,2700);
	}
}

	

