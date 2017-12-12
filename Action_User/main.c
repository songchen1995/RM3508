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

extern DriverType Driver;

void init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	
	  
	TIM_Init(TIM2,999,83,0,0);					//主周期定时1ms	
	MOTOR_OFF;
	USART3_DMA_Init(115200);

	
	CAN_Config(CAN1,1000,GPIOB,GPIO_Pin_8, GPIO_Pin_9);

	DriverInit();
	
	Driver.UnitMode = SPEED_CONTROL_MODE;
	Driver.VelCtrl.Acc = 0.3f;
	Driver.VelCtrl.Dec = 0.3f;
	Driver.VelCtrl.DesiredVel = 200.0f;
	Driver.PosCtrl.DesiredPos = 3.0f*8192.0f;
	
	MOTOR_ON;
}
int main(void)
{
	init();
	while(1)
	{
		Driver.PosCtrl.DesiredPos = 3.0f*8192.0f;
		TIM_Delayms(TIM3,3000);
		Driver.PosCtrl.DesiredPos = -3.0f*8192.0f;
		TIM_Delayms(TIM3,3000);
	}
}

	




























//		static int aaa,bbb,ccc,order=0;
//		if(KeyCatch())
//		{
//		PosCrl(1,0,aaa);
//		PosCrl(2,0,bbb);
//		PosCrl(3,0,ccc);
//		switch(order)
//		{
//			case 0:
//				aaa-=250;
//			order++;
//			break;
//			case 1:
//				aaa+=250;
//			order++;
//			break;
//			case 2:
//				bbb-=250;
//			order++;
//			break;
//			case 3:
//				bbb+=250;
//			order++;
//			break;
//			case 4:
//				ccc-=250;
//			order++;
//			break;
//			case 5:
//				ccc+=250;
//			order=0;
//			break;
//			
//		}
//}




































