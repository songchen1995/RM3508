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
int status = 0;


float track2[20]={0.336901226	,
-390.0473949	,
-1406.815296	,
-3257.329508	,
-6025.562661	,
-9650.535632	,
-13928.08628	,
-18537.06351	,
-23088.09373	,
-27187.76053	,
-30508.76437	,
-32848.29071	,
-34160.52098	,
-34558.23288	,
-34286.35359	,
-33669.48745	,
-33030.72272	,
-32591.06662	,
-32403.83376	,
-32375.61829	,
};

float track[20] = {-33250.9719	,
-35733.51281	,
-39306.77144	,
-43278.1631	,
-46978.18082	,
-49823.3959	,
-51330.10241	,
-51141.10083	,
-49078.08617	,
-45197.82629	,
-39811.11258	,
-33427.42392	,
-26639.03265	,
-20000.30976	,
-13954.4488	,
-8818.136927	,
-4795.704733	,
-1994.118359	,
-417.1679437	,
57.2732085	,
};

float time[40]=
{
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
};

int status;
int main(void)
{
	Driver[0].pvtCtrl.desiredPos = track2;
	Driver[0].pvtCtrl.desiredTime = time;
	init();

//	for(int i = 0;i < 40; i++)
//	{
//		Driver[0].pvtCtrl.desiredPos[i] = track[i] ;
//		Driver[0].pvtCtrl.desiredTime[i] = 10;
//	}
	
//	ZeroPosInit();
//	for(int i = 0; i < 50; i ++)
//	{
//		Driver[0].pvtCtrl.desiredPos[i] = 30.f * 8192.f * sinf(i / 30.f * 2.f * PI);
//		Driver[0].pvtCtrl.desiredTime[i] = 20;
////		USART_OUT(USART3,(uint8_t*)"%d\t%d\r\n",(int)Driver[0].pvtCtrl.desiredPos[i],(int)Driver[0].pvtCtrl.desiredTime[i]);	
//	}
	
//	Driver[0].pvtCtrl.desiredPos[0] = 0;
//	Driver[0].pvtCtrl.desiredPos[1] = 10 * 8192.f;
//	Driver[0].pvtCtrl.desiredPos[2] = 20 * 8192.f;
//	Driver[0].pvtCtrl.desiredPos[3] = 30 * 8192.f;
//	Driver[0].pvtCtrl.desiredPos[4] = 40 * 8192.f;
//	
//	Driver[0].pvtCtrl.desiredPos[5] = 30 * 8192.f;
//	Driver[0].pvtCtrl.desiredPos[6] = 20 * 8192.f;
//	Driver[0].pvtCtrl.desiredPos[7] = 10 * 8192.f;
//	Driver[0].pvtCtrl.desiredPos[8] = 0 * 8192.f;
//	Driver[0].pvtCtrl.desiredPos[9] = 0 * 8192.f;
//	Driver[0].pvtCtrl.desiredVel[0] = 0;
//	Driver[0].pvtCtrl.desiredVel[1] = 100;
//	Driver[0].pvtCtrl.desiredVel[2] = 0 * 8192.f;
	
//	Driver[0].pvtCtrl.desiredTime[0] = 200;
//	Driver[0].pvtCtrl.desiredTime[1] = 200;
//	Driver[0].pvtCtrl.desiredTime[2] = 200;
//	Driver[0].pvtCtrl.desiredTime[3] = 200;
//	Driver[0].pvtCtrl.desiredTime[4] = 200;
//	Driver[0].pvtCtrl.desiredTime[5] = 200;
//	Driver[0].pvtCtrl.desiredTime[6] = 200;
//	Driver[0].pvtCtrl.desiredTime[7] = 200;
//	Driver[0].pvtCtrl.desiredTime[8] = 200;
//	Driver[0].pvtCtrl.desiredTime[9] = 200;	
//	Driver[0].velCtrl.desiredVel[CMD] = 600.0f;

	TIM_Delayms(TIM3,1000);	

	while(1)
	{
		
		CANRespond();
		switch(status)
		{
			case 0:
				if(Driver[0].pvtCtrl.flag == 0x00000001)
				{
						for(int i = 0; i < 20; i++)
						{
							track[i] = 0;
						}
				}
				if(Driver[0].pvtCtrl.flag == 0x0000000)
				{
					for(int i = 0; i < 20; i++)
					{
						track2[i] = 0;
					}
				}
//				Driver[0].pvtCtrl.flag = 1;
//				status = 1;
				break;
			case 1:
				
				break;
		}
//		USART_OUT(USART3,(uint8_t*)"%d\t%d\r\n",(int)Driver[0].velCtrl.speed,(int)Driver[0].posCtrl.actualPos);	
		TIM_Delayms(TIM3,1);
//		Driver[0].posCtrl.desiredPos = 0; 
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

	

