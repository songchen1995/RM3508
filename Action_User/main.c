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

float track2[22]={
-0.004581007	,
790.4993403	,
3167.320395	,
6936.268139	,
11715.36829	,
17044.05348	,
22455.26375	,
27508.41663	,
31806.37182	,
35015.49028	,
36895.0708	,
37329.69802	,
36350.20214	,
34131.41747	,
30966.90198	,
27231.92557	,
23346.77575	,
19743.77189	,
16831.60235	,
14948.05948	,
14302.30654	,
};

int track[22] = {
-65303	,
-65905,
-67590,
-70031,
-72777,
-75336,
-77240,
-78092	,
-77608	,
-75658	,
-72302	,
-67785	,
-62494	,
-56869  ,
-51329	,
-46223,
-41818	,
-38304	,
-35802	,
-34358	,
-33910	,
};
int main(void)
{
	init();
	for(int i = 0;i < 21; i++)
	{
		Driver[0].pvtCtrl.desiredPos[i] = track2[i] ;
		Driver[0].pvtCtrl.desiredTime[i] = 10;
	}
	
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
	while(1)
	{
		
		CANRespond();
		
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

	

