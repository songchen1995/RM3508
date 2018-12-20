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
//int status = 0;


//float track2[20]={
//-27125.08277	,
//-23599.66297	,
//-18139.60693	,
//-11424.32704	,
//-4117.808369	,
//3201.759508	,
//10025.33561	,
//15895.64892	,
//20415.97921	,
//23287.96307	,
//24361.83954	,
//23675.04886	,
//21453.51303	,
//18073.89066	,
//14005.8922	,
//9757.400043	,
//5830.442761	,
//2680.218116	,
//664.1158085	,
//-18.89426166	,
//};

//float track[20] = {
//0.008793576	,
//128.3666364	,
//450.6942171	,
//991.8703953	,
//1686.527187	,
//2369.110414	,
//2795.160888	,
//2688.937913	,
//1804.031017	,
//-18.89426166	,
//-2811.668229	,
//-6462.579928	,
//-10723.51151	,
//-15234.39027	,
//-19562.09232	,
//-23261.72512	,
//-25977.11852	,
//-27570.0363	,
//-28203.59354	,
//-28296.95024,
//};
//float MP[2][2];
//float time2[20]=
//{
//	7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7
//};
//float time[20]={
//	5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,
//};
int flagging = 0;
int main(void)
{
	SpiInit();
	USART3_DMA_Init(921600);
//	ExtiInit();
//	MLX90393_Init();
//	init();
	
		
//	Driver[0].pvtCtrl.desiredPos = track;
//	Driver[0].pvtCtrl.desiredTime = time;
//	MP[0][0] = track[0];
//	MP[0][1] = track[19];
//	MP[1][0] = track2[0];
//	MP[1][1] = track2[19];
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
//	Driver[0].pvtCtrl.flag |= 0x00001000;
//	TIM_Delayms(TIM3,3000);	
//	Driver[0].pvtCtrl.flag &= ~0x00001000;
//	TIM_Delayms(TIM3,3000);	
//	Driver[0].pvtCtrl.flag |= 0x00001000;
	while(1)
	{
			MLX90393_ReadPos();
			if(flagging == 1)
			{
				USART_OUT(USART3,(uint8_t*)"In\r\n");
				flagging = 0;
			}
			TIM_Delayms(TIM3,1);
//		CANRespond();

//		USART_OUT(USART3,(uint8_t*)"%d\t%d\r\n",(int)Driver[0].velCtrl.speed,(int)Driver[0].posCtrl.actualPos);	
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

	

