#include "four_leg.h"
#include "ctrl.h"
#include "string.h"
#include "timer.h"
#include "usart.h"
float track2[20]={
-27125.08277	,
-23599.66297	,
-18139.60693	,
-11424.32704	,
-4117.808369	,
3201.759508	,
10025.33561	,
15895.64892	,
20415.97921	,
23287.96307	,
24361.83954	,
23675.04886	,
21453.51303	,
18073.89066	,
14005.8922	,
9757.400043	,
5830.442761	,
2680.218116	,
664.1158085	,
-18.89426166	,
};

float track[20] = {
0.008793576	,
128.3666364	,
450.6942171	,
991.8703953	,
1686.527187	,
2369.110414	,
2795.160888	,
2688.937913	,
1804.031017	,
-18.89426166	,
-2811.668229	,
-6462.579928	,
-10723.51151	,
-15234.39027	,
-19562.09232	,
-23261.72512	,
-25977.11852	,
-27570.0363	,
-28203.59354	,
-28296.95024,
};


float RaiseUp[] = //单位：角度
{
0,	3,	6,	9,	12,
15,	18,	21,	24,	27,
30,	33,	36,	39,	42,
45,	48,	51,	54,	57,
};
float RaiseDown[] = 
{
59,	54,	51,	48,	45,
42,	39,	36,	33,	30,
27,	24,	21,	18,	15,
12,	9,	6,	3,	0,

};


extern DriverType Driver[8];

/*进行四足机器人的单腿测试，模拟主控*******************************/

void PtStructInit(void)
{
	memset(&Driver[0].ptCtrl,0,sizeof(Driver[0].ptCtrl));
	Driver[0].ptCtrl.velLimit = VEL_MAX_3508;
	Driver[0].ptCtrl.index = 0;
}

void RaiseTest(void)
{
	for(int i = 0; i < 20;i++)
	{
		Driver[0].ptCtrl.desiredPos[POS_FIRST_BUFFER][i] = -(RaiseUp[i] / 360 * 8192)/ KNEE_RATIO / M3508_RATIO;	
	}
	Driver[0].ptCtrl.desiredTime = 50;
	Driver[0].ptCtrl.runMode = CIRCULAR_MODE;
	Driver[0].ptCtrl.size = 20;
	Driver[0].ptCtrl.index = 0;
	SetPtFlag(BEGIN_MOTION);
}

void ExecutorLoadingFirstBufferTest(void)
{
	for(int i = 0; i < 20;i++)
	{
		Driver[0].ptCtrl.desiredPos[POS_FIRST_BUFFER][i] = -(RaiseUp[i] / 360 * 8192)/ KNEE_RATIO / M3508_RATIO;	
	}
	Driver[0].ptCtrl.MP[0] = 0x14003200;//20 size   50 周期
	Driver[0].ptCtrl.index = 0;
	SetPtFlag(EXECUTOR_LOADING_FIRST_BUFFER);
}
//放于while循环当中,测试数组衔接是否正常
void BufferExchangeTest(void)
{
	static int status = 0;
	switch(status)
	{
		case 0:
			Driver[0].ptCtrl.desiredTime = 50;
			Driver[0].ptCtrl.runMode = SINGLE_MODE;
			Driver[0].ptCtrl.size = 20;
			for(int i = 0; i < Driver[0].ptCtrl.size;i++)
			{
				Driver[0].ptCtrl.desiredPos[POS_EXECUTOR][i] = -(RaiseUp[i] / 360.f * 8192.f)/ KNEE_RATIO / M3508_RATIO;	
			}
//			Driver[0].ptCtrl.MP[0] = 0x14003200;//20 size  SINGLE_MODE 50 周期
			SetPtFlag(BEGIN_MOTION);			
			status =1 ;
			break;
		case 1:
			if(CheckPtFlag(ACTION_READY_TO_COMPLETE))	
			{
				for(int i = 0; i < 20;i++)
				{
					Driver[0].ptCtrl.desiredPos[POS_FIRST_BUFFER][i] = -(RaiseDown[i] / 360.f * 8192.f)/ KNEE_RATIO / M3508_RATIO;	
				}
				Driver[0].ptCtrl.MP[0] = 0x14003200;
				SetPtFlag(EXECUTOR_LOADING_FIRST_BUFFER);
			}
			if(CheckPtFlag(ACTION_COMPLETE))
			{
				status = 2;
				SetPtFlag(~ACTION_COMPLETE);
			}
			break;
		case 2:
			if(CheckPtFlag(ACTION_READY_TO_COMPLETE))	
			{
				for(int i = 0; i < 20;i++)
				{
					Driver[0].ptCtrl.desiredPos[POS_FIRST_BUFFER][i] = -(RaiseUp[i] / 360.f * 8192.f)/ KNEE_RATIO / M3508_RATIO;	
				}
				Driver[0].ptCtrl.MP[0] = 0x14013200;
				SetPtFlag(EXECUTOR_LOADING_FIRST_BUFFER);
			}			
			if(CheckPtFlag(ACTION_COMPLETE))
			{
				status = 1;
				SetPtFlag(~ACTION_COMPLETE);
			}
			break;
	}
	TIM_Delayms(TIM3,10);
//	USART_OUT(USART3,(uint8_t*)"%d\r\n",(int)status);
}

//void FirstBufferLoadingSecondBufferTest(void)
//{
//	for(int i = 0; i < 20;i++)
//	{
//		Driver[0].ptCtrl.desiredPos[1][i] = -(RaiseUp[i] / 360 * 8192)/ KNEE_RATIO / M3508_RATIO;	
//	}
//	Driver[0].ptCtrl.MP[0] = 0x14013200;//20 size  CIRCULAR_MODE 50 周期
////	Driver[0].ptCtrl.runMode = CIRCULAR_MODE;
////	Driver[0].ptCtrl.size = 20;
//	Driver[0].ptCtrl.index = 1;
//	SetPtFlag(EXECUTOR_LOADING_FIRST_BUFFER);
//}

void FourLegTest(void)
{
//	for(int i = 0; i < 20; i++)
//	{
//		Driver[0].ptCtrl.desiredPos[2][i] = track[i];	
//	}
//	Driver[0].ptCtrl.desiredTime = 100;
//	Driver[0].ptCtrl.
	
}

