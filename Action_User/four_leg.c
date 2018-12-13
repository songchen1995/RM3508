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

float RaiseTestBuf[12]=
{
	0.355,
	1.34,
	3.0151,
	5.36,
	8.375,
	12.06,
	15.74,
	18.76,
	21.105,
	22.78,
	23.785,
	24.12,
};

//float RaiseTestBuf[13]=
//{
//	0.4,
//	1.6,
//	3.6,
//	6.4,
//	10,
//	14.4,
//	18.8,
//	22.4,
//	25.2,
//	27.2,
//	28.4,
//	28.8,
//	28.8
//};

//float RaiseTestBuf[12]=
//{
//	0.5,
//	2,
//	4.5,
//	8,
//	12.5,
//	18,
//	23.5,
//	28,
//	31.5,
//	34,
//	35.5,
//	36,
//};


float RaiseUp[] = //单位：角度
{
0,	3,	6,	9,	12,
15,	18,	21,	24,	27,
30,	33,	36,	39,	42,
45,	48,	51,	54,	57,
};
float RaiseDown[] = 
{
57,	54,	51,	48,	45,
42,	39,	36,	33,	30,
27,	24,	21,	18,	15,
12,	9,	6,	3,	0,

};

float ResetBuffer[]=
{
	34.00,
	35.41,
	35.41,
	35.41,
	35.41,
};


float slf1_1[] = 
{
0	,
-1.9025	,
-1.7601	,
0.2986	,
3.7797	,
8.0975	,
12.7402	,
17.3184	,
21.5436	,
25.196	,
28.1067	,
30.1489	,
31.236	,
31.3216	,
30.4069	,
28.539	,
25.8067	,
22.3395	,
18.3097	,
13.9395	,
};

float slf1_2[] = 
{
9.5169	,
5.4182	,
2.1208	,
0.1611	,
-0.027	,
1.6859	,
4.1053	,
6.2139	,
8.052	,
9.6489	,
11.0256	,
12.1985	,
13.1791	,
13.9762	,
14.5965	,
15.0451	,
15.3252	,
15.439	,
15.3871	,
15.1693	,
};

float slf1_3[]=
{
14.784	,
14.2283	,
13.498	,
12.5869	,
11.4871	,
10.1878	,
8.6747	,
6.9289	,
4.9245	,
2.6258	,
0	,
};


float slf2_1[]=
{
	0	,
0.144	,
0.7077	,
1.6846	,
2.8756	,
4.0254	,
4.906	,
5.346	,
5.2281	,
4.4791	,
3.0674	,
1.0019	,
-1.6702	,
-4.8692	,
-8.4876	,
-12.4014	,
-16.4827	,
-20.6036	,
-24.6364	,
-28.4456	,
};

float slf2_2[]=
{
	-31.8748	,
-34.7299	,
-36.7659	,
-37.702	,
-37.2913	,
-35.4229	,
-32.9038	,
-30.5136	,
-28.234	,
-26.0523	,
-23.96	,
-21.9506	,
-20.0203	,
-18.1666	,
-16.3882	,
-14.6846	,
-13.056	,
-11.5036	,
-10.029	,
-8.6344	,

};

float slf2_3[]=
{
-7.3227	,
-6.0972	,
-4.9619	,
-3.9218	,
-2.9825	,
-2.1511	,
-1.4359	,
-0.8478	,
-0.4005	,
-0.1127	,
0	,

};



extern DriverType Driver[8];

/*进行四足机器人的单腿测试，模拟主控*******************************/

void PtStructInit(void)
{
	memset(&Driver[0].ptCtrl,0,sizeof(Driver[0].ptCtrl));
	Driver[0].posCtrl.actualPos = 0;
	Driver[0].posCtrl.desiredPos = 0;
	Driver[0].ptCtrl.velLimit = 100;
	Driver[0].ptCtrl.index = 0;
}

void RaiseTest(void)
{
	Driver[0].ptCtrl.desiredTime = 10;
	Driver[0].ptCtrl.runMode = RUN_AND_STOP_MOTION_MODE;
	Driver[0].ptCtrl.size = 12;
	Driver[0].ptCtrl.index = 0;
	Driver[0].ptCtrl.velLimit = VEL_MAX_3508;	
	for(int i = 0; i < Driver[0].ptCtrl.size 	;i++)
	{
		Driver[0].ptCtrl.desiredPos[POS_EXECUTOR][i] = -(RaiseTestBuf[i] / 360 * 8192) * COAXE_RATIO * M3508_RATIO;	
//		Driver[0].ptCtrl.desiredPos[POS_EXECUTOR][i] = -(RaiseTestBuf[i] / 360 * 8192) * KNEE_RATIO * M3508_RATIO;			
	}

	SetPtFlag(BEGIN_MOTION);
	SetPtFlag(NEW_DATA);
}



void ExecutorLoadingFirstBufferTest(void)
{
	for(int i = 0; i < 20;i++)
	{
		Driver[0].ptCtrl.desiredPos[POS_FIRST_BUFFER][i] = -(RaiseUp[i] / 360 * 8192) * COAXE_RATIO * M3508_RATIO;	
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
			Driver[0].ptCtrl.desiredTime = 10;
			Driver[0].ptCtrl.runMode = RUN_AND_STOP_MOTION_MODE;
			Driver[0].ptCtrl.size = 20;
			for(int i = 0; i < Driver[0].ptCtrl.size;i++)
			{
				Driver[0].ptCtrl.desiredPos[POS_EXECUTOR][i] = -(RaiseUp[i] / 360.f * 8192.f) * COAXE_RATIO * M3508_RATIO;	
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
					Driver[0].ptCtrl.desiredPos[POS_FIRST_BUFFER][i] = -(RaiseDown[i] / 360.f * 8192.f) * COAXE_RATIO * M3508_RATIO;	
				}
				Driver[0].ptCtrl.MP[0] = 0x14003200;
				SetPtFlag(EXECUTOR_LOADING_FIRST_BUFFER);
			}
			if(CheckPtFlag(ACTION_COMPLETE ))
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
					Driver[0].ptCtrl.desiredPos[POS_FIRST_BUFFER][i] = -(RaiseUp[i] / 360.f * 8192.f) * COAXE_RATIO * M3508_RATIO;	
				}
				Driver[0].ptCtrl.MP[0] = 0x14000A00;
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



void ResetTest()
{
	Driver[0].ptCtrl.velLimit = 100;
	Driver[0].ptCtrl.desiredTime = 200;
	Driver[0].ptCtrl.runMode = RUN_AND_STOP_MOTION_MODE;
	Driver[0].ptCtrl.size = 5;
	for(int i = 0; i < Driver[0].ptCtrl.size;i++)
	{
		Driver[0].ptCtrl.desiredPos[POS_EXECUTOR][i] = -(ResetBuffer[i] / 360.f * 8192.f) * COAXE_RATIO * M3508_RATIO;	
	}
	SetPtFlag(BEGIN_MOTION);
	SetPtFlag(NEW_DATA);
}

void ResetInit(void)
{
	Driver[0].ptCtrl.desiredTime = 0;
	Driver[0].ptCtrl.runMode = RUN_AND_STOP_MOTION_MODE;
	Driver[0].ptCtrl.size = 0;
	Driver[0].ptCtrl.executeFlag = 0;
	Driver[0].ptCtrl.index = 0;
	Driver[0].ptCtrl.velLimit = 100;
}


void FourLegTest(void)
{
//	for(int i = 0; i < 20; i++)
//	{
//		Driver[0].ptCtrl.desiredPos[2][i] = track[i];	
//	}
//	Driver[0].ptCtrl.desiredTime = 100;
//	Driver[0].ptCtrl.
	
}

