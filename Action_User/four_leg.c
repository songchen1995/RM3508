#include "four_leg.h"
#include "ctrl.h"
#include "string.h"
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


float RaiseUp[] = 
{
	0,
	5,
	10,
	15,
	20,
	25,
	30,
	35,
	40,
	45,
	45,
	40,
	35,
	30,
	25,
	20,
	15,
	10,
	5,
	0
};

extern DriverType *Driver;

/*进行四足机器人的单腿测试，模拟主控*******************************/

void PtStructInit(void)
{
	memset(&Driver[0].ptCtrl,0,sizeof(Driver[0].ptCtrl));
	Driver[0].ptCtrl.velLimit = 100.f;
}

void RaiseTest(void)
{
	for(int i = 0; i < 20;i++)
	{
		Driver[0].ptCtrl.desiredPos[0][i] = -RaiseUp[i] / KNEE_RATIO / M3508_RATIO;	
	}
	Driver[0].ptCtrl.desiredTime = 200;
	Driver[0].ptCtrl.runMode = CIRCULAR_MODE;
	Driver[0].ptCtrl.size = 20;
	SetPtFlag(BEGIN_MOTION);
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

