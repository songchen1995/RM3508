#include "four_leg.h"
#include "ctrl.h"
#include "string.h"
#include "timer.h"
#include "usart.h"
#include "comm.h"



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
1,	2,	3,	4,	5,
6,	7,	8,	9,	10,
11,	12,	13,	14,	15,
16,	17,	18,	19,	20,

};
float RaiseDown[] = 
{
20,	19,	18,	17,	16,
15,	14,	13,	12,	11,
10,	9,	8,	7,	6,
5,	4,	3,	2,	1,
};	

float coaxe_resetBuffer[]=
{
	34.00,
	35.41,
	35.41,
	35.41,
	35.41,
};

float knee_resetBuffer[]=
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







float thigh[20]=
{

}
extern DriverType Driver[8];

/*进行四足机器人的单腿测试，模拟主控*******************************/


/**
  * @brief  CanSendData
  * @param 
  * @param 
  * @retval 
  */
void PtCanHandler(uint8_t motorNum,UnionDataType RxData)
{
	static uint8_t status = 1;
	static uint8_t N = 0;
	if(!CheckPtFlag(motorNum,RECEIVE_START_AND_MP|RECEIVE_QN))
	{
		status = 1;
	}
	switch(status)
	{
		case 1: 
			SetPtFlag(motorNum,~ACTION_COMPLETE);
			SetPtFlag(motorNum,SECOND_BUFFER_LOADING_CAN_BUFFER);
			SetPtFlag(motorNum,RECEIVE_START_AND_MP);
			Driver[motorNum].ptCtrl.MP[1] = RxData.data32[1];
			status = 2;
			SetPtFlag(motorNum,~RECEIVE_START_AND_MP);
			SetPtFlag(motorNum,RECEIVE_QN);
			break;
		case 2:
			
			if(RxData.data32[0] & 0xB0000000)
			{
				Driver[motorNum].ptCtrl.desiredPos[POS_SECOND_BUFFER][N] = ((RxData.data32[0] << 4) | ( RxData.data32[1] >> 28));  
				N++;
				if(N < Driver[motorNum].ptCtrl.MP[1])
				{
					Driver[motorNum].ptCtrl.desiredPos[POS_SECOND_BUFFER][N] = (RxData.data32[1] << 4) >> 4;  
					N++;
				}
				else//若是在接收数组的过程当中重新从START开始，则二级缓存可以被擦写
				{
					status = 0;
					N = 0;
					SetPtFlag(motorNum,~RECEIVE_QN);
					SetPtFlag(motorNum,RECEIVE_BEGIN);
					SetPtFlag(motorNum,~SECOND_BUFFER_LOADING_CAN_BUFFER);
					SetPtFlag(motorNum,FIRST_BUFFER_LOADING_SECOND_BUFFER);
					Driver[motorNum].ptCtrl.MP[0] = Driver[0].ptCtrl.MP[1];	
					Driver[motorNum].ptCtrl.MP[1] = 0;	
				}
			}	
			else //做错误判断用
			{
		
			}				
			break;
		default:
			break;
	}
}

void PtSecondBufferHandler(uint8_t motorNum)
{
	if(CheckPtFlag(motorNum,FIRST_BUFFER_LOADING_SECOND_BUFFER))
	{
		for(int i = 0; i< Driver[motorNum].ptCtrl.MP[1]; i++)
		{
			Driver[motorNum].ptCtrl.desiredPos[POS_FIRST_BUFFER][i] = Driver[motorNum].ptCtrl.desiredPos[POS_SECOND_BUFFER][i];
			Driver[motorNum].ptCtrl.desiredPos[POS_SECOND_BUFFER][i] = 0;
		}
		SetPtFlag(motorNum,~RECEIVE_BEGIN);
		SetPtFlag(motorNum,~FIRST_BUFFER_LOADING_SECOND_BUFFER);
		SetPtFlag(motorNum,EXECUTOR_LOADING_FIRST_BUFFER);
	}
}

void PtFirstBufferHandler(uint8_t motorNum)//接收完上级数组后将上级数组清空
{
	if(!CheckPtFlag(motorNum,BEGIN_MOTION))
	{
		if(CheckPtFlag(motorNum,EXECUTOR_LOADING_FIRST_BUFFER))
		{
			Driver[motorNum].ptCtrl.size =  Driver[motorNum].ptCtrl.MP[0] >> 24;
			Driver[motorNum].ptCtrl.runMode =  (Driver[motorNum].ptCtrl.MP[0]<<8) >> 24;
			Driver[motorNum].ptCtrl.desiredTime =  (Driver[motorNum].ptCtrl.MP[0]<<16) >> 24;
			Driver[motorNum].ptCtrl.MP[0] = 0;
			for(int i = 0; i< Driver[motorNum].ptCtrl.size; i++)
			{
				Driver[motorNum].ptCtrl.desiredPos[POS_EXECUTOR][i] = Driver[motorNum].ptCtrl.desiredPos[POS_FIRST_BUFFER][i];//一级缓冲加载
				Driver[motorNum].ptCtrl.desiredPos[POS_FIRST_BUFFER][i] = 0;//一级缓存数组清空
			}
			SetPtFlag(motorNum,BEGIN_MOTION);//执行器执行
			SetPtFlag(motorNum,~EXECUTOR_LOADING_FIRST_BUFFER);	
			SetPtFlag(motorNum,~ACTION_READY_TO_COMPLETE);
			Driver[motorNum].ptCtrl.index = 0;
		}
		else
		{
			if(Driver[motorNum].ptCtrl.runMode == CIRCULAR_MODE)//循环走
			{
				SetPtFlag(motorNum,BEGIN_MOTION);
				Driver[motorNum].ptCtrl.index = 0;
				Driver[motorNum].ptCtrl.cnt = 0;
			}
			else if(Driver[motorNum].ptCtrl.runMode == SINGLE_MODE)//运行完后以原速度继续向前跑
			{
//				SetPtFlag(BEGIN_MOTION);
				
			}
			else if(Driver[motorNum].ptCtrl.runMode == RUN_AND_STOP_MOTION_MODE)//运行完后立即停下；
			{
				Driver[motorNum].ptCtrl.velOutput = 0;
				
//				if(Driver[motorNum].ptCtrl.size > 0)
					Driver[motorNum].ptCtrl.posOutput = (Driver[motorNum].ptCtrl.kp * 4.f * (Driver[motorNum].ptCtrl.desiredPos[POS_EXECUTOR][Driver[motorNum].ptCtrl.size - 1] - Driver[motorNum].posCtrl.actualPos)) ;
				
				Driver[motorNum].ptCtrl.output = 0;
				Driver[motorNum].ptCtrl.index = 0;
			}
		}
		SetPtFlag(motorNum,~ACTION_READY_TO_COMPLETE);
	}
}


void PtStructInit(void)
{
	memset(&Driver[COAXE_MOTOR_NUM].ptCtrl,0,sizeof(Driver[COAXE_MOTOR_NUM].ptCtrl));
	Driver[COAXE_MOTOR_NUM].posCtrl.actualPos = 0;
	Driver[COAXE_MOTOR_NUM].ptCtrl.velLimit = VEL_MAX_3508;
	Driver[COAXE_MOTOR_NUM].ptCtrl.index = 0;
	Driver[COAXE_MOTOR_NUM].ptCtrl.size = 0;
	Driver[COAXE_MOTOR_NUM].ptCtrl.kp = 0.01f;
	Driver[COAXE_MOTOR_NUM].ptCtrl.kd = 0.35f;
	Driver[COAXE_MOTOR_NUM].ptCtrl.ki = 0.0f;
	Driver[COAXE_MOTOR_NUM].velCtrl.desiredVel[CMD] = 0;
	Driver[COAXE_MOTOR_NUM].ptCtrl.pulseMaxLimit = COAXE_MAX_ANGLE_PULSE;
	Driver[COAXE_MOTOR_NUM].ptCtrl.pulseMinLimit = COAXE_MIN_ANGLE_PULSE;
	
	memset(&Driver[KNEE_MOTOR_NUM].ptCtrl,0,sizeof(Driver[KNEE_MOTOR_NUM].ptCtrl));
	Driver[KNEE_MOTOR_NUM].posCtrl.actualPos = 0;
	Driver[KNEE_MOTOR_NUM].ptCtrl.velLimit = VEL_MAX_3508;
	Driver[KNEE_MOTOR_NUM].ptCtrl.index = 0;
	Driver[KNEE_MOTOR_NUM].ptCtrl.size = 0;
	Driver[KNEE_MOTOR_NUM].ptCtrl.kp = 0.01f;
	Driver[KNEE_MOTOR_NUM].ptCtrl.kd = 0.35f;
	Driver[KNEE_MOTOR_NUM].ptCtrl.ki = 0.0f;
	Driver[KNEE_MOTOR_NUM].velCtrl.desiredVel[CMD] = 0;
	
	Driver[KNEE_MOTOR_NUM].ptCtrl.pulseMaxLimit = KNEE_MAX_ANGLE_PULSE;
	Driver[KNEE_MOTOR_NUM].ptCtrl.pulseMinLimit = KNEE_MIN_ANGLE_PULSE;
}

void RaiseTest(uint8_t motorNum)
{
	Driver[motorNum].ptCtrl.desiredTime = 10;
	Driver[motorNum].ptCtrl.runMode = RUN_AND_STOP_MOTION_MODE;
	Driver[motorNum].ptCtrl.size = 12;
	Driver[motorNum].ptCtrl.index = 0;
	Driver[motorNum].ptCtrl.velLimit = VEL_MAX_3508;	
	for(int i = 0; i < Driver[motorNum].ptCtrl.size;i++)
	{
		Driver[motorNum].ptCtrl.desiredPos[POS_EXECUTOR][i] = -(RaiseTestBuf[i] / 360 * 8192) * COAXE_RATIO * M3508_RATIO;	
//		Driver[0].ptCtrl.desiredPos[POS_EXECUTOR][i] = -(RaiseTestBuf[i] / 360 * 8192) * KNEE_RATIO * M3508_RATIO;			
	}

	SetPtFlag(motorNum,BEGIN_MOTION);
	SetPtFlag(motorNum,NEW_DATA);
}



void ExecutorLoadingFirstBufferTest(uint8_t motorNum)
{
	for(int i = 0; i < 20;i++)
	{
		Driver[motorNum].ptCtrl.desiredPos[POS_FIRST_BUFFER][i] = -(RaiseUp[i] / 360 * 8192) * COAXE_RATIO * M3508_RATIO;	
	}
	Driver[motorNum].ptCtrl.MP[0] = 0x14003200;//20 size   50 周期
	Driver[motorNum].ptCtrl.index = 0;
	SetPtFlag(motorNum,EXECUTOR_LOADING_FIRST_BUFFER);
}
//放于while循环当中,测试数组衔接是否正常
void BufferExchangeTest(uint8_t motorNum)
{
	static int status_coaxe= 0, status_knee = 0;
	if(motorNum == COAXE_MOTOR_NUM)
	{
		switch(status_coaxe)
		{
			case 0:
				Driver[motorNum].ptCtrl.desiredTime = 20;
				Driver[motorNum].ptCtrl.runMode = RUN_AND_STOP_MOTION_MODE;
				Driver[motorNum].ptCtrl.size = 20;
				for(int i = 0; i < Driver[0].ptCtrl.size;i++)
				{
					Driver[motorNum].ptCtrl.desiredPos[POS_EXECUTOR][i] = -(RaiseUp[i] / 360.f * 8192.f) * COAXE_RATIO * M3508_RATIO;	
				}
				SetPtFlag(motorNum,BEGIN_MOTION);			
				status_coaxe =1 ;
				break;
			case 1:
				if(CheckPtFlag(motorNum,ACTION_READY_TO_COMPLETE))	
				{
					for(int i = 0; i < 20;i++)
					{
						Driver[motorNum].ptCtrl.desiredPos[POS_FIRST_BUFFER][i] = -(RaiseDown[i] / 360.f * 8192.f) * COAXE_RATIO * M3508_RATIO;	
					}
					Driver[motorNum].ptCtrl.MP[0] = 0x14001400;
					SetPtFlag(motorNum,EXECUTOR_LOADING_FIRST_BUFFER);
				}
				if(CheckPtFlag(motorNum,ACTION_COMPLETE ))
				{
					status_coaxe= 2;
					SetPtFlag(motorNum,~ACTION_COMPLETE);
				}
				break;
			case 2:
				if(CheckPtFlag(motorNum,ACTION_READY_TO_COMPLETE))	
				{
					for(int i = 0; i < 20;i++)
					{
						Driver[motorNum].ptCtrl.desiredPos[POS_FIRST_BUFFER][i] = -(RaiseUp[i] / 360.f * 8192.f) * COAXE_RATIO * M3508_RATIO;	
					}
					Driver[motorNum].ptCtrl.MP[0] = 0x14001400;
					SetPtFlag(motorNum,EXECUTOR_LOADING_FIRST_BUFFER);
				}			
				if(CheckPtFlag(motorNum,ACTION_COMPLETE))
				{
					status_coaxe= 1;
					SetPtFlag(motorNum,~ACTION_COMPLETE);
				}
				break;
		}
	}
	else if(motorNum == KNEE_MOTOR_NUM)
	{
		switch(status_knee)
		{
			case 0:
				Driver[motorNum].ptCtrl.desiredTime = 50;
				Driver[motorNum].ptCtrl.runMode = RUN_AND_STOP_MOTION_MODE;
				Driver[motorNum].ptCtrl.size = 20;
				for(int i = 0; i < Driver[0].ptCtrl.size;i++)
				{
					Driver[motorNum].ptCtrl.desiredPos[POS_EXECUTOR][i] = -(RaiseUp[i] / 360.f * 8192.f) * KNEE_RATIO * M3508_RATIO;	
				}
				SetPtFlag(motorNum,BEGIN_MOTION);			
				status_knee=1 ;
				break;
			case 1:
				if(CheckPtFlag(motorNum,ACTION_READY_TO_COMPLETE))	
				{
					for(int i = 0; i < 20;i++)
					{
						Driver[motorNum].ptCtrl.desiredPos[POS_FIRST_BUFFER][i] = -(RaiseDown[i] / 360.f * 8192.f) * KNEE_RATIO * M3508_RATIO;	
					}
					Driver[motorNum].ptCtrl.MP[0] = 0x14001400;
					SetPtFlag(motorNum,EXECUTOR_LOADING_FIRST_BUFFER);
				}
				if(CheckPtFlag(motorNum,ACTION_COMPLETE ))
				{
					status_knee= 2;
					SetPtFlag(motorNum,~ACTION_COMPLETE);
				}
				break;
			case 2:
				if(CheckPtFlag(motorNum,ACTION_READY_TO_COMPLETE))	
				{
					for(int i = 0; i < 20;i++)
					{
						Driver[motorNum].ptCtrl.desiredPos[POS_FIRST_BUFFER][i] = -(RaiseUp[i] / 360.f * 8192.f) * KNEE_RATIO * M3508_RATIO;	
					}
					Driver[motorNum].ptCtrl.MP[0] = 0x14001400;
					SetPtFlag(motorNum,EXECUTOR_LOADING_FIRST_BUFFER);
				}			
				if(CheckPtFlag(motorNum,ACTION_COMPLETE))
				{
					status_knee= 1;
					SetPtFlag(motorNum,~ACTION_COMPLETE);
				}
				break;
		}		
	}

//	USART_OUT(USART3,(uint8_t*)"%d\t%d\r\n",(int)status_coaxe,(int)status_knee);
}


void SlfTest(uint8_t motorNum)//学弟给的曲线
{
	static int status_coaxe= 0, status_knee = 0;
	static int start_flag_coaxe = 0,start_flag_knee = 0;
	if(start_flag_coaxe == 0 && motorNum == COAXE_MOTOR_NUM)
	{
		start_flag_coaxe = 1;
		for(int i = 0; i < 20;i++)
		{
			Driver[motorNum].ptCtrl.desiredPos[POS_FIRST_BUFFER][i] = -(slf2_1[i] / 360.f * 8192.f) * COAXE_RATIO * M3508_RATIO;	
		}
		Driver[motorNum].ptCtrl.MP[0] = 0x14000A00;
		SetPtFlag(motorNum,EXECUTOR_LOADING_FIRST_BUFFER);
	}
	if(start_flag_knee == 0 && motorNum == KNEE_MOTOR_NUM)
	{
		for(int i = 0; i < 0x14;i++)
		{
			Driver[motorNum].ptCtrl.desiredPos[POS_FIRST_BUFFER][i] = -(slf1_1[i] / 360.f * 8192.f) * COAXE_RATIO * M3508_RATIO;	
		}
		Driver[motorNum].ptCtrl.MP[0] = 0x14000A00;
		SetPtFlag(motorNum,EXECUTOR_LOADING_FIRST_BUFFER);
		start_flag_knee = 1;
	}	
	if(motorNum == COAXE_MOTOR_NUM)
	{
		switch(status_coaxe)
		{
			case 0:
				if(CheckPtFlag(motorNum,ACTION_READY_TO_COMPLETE))	
				{
					for(int i = 0; i < 0x14;i++)
					{
						Driver[motorNum].ptCtrl.desiredPos[POS_FIRST_BUFFER][i] = -(slf2_2[i] / 360.f * 8192.f) * COAXE_RATIO * M3508_RATIO;	
					}
					Driver[motorNum].ptCtrl.MP[0] = 0x14000A00;
					SetPtFlag(motorNum,EXECUTOR_LOADING_FIRST_BUFFER);
				}
				if(CheckPtFlag(motorNum,ACTION_COMPLETE ))
				{
					status_coaxe= 1;
					SetPtFlag(motorNum,~ACTION_COMPLETE);
				}
				break;
			case 1:
				if(CheckPtFlag(motorNum,ACTION_READY_TO_COMPLETE))	
				{
					for(int i = 0; i < 0x0B;i++)
					{
						Driver[motorNum].ptCtrl.desiredPos[POS_FIRST_BUFFER][i] = -(slf2_3[i] / 360.f * 8192.f) * COAXE_RATIO * M3508_RATIO;	
					}
					Driver[motorNum].ptCtrl.MP[0] = 0x0B000A00;
					SetPtFlag(motorNum,EXECUTOR_LOADING_FIRST_BUFFER);
				}
				if(CheckPtFlag(motorNum,ACTION_COMPLETE ))
				{
					status_coaxe= 2;
					SetPtFlag(motorNum,~ACTION_COMPLETE);
				}
				break;
			case 2:
				if(CheckPtFlag(motorNum,ACTION_READY_TO_COMPLETE))	
				{
					for(int i = 0; i < 0x14;i++)
					{
						Driver[motorNum].ptCtrl.desiredPos[POS_FIRST_BUFFER][i] = -(slf2_1[i] / 360.f * 8192.f) * COAXE_RATIO * M3508_RATIO;	
					}
					Driver[motorNum].ptCtrl.MP[0] = 0x14000A00;
					SetPtFlag(motorNum,EXECUTOR_LOADING_FIRST_BUFFER);
				}			
				if(CheckPtFlag(motorNum,ACTION_COMPLETE))
				{
					status_coaxe= 0;
					SetPtFlag(motorNum,~ACTION_COMPLETE);
				}
				break;
		}
	}
	else if(motorNum == KNEE_MOTOR_NUM)
	{
		switch(status_knee)
		{
			case 0:
				if(CheckPtFlag(motorNum,ACTION_READY_TO_COMPLETE))	
				{
					for(int i = 0; i < 0x14;i++)
					{
						Driver[motorNum].ptCtrl.desiredPos[POS_FIRST_BUFFER][i] = -(slf1_2[i] / 360.f * 8192.f) * KNEE_RATIO * M3508_RATIO;	
					}
					Driver[motorNum].ptCtrl.MP[0] = 0x14000A00;
					SetPtFlag(motorNum,EXECUTOR_LOADING_FIRST_BUFFER);
				}
				if(CheckPtFlag(motorNum,ACTION_COMPLETE ))
				{
					status_knee= 1;
					SetPtFlag(motorNum,~ACTION_COMPLETE);
				}
				break;
			case 1:
				if(CheckPtFlag(motorNum,ACTION_READY_TO_COMPLETE))	
				{
					for(int i = 0; i < 0x0B;i++)
					{
						Driver[motorNum].ptCtrl.desiredPos[POS_FIRST_BUFFER][i] = -(slf1_3[i] / 360.f * 8192.f) * KNEE_RATIO * M3508_RATIO;	
					}
					Driver[motorNum].ptCtrl.MP[0] = 0x0B000A00;
					SetPtFlag(motorNum,EXECUTOR_LOADING_FIRST_BUFFER);
				}
				if(CheckPtFlag(motorNum,ACTION_COMPLETE ))
				{
					status_knee= 2;
					SetPtFlag(motorNum,~ACTION_COMPLETE);
				}
				break;
			case 2:
				if(CheckPtFlag(motorNum,ACTION_READY_TO_COMPLETE))	
				{
					for(int i = 0; i < 0x14;i++)
					{
						Driver[motorNum].ptCtrl.desiredPos[POS_FIRST_BUFFER][i] = -(slf1_1[i] / 360.f * 8192.f) * KNEE_RATIO * M3508_RATIO;	
					}
					Driver[motorNum].ptCtrl.MP[0] = 0x14000A00;
					SetPtFlag(motorNum,EXECUTOR_LOADING_FIRST_BUFFER);
				}			
				if(CheckPtFlag(motorNum,ACTION_COMPLETE))
				{
					status_knee= 0;
					SetPtFlag(motorNum,~ACTION_COMPLETE);
				}
				break;
		}		
	}

//	USART_OUT(USART3,(uint8_t*)"%d\t%d\r\n",(int)status_coaxe,(int)CheckPtFlag(motorNum,ACTION_READY_TO_COMPLETE));
}



void ResetTest(uint8_t motorNum)
{
	Driver[motorNum].ptCtrl.velLimit = 100;
	Driver[motorNum].ptCtrl.desiredTime = 200;
	Driver[motorNum].ptCtrl.runMode = RUN_AND_STOP_MOTION_MODE;
	Driver[motorNum].ptCtrl.size = 5;
	for(int i = 0; i < Driver[0].ptCtrl.size;i++)
	{
//		Driver[motorNum].ptCtrl.desiredPos[POS_EXECUTOR][i] = -(ResetBuffer[i] / 360.f * 8192.f) * COAXE_RATIO * M3508_RATIO;	
	}
	SetPtFlag(motorNum,BEGIN_MOTION);
	SetPtFlag(motorNum,NEW_DATA);
}

void ResetInit(void)
{
	PtStructInit();
	Driver[COAXE_MOTOR_NUM].ptCtrl.desiredTime = 0;
	Driver[COAXE_MOTOR_NUM].ptCtrl.runMode = RUN_AND_STOP_MOTION_MODE;
	Driver[COAXE_MOTOR_NUM].ptCtrl.size = 0;
	Driver[COAXE_MOTOR_NUM].ptCtrl.executeFlag = 0;
	Driver[COAXE_MOTOR_NUM].ptCtrl.index = 0;
	Driver[COAXE_MOTOR_NUM].ptCtrl.velLimit = 100;
	
	Driver[KNEE_MOTOR_NUM].ptCtrl.desiredTime = 0;
	Driver[KNEE_MOTOR_NUM].ptCtrl.runMode = RUN_AND_STOP_MOTION_MODE;
	Driver[KNEE_MOTOR_NUM].ptCtrl.size = 0;
	Driver[KNEE_MOTOR_NUM].ptCtrl.executeFlag = 0;
	Driver[KNEE_MOTOR_NUM].ptCtrl.index = 0;
	Driver[KNEE_MOTOR_NUM].ptCtrl.velLimit = 100;
	
	Driver[COAXE_MOTOR_NUM].ptCtrl.velLimit = 100;
	Driver[COAXE_MOTOR_NUM].ptCtrl.desiredTime = 200;
	Driver[COAXE_MOTOR_NUM].ptCtrl.runMode = RUN_AND_STOP_MOTION_MODE;
	Driver[COAXE_MOTOR_NUM].ptCtrl.size = 5;
	for(int i = 0; i < Driver[0].ptCtrl.size;i++)
	{
		Driver[COAXE_MOTOR_NUM].ptCtrl.desiredPos[POS_EXECUTOR][i] = -(coaxe_resetBuffer[i] / 360.f * 8192.f) * COAXE_RATIO * M3508_RATIO;	
	}
	SetPtFlag(COAXE_MOTOR_NUM,BEGIN_MOTION);
	SetPtFlag(COAXE_MOTOR_NUM,NEW_DATA);	

	Driver[KNEE_MOTOR_NUM].ptCtrl.velLimit = 100;
	Driver[KNEE_MOTOR_NUM].ptCtrl.desiredTime = 200;
	Driver[KNEE_MOTOR_NUM].ptCtrl.runMode = RUN_AND_STOP_MOTION_MODE;
	Driver[KNEE_MOTOR_NUM].ptCtrl.size = 5;
	for(int i = 0; i < Driver[0].ptCtrl.size;i++)
	{
		Driver[KNEE_MOTOR_NUM].ptCtrl.desiredPos[POS_EXECUTOR][i] = -(knee_resetBuffer[i] / 360.f * 8192.f) * KNEE_RATIO * M3508_RATIO;	
	}
	SetPtFlag(KNEE_MOTOR_NUM,BEGIN_MOTION);
	SetPtFlag(KNEE_MOTOR_NUM,NEW_DATA);	
	
	while(!CheckPtFlag(COAXE_MOTOR_NUM,ACTION_COMPLETE) || !CheckPtFlag(KNEE_MOTOR_NUM,ACTION_COMPLETE)){};
		
	SetPtFlag(COAXE_MOTOR_NUM,~ACTION_COMPLETE);
	SetPtFlag(KNEE_MOTOR_NUM,~ACTION_COMPLETE);	
	SetPtFlag(COAXE_MOTOR_NUM,~BEGIN_MOTION);	
	SetPtFlag(KNEE_MOTOR_NUM,~BEGIN_MOTION);
	
	Driver[COAXE_MOTOR_NUM].ptCtrl.velLimit = VEL_MAX_3508;
	Driver[KNEE_MOTOR_NUM].ptCtrl.velLimit = VEL_MAX_3508;	
	Driver[KNEE_MOTOR_NUM].posCtrl.actualPos = 0;		
	Driver[COAXE_MOTOR_NUM].posCtrl.actualPos = 0;
	Driver[KNEE_MOTOR_NUM].posCtrl.actualPos = 0;
	for(int i = 0; i < Driver[0].ptCtrl.size;i++)
	{
		Driver[KNEE_MOTOR_NUM].ptCtrl.desiredPos[POS_EXECUTOR][i] = 0;
		Driver[COAXE_MOTOR_NUM].ptCtrl.desiredPos[POS_EXECUTOR][i] = 0;		
	}
	
}




