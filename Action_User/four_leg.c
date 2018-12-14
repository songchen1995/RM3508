#include "four_leg.h"
#include "ctrl.h"
#include "string.h"
#include "timer.h"
#include "usart.h"
#include "comm.h"


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

//需要复位
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






//老师的轨迹点（low_acceleration）
float coaxe_low_acc_reset_buffer[5]=
{
	10.f,
	20.f,
	25.f,
	26.914f,
	26.914f
};

float knee_low_acc_reset_buffer[5]=
{
	10.f,
	25.f,
	40.f,
	57.312f,
	57.312f,
};
//注意，以下点位相对于机械限位而不是初始复位
float coaxe_start_1[20]=
{
26.914,	26.942,	27.025,	27.163,	27.354,
27.599,	27.896,	28.243,	28.639,	29.078,
29.562,	30.084,	30.644,	31.235,	31.855,
32.499,	33.159,	33.833,	34.513,	35.191,
};
float  coaxe_start_2[6]=
{
35.86,
36.51,
37.132,
37.714,
38.242,
38.694,
};
float coaxe_move_1[20] = 
{
38.694,	39.03,	39.27,	39.462,	39.627,
39.766,	39.85,	39.837,	39.608,	38.923,
37.732,	35.876,	33.345,	30.152,	26.641,
22.473,	18.299,	14.309,	10.589,	7.244,
};
float coaxe_move_2[6] = 
{
4.424,
2.408,
1.292,
1.328,
2.482,
4.545,
};
float coaxe_move_3[20]=
{
4.545,	6.756,	8.876,	10.914,	12.876,
14.768,	16.592,	18.352,	20.048,	21.682,
23.254,	24.765,	26.213,	27.599,	28.921,
30.179,	31.369,	32.492,	33.544,	34.524,
};
float coaxe_move_4[5]=
{
35.429,
36.255,
36.999,
37.657,
38.224,
};

float knee_start_1[20]=
{
57.312,	57.311,	57.308,	57.302,	57.292,
57.276,	57.25,	57.212,	57.157,	57.081,
56.979,	56.845,	56.673,	56.456,	56.186,
55.855,	55.454,	54.971,	54.398,	53.722,
};

float knee_start_2[6]=
{
52.928,
52.003,
50.93,
49.689,
48.258,
46.636,
};


float knee_move_1[20]=
{
46.636,	45.436,	44.862,	44.862,	45.262,
45.962,	47.112,	48.712,	50.962,	53.962,
56.962,	59.762,	62.012,	63.512,	64.132,
63.882,	62.692,	60.562,	57.83,	54.758,
	
};

float knee_move_2[6]=
{
51.626,
48.866,
46.715,
45.588,
45.52,
46.678,
	


};

float knee_move_3[20]=
{
46.678,	48.515,	50.134,	51.558,	52.801,
53.875,	54.792,	55.553,	56.18,	56.663,
57.012,	57.228,	57.316,	57.276,	57.11,
56.818,	56.401,	55.858,	55.188,	54.388,

};

float knee_move_4[5]=
{
53.456,
52.388,
51.179,
49.823,
48.312,

};


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
	static uint8_t status_knee = 1,status_coaxe = 1;
	static uint8_t N_knee = 0 , N_coaxe = 0;
//	if(!CheckPtFlag(motorNum,RECEIVE_START_AND_MP|RECEIVE_QN))
//	{
//		status = 1;
//	}
		if(motorNum == KNEE_MOTOR_NUM)
		{
			switch(status_knee)
			{
				case 1: 
					if(CheckPtFlag(motorNum,RECEIVE_START_AND_MP))
					{
						SetPtFlag(motorNum,~ACTION_COMPLETE);
						SetPtFlag(motorNum,SECOND_BUFFER_LOADING_CAN_BUFFER);
						Driver[motorNum].ptCtrl.MP[1] = RxData.data32[1];
						status_knee = 2;
						SetPtFlag(motorNum,~RECEIVE_START_AND_MP);
						SetPtFlag(motorNum,RECEIVE_QN);
					}
					break;
				case 2:
					if(CheckPtFlag(motorNum,CheckPtFlag(motorNum,RECEIVE_QN)))
					{
						Driver[motorNum].ptCtrl.desiredPos[POS_SECOND_BUFFER][N_knee] = RxData.data32[0] ;
						N_knee++;
						if(N_knee < Driver[motorNum].ptCtrl.MP[1] >> 24)
						{
							Driver[motorNum].ptCtrl.desiredPos[POS_SECOND_BUFFER][N_knee] = RxData.data32[1];
							N_knee++;
						}
						if(N_knee == Driver[motorNum].ptCtrl.MP[1] >> 24)
						{
							SetPtFlag(motorNum,~CAN_RECEIVING);
							SetPtFlag(motorNum,~RECEIVE_QN);
							SetPtFlag(motorNum,RECEIVE_BEGIN);
							SetPtFlag(motorNum,~SECOND_BUFFER_LOADING_CAN_BUFFER);
							SetPtFlag(motorNum,FIRST_BUFFER_LOADING_SECOND_BUFFER);
							Driver[motorNum].ptCtrl.MP[0] = Driver[KNEE_MOTOR_NUM].ptCtrl.MP[1];	
							Driver[motorNum].ptCtrl.MP[1] = 0;	
							N_knee = 0;
						}
					}
						if(CheckPtFlag(motorNum,RECEIVE_BEGIN))
							status_knee = 1;
		//			else//若是在接收数组的过程当中重新从START开始，则二级缓存可以被擦写
		//			{
		//				status = 0;
		//				N = 0;
		//				SetPtFlag(motorNum,~RECEIVE_QN);
		//				SetPtFlag(motorNum,RECEIVE_BEGIN);
		//				SetPtFlag(motorNum,~SECOND_BUFFER_LOADING_CAN_BUFFER);
		//				SetPtFlag(motorNum,FIRST_BUFFER_LOADING_SECOND_BUFFER);
		//				Driver[motorNum].ptCtrl.MP[0] = Driver[0].ptCtrl.MP[1];	
		//				Driver[motorNum].ptCtrl.MP[1] = 0;	
		//			}	
						
					break;
				default:
					break;
			}
		}
		else if(motorNum == COAXE_MOTOR_NUM)
		{
			switch(status_coaxe)
			{
				case 1: 
					if(CheckPtFlag(motorNum,RECEIVE_START_AND_MP))
					{
						SetPtFlag(motorNum,~ACTION_COMPLETE);
						SetPtFlag(motorNum,SECOND_BUFFER_LOADING_CAN_BUFFER);
						Driver[motorNum].ptCtrl.MP[1] = RxData.data32[1];
						status_coaxe = 2;
						SetPtFlag(motorNum,~RECEIVE_START_AND_MP);
						SetPtFlag(motorNum,RECEIVE_QN);
					}
					break;
				case 2:
					if(CheckPtFlag(motorNum,CheckPtFlag(motorNum,RECEIVE_QN)))
					{
						Driver[motorNum].ptCtrl.desiredPos[POS_SECOND_BUFFER][N_coaxe] = RxData.data32[0] ;
						N_coaxe++;
						if(N_coaxe < Driver[motorNum].ptCtrl.MP[1] >> 24)
						{
							Driver[motorNum].ptCtrl.desiredPos[POS_SECOND_BUFFER][N_coaxe] = RxData.data32[1];
							N_coaxe++;
						}
						if(N_coaxe == Driver[motorNum].ptCtrl.MP[1] >> 24)
						{
							SetPtFlag(motorNum,~CAN_RECEIVING);
							SetPtFlag(motorNum,~RECEIVE_QN);
							SetPtFlag(motorNum,RECEIVE_BEGIN);
							SetPtFlag(motorNum,~SECOND_BUFFER_LOADING_CAN_BUFFER);
							SetPtFlag(motorNum,FIRST_BUFFER_LOADING_SECOND_BUFFER);
							Driver[motorNum].ptCtrl.MP[0] = Driver[motorNum].ptCtrl.MP[1];	
							Driver[motorNum].ptCtrl.MP[1] = 0;	
							N_coaxe = 0;
						}
					}
						if(CheckPtFlag(motorNum,RECEIVE_BEGIN))
							status_coaxe = 1;
					break;
				default:
					break;
			}
		}
}


void PtSecondBufferHandler(uint8_t motorNum)
{
	if(CheckPtFlag(motorNum,FIRST_BUFFER_LOADING_SECOND_BUFFER))
	{
		for(int i = 0; i< Driver[motorNum].ptCtrl.MP[0] >> 24; i++)
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

void Low_Acceleration_Test(uint8_t motorNum)//老师给的曲线
{
	static int status_coaxe= 0, status_knee = 0;
	static int start_flag_coaxe = 0,start_flag_knee = 0;
	if(start_flag_coaxe == 0 && motorNum == COAXE_MOTOR_NUM)
	{
		start_flag_coaxe = 1;
		for(int i = 0; i < 0x14;i++)
		{
			Driver[motorNum].ptCtrl.desiredPos[POS_FIRST_BUFFER][i] = -((coaxe_start_1[i]- 26.914f) / 360.f * 8192.f) * COAXE_RATIO * M3508_RATIO;	
		}
		Driver[motorNum].ptCtrl.MP[0] = 0x14003000;
		SetPtFlag(motorNum,EXECUTOR_LOADING_FIRST_BUFFER);
	}
	if(start_flag_knee == 0 && motorNum == KNEE_MOTOR_NUM)
	{
		for(int i = 0; i < 0x14;i++)
		{
			Driver[motorNum].ptCtrl.desiredPos[POS_FIRST_BUFFER][i] = -((knee_start_1[i]- 57.312f) / 360.f * 8192.f) * COAXE_RATIO * M3508_RATIO;	
		}
		Driver[motorNum].ptCtrl.MP[0] = 0x14003000;
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
					for(int i = 0; i < 0x06;i++)
					{
						Driver[motorNum].ptCtrl.desiredPos[POS_FIRST_BUFFER][i] = -((coaxe_start_2[i] - 26.914f) / 360.f * 8192.f) * COAXE_RATIO * M3508_RATIO;	
					}
					Driver[motorNum].ptCtrl.MP[0] = 0x06003000;
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
					for(int i = 0; i < 0x14;i++)
					{
						Driver[motorNum].ptCtrl.desiredPos[POS_FIRST_BUFFER][i] = -((coaxe_move_1[i] - 26.914f) / 360.f * 8192.f) * COAXE_RATIO * M3508_RATIO;	
					}
					Driver[motorNum].ptCtrl.MP[0] = 0x14003000;
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
					for(int i = 0; i < 0x06;i++)
					{
						Driver[motorNum].ptCtrl.desiredPos[POS_FIRST_BUFFER][i] = -((coaxe_move_2[i] - 26.914f)/ 360.f * 8192.f) * COAXE_RATIO * M3508_RATIO;	
					}
					Driver[motorNum].ptCtrl.MP[0] = 0x06003000;
					SetPtFlag(motorNum,EXECUTOR_LOADING_FIRST_BUFFER);
				}			
				if(CheckPtFlag(motorNum,ACTION_COMPLETE))
				{
					status_coaxe= 3;
					SetPtFlag(motorNum,~ACTION_COMPLETE);
				}
				break;
			case 3:
				if(CheckPtFlag(motorNum,ACTION_READY_TO_COMPLETE))	
				{
					for(int i = 0; i < 0x14;i++)
					{
						Driver[motorNum].ptCtrl.desiredPos[POS_FIRST_BUFFER][i] = -((coaxe_move_3[i] - 26.914f) / 360.f * 8192.f) * COAXE_RATIO * M3508_RATIO;	
					}
					Driver[motorNum].ptCtrl.MP[0] = 0x14003000;
					SetPtFlag(motorNum,EXECUTOR_LOADING_FIRST_BUFFER);
				}			
				if(CheckPtFlag(motorNum,ACTION_COMPLETE))
				{
					status_coaxe= 4;
					SetPtFlag(motorNum,~ACTION_COMPLETE);
				}				
				break;
			case 4:
				if(CheckPtFlag(motorNum,ACTION_READY_TO_COMPLETE))	
				{
					for(int i = 0; i < 0x05;i++)
					{
						Driver[motorNum].ptCtrl.desiredPos[POS_FIRST_BUFFER][i] = -((coaxe_move_4[i] - 26.914f) / 360.f * 8192.f) * COAXE_RATIO * M3508_RATIO;	
					}
					Driver[motorNum].ptCtrl.MP[0] = 0x05003000;
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
				if(CheckPtFlag(motorNum,ACTION_READY_TO_COMPLETE))	
				{
					for(int i = 0; i < 0x06;i++)
					{
						Driver[motorNum].ptCtrl.desiredPos[POS_FIRST_BUFFER][i] = -((knee_start_2[i] - 57.312f)/ 360.f * 8192.f) * KNEE_RATIO * M3508_RATIO;	
					}
					Driver[motorNum].ptCtrl.MP[0] = 0x06003000;
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
					for(int i = 0; i < 0x14;i++)
					{
						Driver[motorNum].ptCtrl.desiredPos[POS_FIRST_BUFFER][i] = -((knee_move_1[i] - 57.312f) / 360.f * 8192.f) * KNEE_RATIO * M3508_RATIO;	
					}
					Driver[motorNum].ptCtrl.MP[0] = 0x14003000;
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
					for(int i = 0; i < 0x06;i++)
					{
						Driver[motorNum].ptCtrl.desiredPos[POS_FIRST_BUFFER][i] = -((knee_move_2[i] - 57.312f) / 360.f * 8192.f) * KNEE_RATIO * M3508_RATIO;	
					}
					Driver[motorNum].ptCtrl.MP[0] = 0x06003000;
					SetPtFlag(motorNum,EXECUTOR_LOADING_FIRST_BUFFER);
				}			
				if(CheckPtFlag(motorNum,ACTION_COMPLETE))
				{
					status_knee= 3;
					SetPtFlag(motorNum,~ACTION_COMPLETE);
				}
				break;
			case 3:
				if(CheckPtFlag(motorNum,ACTION_READY_TO_COMPLETE))	
				{
					for(int i = 0; i < 0x14;i++)
					{
						Driver[motorNum].ptCtrl.desiredPos[POS_FIRST_BUFFER][i] = -((knee_move_3[i] - 57.312f) / 360.f * 8192.f) * KNEE_RATIO * M3508_RATIO;	
					}
					Driver[motorNum].ptCtrl.MP[0] = 0x14003000;
					SetPtFlag(motorNum,EXECUTOR_LOADING_FIRST_BUFFER);
				}			
				if(CheckPtFlag(motorNum,ACTION_COMPLETE))
				{
					status_knee= 4;
					SetPtFlag(motorNum,~ACTION_COMPLETE);
				}
				break;
			case 4:
				if(CheckPtFlag(motorNum,ACTION_READY_TO_COMPLETE))	
				{
					for(int i = 0; i < 0x05;i++)
					{
						Driver[motorNum].ptCtrl.desiredPos[POS_FIRST_BUFFER][i] = -((knee_move_4[i] - 57.312f) / 360.f * 8192.f) * KNEE_RATIO * M3508_RATIO;	
					}
					Driver[motorNum].ptCtrl.MP[0] = 0x05003000;
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
//		Driver[COAXE_MOTOR_NUM].ptCtrl.desiredPos[POS_EXECUTOR][i] = -(coaxe_resetBuffer[i] / 360.f * 8192.f) * COAXE_RATIO * M3508_RATIO;	
		Driver[COAXE_MOTOR_NUM].ptCtrl.desiredPos[POS_EXECUTOR][i] = -(coaxe_low_acc_reset_buffer[i] / 360.f * 8192.f) * COAXE_RATIO * M3508_RATIO;	
	}
	SetPtFlag(COAXE_MOTOR_NUM,BEGIN_MOTION);
	SetPtFlag(COAXE_MOTOR_NUM,NEW_DATA);	

	Driver[KNEE_MOTOR_NUM].ptCtrl.velLimit = 100;
	Driver[KNEE_MOTOR_NUM].ptCtrl.desiredTime = 200;
	Driver[KNEE_MOTOR_NUM].ptCtrl.runMode = RUN_AND_STOP_MOTION_MODE;
	Driver[KNEE_MOTOR_NUM].ptCtrl.size = 5;
	for(int i = 0; i < Driver[0].ptCtrl.size;i++)
	{
		Driver[KNEE_MOTOR_NUM].ptCtrl.desiredPos[POS_EXECUTOR][i] = -(knee_low_acc_reset_buffer[i] / 360.f * 8192.f) * KNEE_RATIO * M3508_RATIO;	
	}
	SetPtFlag(KNEE_MOTOR_NUM,BEGIN_MOTION);
	SetPtFlag(KNEE_MOTOR_NUM,NEW_DATA);	
	
	while(!CheckPtFlag(COAXE_MOTOR_NUM,ACTION_COMPLETE) || !CheckPtFlag(KNEE_MOTOR_NUM,ACTION_COMPLETE)){};
		
	SetPtFlag(COAXE_MOTOR_NUM,~ACTION_COMPLETE);
	SetPtFlag(KNEE_MOTOR_NUM,~ACTION_COMPLETE);	
	SetPtFlag(COAXE_MOTOR_NUM,~BEGIN_MOTION);	
	SetPtFlag(KNEE_MOTOR_NUM,~BEGIN_MOTION);
	
	Driver[COAXE_MOTOR_NUM].ptCtrl.velLimit = VEL_MAX_3508 ;
	Driver[KNEE_MOTOR_NUM].ptCtrl.velLimit = VEL_MAX_3508 ;	
	Driver[KNEE_MOTOR_NUM].posCtrl.actualPos = 0;		
	Driver[COAXE_MOTOR_NUM].posCtrl.actualPos = 0;
	Driver[KNEE_MOTOR_NUM].posCtrl.actualPos = 0;
	for(int i = 0; i < Driver[0].ptCtrl.size;i++)
	{
		Driver[KNEE_MOTOR_NUM].ptCtrl.desiredPos[POS_EXECUTOR][i] = 0;
		Driver[COAXE_MOTOR_NUM].ptCtrl.desiredPos[POS_EXECUTOR][i] = 0;		
	}
	
}




