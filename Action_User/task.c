#include "includes.h"
#include <app_cfg.h>
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "elmo.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_usart.h"
#include "pps.h"
#include "fort.h"
#include "movebase.h"
/*
===============================================================
						信号量定义
===============================================================
*/
OS_EXT INT8U OSCPUUsage;
OS_EVENT *PeriodSem;

static OS_STK App_ConfigStk[Config_TASK_START_STK_SIZE];
static OS_STK WalkTaskStk[Walk_TASK_STK_SIZE];

void App_Task()
{
	CPU_INT08U os_err;
	os_err = os_err; /*防止警告...*/

	/*创建信号量*/
	PeriodSem = OSSemCreate(0);

	/*创建任务*/
	os_err = OSTaskCreate((void (*)(void *))ConfigTask, /*初始化任务*/
						  (void *)0,
						  (OS_STK *)&App_ConfigStk[Config_TASK_START_STK_SIZE - 1],
						  (INT8U)Config_TASK_START_PRIO);

	os_err = OSTaskCreate((void (*)(void *))WalkTask,
						  (void *)0,
						  (OS_STK *)&WalkTaskStk[Walk_TASK_STK_SIZE - 1],
						  (INT8U)Walk_TASK_PRIO);
}

/*
   ===============================================================
   初始化任务
   ===============================================================
   */
void ConfigTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	TIM_Init(TIM2, 99, 839, 1, 0);
	ElmoInit(CAN1);//驱动初始化
//	VelLoopCfg(CAN1,1,32768*20,32768*100);//速度环初始化
	PosLoopCfg(CAN1,1, 20000000,20000000,20000000);//位置环初始化
	MotorOn(CAN1,1);//使能
	UART4_Init(921600);	
	GPIO_Init_Pins(GPIOB,2,GPIO_Mode_IN); //初始化按键
	//GPIO_Init_Pins(GPIOB,11,GPIO_Mode_OUT); //初始化按键
	OSTaskSuspend(OS_PRIO_SELF);
}
void WalkTask(void)
{

	CPU_INT08U os_err;
	os_err = os_err;
	OSSemSet(PeriodSem, 0, &os_err);
	extern union readVEL_ readVel;
	int stop=0;
	int cnt=0;
	int riseup=0;
	
	union U1
{
	char s[4];
	float d;
};
union U1 u1;
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		extern readMotor_ readmotor;
		extern int changedata; 
		if(changedata%2==1)
		ReadActualVel(CAN1, 1);
		if(changedata%2==0)
		{
			ReadActualPos(CAN1, 1);
			if(changedata>100)
				changedata=0;
		}
		uint8_t key_B= GPIO_ReadInputDataBit (GPIOB,GPIO_Pin_2);	
		if(key_B==0)
		{
			riseup=1;
		}
		else riseup=0;
 		
		
			
		
		if(riseup==1)
		{
			uint8_t CANSendData[4];
			u1.d=1;
				CANSendData[0]=u1.s[0];
				CANSendData[1]=u1.s[1];
				CANSendData[2]=u1.s[2];
				CANSendData[3]=u1.s[3];		
    //发送数据		
	CAN_TxMsg(CAN1,0x0001,CANSendData,0x0004);
		}
		
 

	}
}


