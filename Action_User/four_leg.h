#ifndef __FOUR_LEG__H
#define __FOUR_LEG__H

#include "stm32f4xx.h"
#include "comm.h"

#define LEFT_FORWARD_LEG 1
#define RIGHT_FORWARD_LEG 2
#define LEFT_BACKWARD_LEG 3
#define RIGHT_BACKWARD_LEG 4

#define BOARD RIGHT_FORWARD_LEG 


/*PT模式下的Flag*****************************88*******/
#define SECOND_BUFFER_LOADING_CAN_BUFFER 0x00000001
#define FIRST_BUFFER_LOADING_SECOND_BUFFER  0x00000002
#define EXECUTOR_LOADING_FIRST_BUFFER				0x00000004
#define RECEIVE_START_AND_MP								0x00000008
#define RECEIVE_QN													0x00000010
#define RECEIVE_BEGIN												0x00000020//手动清除标志位
#define BEGIN_MOTION												0x00000040
#define NEW_DATA														0x00000080//新数据进入，手动清除标志位（pt斜坡中接收到新指令后第一次执行需要）
#define ACTION_READY_TO_COMPLETE						0x00000100//即将完成一次动作
#define ACTION_COMPLETE											0x00000200//完成一次动作,手动清除标志位（接收到一次新指令清除标志位）
#define INDEX_JUMP													0x00000400//发生一次index跳变，手动清除标志位（PTVelSlope中清除）
#define CAN_RECEIVING												0x00000800
/*Author: Oliver********************************/

/*PT模式下的runMode************************************/
#define	SINGLE_MODE						0x00
#define CIRCULAR_MODE												0x01
#define	RUN_AND_STOP_MOTION_MODE						0x02

/*PT模式下的位置信息的data buf************************************/
#define POS_SECOND_BUFFER	0x02
#define POS_FIRST_BUFFER 0x01
#define POS_EXECUTOR 0x00
/*Author: Oliver********************************/

#define COAXE_RATIO 3.4f
#define	KNEE_RATIO  2.5f
#define SHOULDER_RATIO 3.0f
#define M3508_RATIO  19.2f
#if BOARD == LEFT_FORWARD_LEG
	#define COAXE_MAX_ANGLE_PULSE  5000
	#define COAXE_MIN_ANGLE_PULSE  -96000
	#define KNEE_MAX_ANGLE_PULSE		5000
	#define KNEE_MIN_ANGLE_PULSE		-131072//120度
#elif BOARD == LEFT_BACKWARD_LEG
#define COAXE_MAX_ANGLE_PULSE  5000
	#define COAXE_MIN_ANGLE_PULSE  -96000
	#define KNEE_MAX_ANGLE_PULSE		5000
	#define KNEE_MIN_ANGLE_PULSE		-131072//120度
#elif BOARD == RIGHT_FORWARD_LEG
	#define COAXE_MAX_ANGLE_PULSE  96000
	#define COAXE_MIN_ANGLE_PULSE  -5000
	#define KNEE_MAX_ANGLE_PULSE		131072
	#define KNEE_MIN_ANGLE_PULSE		-5000//120度
#elif BOARD == RIGHT_BACKWARD_LEG
	#define COAXE_MAX_ANGLE_PULSE  96000
	#define COAXE_MIN_ANGLE_PULSE  -5000
	#define KNEE_MAX_ANGLE_PULSE		131072
	#define KNEE_MIN_ANGLE_PULSE		-5000//120度
#endif

#define COAXE_MOTOR_NUM 0
#define KNEE_MOTOR_NUM	1
#define SHOULDER_MOTOR_NUM 2

void PtCanHandler(uint8_t motorNum,UnionDataType txData);
void PtSecondBufferHandler(uint8_t motorNum);
void PtFirstBufferHandler(uint8_t motorNum);

void RaiseTest(uint8_t motorNum);
void PtStructInit(void);
void ExecutorLoadingFirstBufferTest(uint8_t motorNum);
void BufferExchangeTest(uint8_t motorNum);
void ResetTest(uint8_t motorNum);
void ResetInit(void);
void SlfTest(uint8_t motorNum);

void Low_Acceleration_Test(uint8_t motorNum);
#endif


