#ifndef __FOUR_LEG__H
#define __FOUR_LEG__H

#include "stm32f4xx.h"
#include "comm.h"

#define LEFT_FORWARD_LEG 1
#define RIGHT_FORWARD_LEG 2
#define LEFT_BACKWARD_LEG 3
#define RIGHT_BACKWARD_LEG 4

#define BOARD LEFT_BACKWARD_LEG

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
	#define COAXE_MAX_ANGLE_PULSE  -5000
	#define COAXE_MIN_ANGLE_PULSE  96000
	#define KNEE_MAX_ANGLE_PULSE		-5000
	#define KNEE_MIN_ANGLE_PULSE		131072//120度
#elif BOARD == RIGHT_BACKWARD_LEG
	#define COAXE_MAX_ANGLE_PULSE  -5000
	#define COAXE_MIN_ANGLE_PULSE  96000
	#define KNEE_MAX_ANGLE_PULSE		-5000
	#define KNEE_MIN_ANGLE_PULSE		131072//120度
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


