#ifndef __FOUR_LEG__H
#define __FOUR_LEG__H

#include "stm32f4xx.h"
#include "comm.h"


#define COAXE_RATIO 3.4f
#define	KNEE_RATIO  2.5f
#define M3508_RATIO  19.2f
#define COAXE_MAX_ANGLE_PULSE  1000
#define COAXE_MIN_ANGLE_PULSE  -96000
#define KNEE_MAX_ANGLE_PULSE		0
#define KNEE_MIN_ANGLE_PULSE		-131072//120度
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


