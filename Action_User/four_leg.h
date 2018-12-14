#ifndef __FOUR_LEG__H
#define __FOUR_LEG__H

#include "stm32f4xx.h"
#include "comm.h"


#define COAXE_RATIO 3.4f
#define	KNEE_RATIO  2.5f
#define M3508_RATIO  19.2f
#define COAXE_MAX_ANGLE_PULSE  0
#define COAXE_MIN_ANGLE_PULSE  -89000
#define KNEE_MAX_ANGLE_PULSE		0
#define KNEE_MIN_ANGLE_PULSE		-65536
#define COAXE_MOTOR_NUM 0
#define KNEE_MOTOR_NUM	1

void PtCanHandler(uint8_t motorNum,UnionDataType txData);
void PtSecondBufferHandler(uint8_t motorNum);
void PtFirstBufferHandler(uint8_t motorNum);

void RaiseTest(uint8_t motorNum);
void PtStructInit(void);
void ExecutorLoadingFirstBufferTest(uint8_t motorNum);
void BufferExchangeTest(uint8_t motorNum);
void ResetTest(uint8_t motorNum);
void ResetInit(void);
#endif


