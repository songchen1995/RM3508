#ifndef __FOUR_LEG__H
#define __FOUR_LEG__H

#include "stm32f4xx.h"
#include "comm.h"


#define COAXE_RATIO 3.4f
#define	KNEE_RATIO  2.5f
#define M3508_RATIO  19.2f
#define COAXE_MAX_ANGLE_PULSE  0
#define COAXE_MIN_ANGLE_PULSE  -89000


void PtCanHandler(int id,UnionDataType txData);
void PtSecondBufferHandler(int id, UnionDataType txData);
void PtFirstBufferHandler(void);

void RaiseTest(uint8_t motorNum);
void PtStructInit(void);
void ExecutorLoadingFirstBufferTest(uint8_t motorNum);
void BufferExchangeTest(uint8_t motorNum);
void ResetTest(uint8_t motorNum);
void ResetInit(uint8_t motorNum);
#endif


