#ifndef __FOUR_LEG__H
#define __FOUR_LEG__H

#define COAXE_RATIO 3.4f
#define	KNEE_RATIO  2.5f
#define M3508_RATIO  19.2f
#define COAXE_MAX_ANGLE_PULSE  0
#define COAXE_MIN_ANGLE_PULSE  -89000


void RaiseTest(void);
void PtStructInit(void);
void ExecutorLoadingFirstBufferTest(void);
void BufferExchangeTest(void);
void ResetTest(void);
void ResetInit(void);
#endif


