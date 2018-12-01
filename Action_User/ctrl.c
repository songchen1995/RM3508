/**
  ******************************************************************************
  * @file     
  * @author  Tmax Sco
  * @version V1.0.0
  * @date    2017-2-22
  * @brief   
  ******************************************************************************
  * @attention
  *
  *
  *
  * 
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/
#include "ctrl.h"
#include "math.h"
#include "rm_motor.h"
#include "usart.h"
#include "elmo.h"
#include "comm.h"
#include "timer.h"
#include "TLE5012.h"

/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
DriverType Driver[8] = {0};
/* Extern   variables ---------------------------------------------------------*/
extern MotorType Motor[8];
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/

#define AUTO_3508 1
#define AUTO_2006 2
#define MANUAL    3

#define BOARD  AUTO_3508

/**
  * @brief  Çý¶¯Æ÷³õÊ¼»¯
  * @param  None
  * @retval 
  */
void DriverInit(void)
{
	Motor[0].type = RM_3508;
	Motor[1].type = NONE;
#if BOARD == AUTO_3508
	Motor[2].type = M_2006;
#elif BOARD == AUTO_2006
	Motor[2].type = M_2006;
#else
	Motor[2].type = NONE;
#endif
	Motor[3].type = M_2006;
	Motor[4].type = NONE;
	Motor[5].type = RM_3508;
	Motor[6].type = M_2006;
	Motor[7].type = M_2006;
	
#if BOARD == AUTO_3508
	Driver[0].command.canId = 5;
	Driver[1].command.canId = 6;
	Driver[2].command.canId = 7;
	Driver[3].command.canId = 8;
#elif BOARD == AUTO_2006
	Driver[0].command.canId = 15;
	Driver[1].command.canId = 16;
	Driver[2].command.canId = 7;
	Driver[3].command.canId = 8;
#else
	Driver[0].command.canId = 5;
	Driver[1].command.canId = 7;
	Driver[2].command.canId = 17;
	Driver[3].command.canId = 18;
#endif
	
	for(int i = 0; i < 8; i++)
	{
		Driver[i].status = DISABLE;
		Driver[i].encoder.period = 8192;		
		
	  if(Motor[i].type == RM_3508)
		{
			//Driver[i].unitMode = HOMING_MODE;
		 // Driver[i].unitMode = POSITION_CONTROL_MODE;
		//  Driver[i].unitMode = SPEED_CONTROL_MODE;
			Driver[i].unitMode = PVT_MODE;
			Driver[i].velCtrl.kp = VEL_KP_3508;
			Driver[i].velCtrl.ki = VEL_KI_3508;
			Driver[i].velCtrl.maxOutput = CURRENT_MAX_3508;
			Driver[i].velCtrl.desiredVel[MAX_V] = VEL_MAX_3508;
			Driver[i].posCtrl.kd = POS_KD_3508;
			Driver[i].posCtrl.kp = POS_KP_3508;
			Driver[i].homingMode.current = 0.8f;
			
			Driver[i].velCtrl.acc = 1000.0f;
			Driver[i].velCtrl.dec = 1000.0f;
			Driver[i].velCtrl.desiredVel[CMD] = 250.0f;
			Driver[i].posCtrl.desiredPos = 0.0f;
			Driver[i].posCtrl.acc = Driver[i].velCtrl.dec;
			Driver[i].posCtrl.posVel = 250.0f;
			Driver[i].homingMode.vel = -160.0f;

		}
		else if(Motor[i].type == M_2006)  //M2006µÄ²ÎÊý
		{
			Driver[i].unitMode = HOMING_MODE;
		//  Driver[i].unitMode = POSITION_CONTROL_MODE;
		//  Driver[i].unitMode = SPEED_CONTROL_MODE;
			
			Driver[i].velCtrl.kp = VEL_KP_2006;
			Driver[i].velCtrl.ki = VEL_KI_2006;
			Driver[i].velCtrl.maxOutput = CURRENT_MAX_2006;		
			Driver[i].velCtrl.desiredVel[MAX_V] = VEL_MAX_2006;
			Driver[i].posCtrl.kd = POS_KD_2006;
			Driver[i].posCtrl.kp = POS_KP_2006;
			Driver[i].homingMode.current = 2.8f;
			
			Driver[i].velCtrl.acc = 100.0f;
			Driver[i].velCtrl.dec = 100.0f;
			Driver[i].velCtrl.desiredVel[CMD] = 250.0f;
			Driver[i].posCtrl.desiredPos = 0.0f;
			Driver[i].posCtrl.acc = 0.7f*Driver[i].velCtrl.dec;
			Driver[i].posCtrl.posVel = 250.0f;
			Driver[i].homingMode.vel = -160.0f;
		}
		else
		{
			break;
		}
	}
	
	//
	
  //ÅäÖÃ³õÊ¼×´Ì¬
	Driver[0].homingMode.current = 2.8f;
	Driver[1].homingMode.current = 2.8f;

//	Driver[1].homingMode.vel = -60.0f;
//	Driver[1].unitMode = SPEED_CONTROL_MODE;
#if BOARD == AUTO_3508
	//×Ô¶¯³µ¸©ÑöÕý×ª¹éÎ»
	Driver[0].homingMode.vel = 160.f;
	Driver[1].homingMode.vel = 160.f;
	Driver[0].pvtCtrl.velLimit = VEL_MAX_3508;
	Driver[0].pvtCtrl.pos_kp = POS_KP_3508;
	Driver[0].pvtCtrl.pos_kd = POS_KD_3508;
//	Driver[0].unitMode = HOMING_MODE;
#elif BOARD == AUTO_2006
	Driver[0].unitMode = HOMING_MODE;
#else
	Driver[0].unitMode = POSITION_CONTROL_MODE;
#endif
	

}

/**
  * @brief  ZeroPosInit
	* @param  using the outer encoder to get the zero point of the motor
  * @note   There will be 0.4 degree of errors while zeropos initing
	* @retval None
  */
void ZeroPosInit(void)
{
	Driver[0].target5012B = 6000;
	for(int i = 0; i < 8; i++)
	{
		if(Motor[i].type == RM_3508)
		{
			Driver[i].encoder5012B = TLE5012B_GetPos14bit();
			Driver[i].posCtrl.actualPos =(( Driver[i].encoder5012B) - Driver[i].target5012B); 
			Driver[i].posCtrl.desiredPos = Driver[i].posCtrl.actualPos;
		}
		else
		{
			break;
		}
	}
}

void ZeroPosCtrl(DriverType* driver)
{
	TLE5012B_UpateData();
	Driver[0].encoder5012B = TLE5012B_GetPos15bit()/4;
	PosCtrl(&driver->posCtrl);
}

/**
  * @brief  MotorCtrl
	* @param  None
	* @retval None
  */
float PerCur[4] = {0.0f};

void MotorCtrl(void)
{
//	CalculSpeed();
	TLE5012B_UpateData();
	Driver[0].encoder5012B = TLE5012B_GetPos14bit();
	for(int i = 0; i < 8; i++)
	{
		if(Motor[i].type == NONE)
			break;
		
		CalculSpeed_Pos(&Driver[i],&Motor[i]);
		
		if(Driver[i].status != ENABLE)   
		{
			Driver[i].output = 0.0f;		
			continue;
		}
			
		switch(Driver[i].unitMode)
		{
			case POSITION_CONTROL_MODE:
				//ÐÂ°æ±¾Î»ÖÃ»·¼ÆËã£¬Ê¹ÓÃÐ±ÆÂ
				PosCtrl(&Driver[i].posCtrl);
				Driver[i].velCtrl.desiredVel[CMD] = Driver[i].posCtrl.output;
				VelSlope(&Driver[i].velCtrl);
				Driver[i].output = VelPidCtrl(&Driver[i].velCtrl);
				break;
			case SPEED_CONTROL_MODE:
	//			Driver[i].output = VelCtrl(VelSlope(Driver[i].velCtrl.desiredVel[CMD]));
				VelSlope(&Driver[i].velCtrl);
				Driver[i].output = VelPidCtrl(&Driver[i].velCtrl);
				break;
			case HOMING_MODE:
				HomingMode(&Driver[i]);
				Driver[i].output = Driver[i].homingMode.output;
				break;
			case PVT_MODE:
			  PVTCtrl(&Driver[i].pvtCtrl,&Driver[i].posCtrl,&Driver[i].velCtrl);
				Driver[i].velCtrl.desiredVel[CMD] = Driver[i].pvtCtrl.output;
				VelSlope(&Driver[i].velCtrl);
				Driver[i].output = VelPidCtrl(&Driver[i].velCtrl);		
						
//				DMA_Send_Data('\r','\n');
				break;
			default:break;
		}

	}


  
//	PerCur[0] = Driver[0].output;
//	PerCur[1] = Driver[1].output;
//	PerCur[2] = Driver[2].output;
//	PerCur[3] = Driver[3].output;
//	PerCur[0] = 0.0f;
	
	for(int i = 0; i < 4; i++)
	{
		if(Motor[i].type == RM_3508)
			PerCur[i] = Driver[i].output*16384.0f/20.0f;
		else if(Motor[i].type == M_2006)
			PerCur[i] = Driver[i].output*10000.0f/10.0f;  //M2006
		else 
			PerCur[i] = 0.0f;
	}
	SetCur(PerCur);
	
//	DMA_Send_Data((int)(Driver[0].velCtrl.speed) ,(int)(Driver[0].output*100.0f));
//	DMA_Send_Data((int)(Driver[2].velCtrl.speed) ,(int)(Driver[2].posCtrl.actualPos/10.0f));
//	DMA_Send_Data((int)(Driver[2].velCtrl.speed) ,(int)(Driver[2].posCtrl.output));
//	DMA_Send_Data((int)(Driver[0].velCtrl.speed) ,(int)(Driver[0].output*10.0f));
	
}
/**
  * @brief  ËÙ¶ÈÐ±ÆÂÊäÈë
  * @param  None
  * @retval ËÙ¶ÈÆÚÍûÊä³ö
  */
float VelSlope(VelCtrlType *velPid)
{
	/*************¼ÆËã¼Ó¼õËÙ¶ÈÐ±ÆÂ**************/
	if(velPid->desiredVel[SOFT] < (velPid->desiredVel[CMD] - velPid->acc)){
		velPid->desiredVel[SOFT] +=velPid->acc;
	}else if(velPid->desiredVel[SOFT] > (velPid->desiredVel[CMD] + velPid->dec)){
		velPid->desiredVel[SOFT] -=velPid->dec;
	}else{
		velPid->desiredVel[SOFT] = velPid->desiredVel[CMD];
	}	
	return velPid->desiredVel[SOFT];
}

/**
  * @brief  ËÙ¶È¿ØÖÆ
  * @param  None
  * @retval ËÙ¶ÈPIDµÄÊä³ö
  */
float VelPidCtrl(VelCtrlType *velPid)
{
	/*****************ËÙ¶È»·PID*****************/
	velPid->velErr = velPid->desiredVel[SOFT] - velPid->speed;	
	//¼ÆËã»ý·Ö
	velPid->iOut += velPid->ki * velPid->velErr;
	//»ý·ÖÏÞ·ù
	velPid->iOut = MaxMinLimit(velPid->iOut,velPid->maxOutput);
	//¼ÆËãÊä³ö
	velPid->output = velPid->kp * velPid->velErr + velPid->iOut;
	//Êä³öÏÞ·ù
	velPid->output = MaxMinLimit(velPid->output,velPid->maxOutput);
	
	return velPid->output;
}

/**
  * @brief  ËÙ¶È»·³õÊ¼»¯
  * @param  None
  * @retval None 
  */
void VelCtrlInit(void)
{

}
/**
  * @brief  ÏÞÖÆÊä³ö·ùÖµ
  * @param  val£ºÊäÈëÖµ
  * @retval Êä³öÖµ
  */
float OutPutLim(float value)
{
	float outputMax,outputMin,outputBasic;
	/********************¼ÆËã¶¯Ì¬×î´ó×îÐ¡Êä³ö****************************/
	outputBasic = Driver[0].velCtrl.speed * EMF_CONSTANT;								//¹ÀËã·´µç¶¯ÊÆ
	outputMax = outputBasic + VOL_AMP;									//Êä³ö·ù¶È
	outputMin = outputBasic - VOL_AMP;	 								//ÐèÒª¸ù¾ÝËÙ¶ÈÓëµçÑ¹¹ØÏµ¸Ä±ä
	if(outputMax <  VOL_AMP) outputMax =  VOL_AMP;			//
	if(outputMin > -VOL_AMP) outputMin = -VOL_AMP;
		
	if(value < outputMin)	value = outputMin;						//
	if(value > outputMax)	value = outputMax;

	if(value > VOL_MAX) value = VOL_MAX;
	if(value <-VOL_MAX) value =-VOL_MAX;

//	CurrentOutput = (value - (float)velpms*0.04315f)*25.0f;
	if(value < 0)	  value -= VOL_BLIND_AREA;												//Ïû³ý¿ØÖÆÃ¤Çø0.3043f Vq·¢²¼¸øtim4
	else       	 		value += VOL_BLIND_AREA;

	return value;
}

/**
  * @brief  Î»ÖÃ¿ØÖÆ(ÐÂÎ»ÖÃ»·³ÌÐò)
  * @param  None
  * @retval Î»ÖÃ»·PIDµÄÊä³ö¡£
  */
float PosCtrl(PosCtrlType *posPid)
{
	float posPidOut = 0.0f;
	float desiredVel = 0.0f,signVel = 1.0f;
	
	/******************************¼ÆËãÎ»ÖÃ»·Êä³ö**************************************/
	posPid->posErr = posPid->desiredPos - posPid->actualPos;				
	posPidOut = posPid->posErr*posPid->kp + posPid->kd*(posPid->posErr-posPid->posErrLast);		
	posPid->posErrLast = posPid->posErr;
	
	if(posPid->posErr < 0.0f) signVel = -1.0f;	
	
	//³ËÒÔ0.7ÊÇÒòÎª¼õËÙÐèÒªÓÐÔ£Á¿£¬ÓÐ´ýÓÅ»¯£¨Ð±ÆÂÎÊÌâ£©
	desiredVel = signVel*__sqrtf(2.0f*0.7f*posPid->acc*signVel*posPid->posErr);
		
	if(fabsf(desiredVel) < fabsf(posPidOut))
		posPidOut = desiredVel;
	//¸øÒ»¶¨´óÐ¡µÄËÀÇø
//	if(fabsf(posPid->posErr) <= 200.0f)		posPidOut = 0.0f;
	
	posPid->output = MaxMinLimit(posPidOut,posPid->posVel);
	
	return posPid->output;
}






/**
  * @brief PVTPos
  * @param  None
  * @retval 
  */
float PVTPosCtrl(PVTCtrlType *pvtPid, PosCtrlType *posPid)
{
	float posPidOut = 0.0f;
	float desiredVel = 0.0f,signVel = 1.0f;
	
	/******************************Ý†Ì£Î»×ƒÛ·Ë¤Ô¶**************************************/
	pvtPid->posErr = pvtPid->desiredPos - posPid->actualPos;				
	posPidOut = pvtPid->posErr * pvtPid->pos_kp + pvtPid->pos_kd * (pvtPid->posErr-pvtPid->posErrLast);		
	pvtPid->posErrLast = posPid->posErr;
	
	if(pvtPid->posErr < 0.0f) signVel = -1.0f;	
	
	
	desiredVel = signVel*__sqrtf(2.0f*0.7f*posPid->acc*signVel*pvtPid->posErr);
		
	if(fabsf(desiredVel) < fabsf(posPidOut))
		posPidOut = desiredVel;

//	if(fabsf(posPid->posErr) <= 200.0f)		posPidOut = 0.0f;
	
	posPid->output = MaxMinLimit(posPidOut,posPid->posVel);
	
	return posPid->output;
}


/**
  * @brief PVT
  * @param  None
  * @retval 
  */
//é€Ÿåº¦å’Œä½ç½®çš„é—­çŽ¯
//delp,delvå¼‚å·æ²¡æœ‰æ„ä¹‰
float PVTCtrl(PVTCtrlType *pvtPid, PosCtrlType *posPid, VelCtrlType *velPid)
{
	float posPidOut = 0.0f, velPidOut  = 0, pvtPidOut = 0;
	static float desiredVel = 0.0f,signVel = 1.0f;
	

	if(pvtPid->desiredPos != pvtPid->desiredPosLast )
	{
		if(pvtPid->desiredPos - pvtPid->desiredPosLast  < 0.01f)
		{
			signVel = -1.f;
			pvtPid->desiredPosLast =  pvtPid->desiredPos;
		}
		else if(pvtPid->desiredPos - pvtPid->desiredPosLast> 0.01f)
		{
			signVel = 1.f;
			pvtPid->desiredPosLast =  pvtPid->desiredPos;
		}
	}
	//ä½¿ç”¨ä¸²è¡ŒPIDè¿›è¡Œ(p,v)æŽ§åˆ¶
	//åˆ°è¾¾å¯¹åº”(p,v)ï¼Œç‚¹åŽï¼Œä¸å†å¯¹pé—­çŽ¯ï¼Œè€Œåªå¯¹vé—­çŽ¯
	//åœ¨å®Œæˆä¸€ä¸ªç›®æ ‡ç‚¹ä¹‹å‰ä¸ä¼šè¿›è¡Œä¸‹ä¸€æ¬¡è§„åˆ’
	posPidOut = PVTPosCtrl(pvtPid,posPid);
	velPidOut = pvtPid->desiredVel;
	if(signVel == 1.f)
	{
		if(pvtPid->desiredPos - posPid->actualPos <= 0.f)
		{
			posPidOut = 0;
			//signVel = 0.f;
		}
	}	
	else if(signVel == -1.f)
	{
		if(pvtPid->desiredPos - posPid->actualPos >= 0.f)
		{
			posPidOut = 0;
			//signVel = 0.f;
		}
	}
	
//	DMA_Send_Data(signVel,(int)pvtPid->desiredPos);
	pvtPid->output = MaxMinLimit(posPidOut + velPidOut,pvtPid -> velLimit);

	return pvtPid->output;
}
/**
  * @brief  Homing mode
  * @param  None
  * @retval Êä³öµÄÖµ
  */

void HomingMode(DriverType *driver)
{
	float output;

	driver->velCtrl.desiredVel[SOFT] = driver->homingMode.vel;
	output = VelPidCtrl(&driver->velCtrl);
	
	driver->homingMode.output = MaxMinLimit(output,driver->homingMode.current);//ÏÞÖÆhomeÄ£Ê½Ê±µçÁ÷Öµ
	
	if(fabsf(driver->velCtrl.speed) <=2){		//2
		driver->homingMode.cnt++;
	}else{
		driver->homingMode.cnt = 0;
	}
	
	if(driver->homingMode.cnt >= 500){									//500ms

		driver->posCtrl.actualPos=0.0f;				//
		driver->posCtrl.desiredPos = driver->posCtrl.actualPos + 8192.0f;
		//Çå³ýÊä³ö
		driver->homingMode.output = 0.0f;
		driver->velCtrl.desiredVel[CMD] = 0.0f;
		driver->velCtrl.desiredVel[SOFT] = 0.0f;
		driver->velCtrl.output = 0.0f;
		driver->output = 0.0f;
		driver->homingMode.output = 0.0f;
		driver->velCtrl.iOut = 0.0f;
		driver->unitMode = POSITION_CONTROL_MODE;
	}
}
/**
  * @brief  Homing mode Init
  * @param  None
  * @retval Êä³öµÄÖµ
  */
void HomingModeInit(void)
{
}

/**
  * @brief  ´«µÝÊä³öµçÑ¹
  * @param  None
  * @retval Î»ÖÃ»·Êä³öµÄÖµ
  */
float GetPosPidOut(void)
{
	return Driver[0].posCtrl.output;
}

/**
  * @brief  Î»ÖÃ»·³õÊ¼»¯
  * @param  None
  * @retval None
  */
void PosCtrlInit(void)
{					

}

/**
  * @brief  Calculate Speed
  * @param  None
  * @retval Subtraction number between every two times.
**/
float CalculSpeed_Pos(DriverType *driver,MotorType *motor)
{
	int deltaPos = 0;
	deltaPos = (motor->pos - motor->posLast);
	motor->posLast = motor->pos;
	if(deltaPos > (driver->encoder.period/2)) deltaPos -= driver->encoder.period;
	if(deltaPos <-(driver->encoder.period/2)) deltaPos += driver->encoder.period;
	
	driver->posCtrl.actualPos += deltaPos;
	
	//ÓÃ·´À¡ËÙ¶ÈÊäÈë
	driver->velCtrl.speed = (float)(motor->vel)*0.1365333f;					//1/60*8192/1000=0.136533
	//ÓÃÎ»ÖÃ²î·Ö³öµÄËÙ¶ÈÊäÈë
//	driver->velCtrl.speed = speed;
	
	return driver->velCtrl.speed; 
}
/**
  * @brief  Get Speed
  * @param  None
  * @retval Speed
**/
float GetSpeed(void)
{
	return Driver[0].velCtrl.speed;
}

/**
  * @brief  ´«µÝÊä³öµçÑ¹
  * @param  None
  * @retval µÃµ½µÄÖµ
  */
float GetVelPidOut(void)
{
	return Driver[0].velCtrl.output;
}

/**
  * @brief  max min limit
	* @param  inDat:
	* @retval outDat
  */
float MaxMinLimit(float val,float limit)
{
	if(val > limit) val =  limit;
	if(val <-limit) val = -limit;
	
	return val;
}

/**
  * @brief  µç»úÊ¹ÄÜ
  * @param  n:ÄÄ¸öµç»ú  (0-7)
	* @retval None
  */
void MotorOn(int n)
{
 if(Driver[n].unitMode == POSITION_CONTROL_MODE)
    Driver[n].posCtrl.desiredPos = Driver[n].posCtrl.actualPos;
  if(Driver[n].unitMode == SPEED_CONTROL_MODE)
    Driver[n].velCtrl.desiredVel[CMD] = 0.0f;
  
  Driver[n].velCtrl.iOut = 0.0f;

  Driver[n].status = ENABLE;
}

/**
  * @brief  µç»úÊ§ÄÜ
  * @param  n:ÄÄ¸öµç»ú  (0-7)
	* @retval None
  */
void MotorOff(int n)
{
  Driver[n].status = DISABLE;
}

/**
  * @brief  ËÙ¶È»·²âÊÔ
	* @param  vel£º²âÊÔÓÃËÙ¶È´óÐ¡
	* @param  tim£ºËÙ¶ÈÇÐ»»Ê±¼ä
	* @retval None
  */
void VelCtrlTest(float vel,int tim)
{
	Driver[0].velCtrl.desiredVel[CMD] = vel;
	TIM_Delayms(TIM3,tim);
	Driver[0].velCtrl.desiredVel[CMD] = -vel;
	TIM_Delayms(TIM3,tim);

}


/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
