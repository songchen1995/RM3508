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
  * @brief  ��������ʼ��
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
		 // Driver[i].unitMode = SPEED_CONTROL_MODE;
			Driver[i].unitMode = PVT_MODE;
			Driver[i].velCtrl.kp = VEL_KP_3508;
			Driver[i].velCtrl.ki = VEL_KI_3508 * 1;
			Driver[i].velCtrl.maxOutput = CURRENT_MAX_3508;
			Driver[i].velCtrl.desiredVel[MAX_V] = VEL_MAX_3508;
			Driver[i].posCtrl.kd = POS_KD_3508;
			Driver[i].posCtrl.kp = POS_KP_3508;
			Driver[i].homingMode.current = 0.8f;
			
			Driver[i].velCtrl.acc = 100.0f;
			Driver[i].velCtrl.dec = 100.0f;
			Driver[i].velCtrl.desiredVel[CMD] = 250.0f;
			Driver[i].posCtrl.desiredPos = 0.0f;
			Driver[i].posCtrl.acc = Driver[i].velCtrl.dec;
			Driver[i].posCtrl.posVel = 250.0f;
			Driver[i].homingMode.vel = -160.0f;

		}
		else if(Motor[i].type == M_2006)  //M2006�Ĳ���
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
	
  //���ó�ʼ״̬
	Driver[0].homingMode.current = 2.8f;
	Driver[1].homingMode.current = 2.8f;

//	Driver[1].homingMode.vel = -60.0f;
//	Driver[1].unitMode = SPEED_CONTROL_MODE;
#if BOARD == AUTO_3508
	//�Զ���������ת��λ
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
				//�°汾λ�û����㣬ʹ��б��
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
//	USART_OUT(USART3,(uint8_t*)"%d\t%d\r\n",(int)Driver[0].velCtrl.speed,(int)PerCur[0]);	
//	DMA_Send_Data((int)(Driver[0].velCtrl.speed) ,(int)(Driver[0].output*100.0f));
//	DMA_Send_Data((int)(Driver[2].velCtrl.speed) ,(int)(Driver[2].posCtrl.actualPos/10.0f));
//	DMA_Send_Data((int)(Driver[2].velCtrl.speed) ,(int)(Driver[2].posCtrl.output));
//	DMA_Send_Data((int)(Driver[0].velCtrl.speed) ,(int)(Driver[0].output*10.0f));
	
}
/**
  * @brief  �ٶ�б������
  * @param  None
  * @retval �ٶ��������
  */
float VelSlope(VelCtrlType *velPid)
{
	/*************����Ӽ��ٶ�б��**************/
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
  * @brief  �ٶȿ���
  * @param  None
  * @retval �ٶ�PID�����
  */
float VelPidCtrl(VelCtrlType *velPid)
{
	/*****************�ٶȻ�PID*****************/
	velPid->velErr = velPid->desiredVel[SOFT] - velPid->speed;	
	//�������
	velPid->iOut += velPid->ki * velPid->velErr;
	//�����޷�
	velPid->iOut = MaxMinLimit(velPid->iOut,velPid->maxOutput);
	//�������
	velPid->output = velPid->kp * velPid->velErr + velPid->iOut;
	//����޷�
	velPid->output = MaxMinLimit(velPid->output,velPid->maxOutput);
	
	return velPid->output;
}

/**
  * @brief  �ٶȻ���ʼ��
  * @param  None
  * @retval None 
  */
void VelCtrlInit(void)
{

}
/**
  * @brief  ���������ֵ
  * @param  val������ֵ
  * @retval ���ֵ
  */
float OutPutLim(float value)
{
	float outputMax,outputMin,outputBasic;
	/********************���㶯̬�����С���****************************/
	outputBasic = Driver[0].velCtrl.speed * EMF_CONSTANT;								//���㷴�綯��
	outputMax = outputBasic + VOL_AMP;									//�������
	outputMin = outputBasic - VOL_AMP;	 								//��Ҫ�����ٶ����ѹ��ϵ�ı�
	if(outputMax <  VOL_AMP) outputMax =  VOL_AMP;			//
	if(outputMin > -VOL_AMP) outputMin = -VOL_AMP;
		
	if(value < outputMin)	value = outputMin;						//
	if(value > outputMax)	value = outputMax;

	if(value > VOL_MAX) value = VOL_MAX;
	if(value <-VOL_MAX) value =-VOL_MAX;

//	CurrentOutput = (value - (float)velpms*0.04315f)*25.0f;
	if(value < 0)	  value -= VOL_BLIND_AREA;												//��������ä��0.3043f Vq������tim4
	else       	 		value += VOL_BLIND_AREA;

	return value;
}

/**
  * @brief  λ�ÿ���(��λ�û�����)
  * @param  None
  * @retval λ�û�PID�������
  */
float PosCtrl(PosCtrlType *posPid)
{
	float posPidOut = 0.0f;
	float desiredVel = 0.0f,signVel = 1.0f;
	
	/******************************����λ�û����**************************************/
	posPid->posErr = posPid->desiredPos - posPid->actualPos;				
	posPidOut = posPid->posErr*posPid->kp + posPid->kd*(posPid->posErr-posPid->posErrLast);		
	posPid->posErrLast = posPid->posErr;
	
	if(posPid->posErr < 0.0f) signVel = -1.0f;	
	
	//����0.7����Ϊ������Ҫ��ԣ�����д��Ż���б�����⣩
	desiredVel = signVel*__sqrtf(2.0f*0.7f*posPid->acc*signVel*posPid->posErr);
		
	if(fabsf(desiredVel) < fabsf(posPidOut))
		posPidOut = desiredVel;
	//��һ����С������
//	if(fabsf(posPid->posErr) <= 200.0f)		posPidOut = 0.0f;
	
	posPid->output = MaxMinLimit(posPidOut,posPid->posVel);
	
	return posPid->output;
}






/**
  * @brief PVTPos
  * @param  None
  * @retval 
  */
//float PVTPosCtrl(PVTCtrlType *pvtPid, PosCtrlType *posPid)
//{
//	float posPidOut = 0.0f;
//	float desiredVel = 0.0f,signVel = 1.0f;
//	
//	/******************************݆̣λ׃۷ˤԶ**************************************/
//	pvtPid->posErr = pvtPid->desiredPos - posPid->actualPos;				
//	posPidOut = pvtPid->posErr * pvtPid->pos_kp + pvtPid->pos_kd * (pvtPid->posErr-pvtPid->posErrLast);		
//	pvtPid->posErrLast = posPid->posErr;
//	
//	if(pvtPid->posErr < 0.0f) signVel = -1.0f;	
//	
//	
//	desiredVel = signVel*__sqrtf(2.0f*0.7f*posPid->acc*signVel*pvtPid->posErr);
//		
//	if(fabsf(desiredVel) < fabsf(posPidOut))
//		posPidOut = desiredVel;

////	if(fabsf(posPid->posErr) <= 200.0f)		posPidOut = 0.0f;
//	
//	posPid->output = MaxMinLimit(posPidOut,posPid->posVel);
//	
//	return posPid->output;
//}


/**
  * @brief PVT
  * @param  None
  * @retval 
  */
//速度和位置的闭环(时间分段，PID平滑)
//CIRCULAR MODE
//delp,delv异号没有意义

float PVTCtrl(PVTCtrlType *pvtPid, PosCtrlType *posPid, VelCtrlType *velPid)
{
	static float posPidOut = 0.0f, velPidOut  = 0, pvtPidOut = 0;
	static float desiredVel = 0.0f,signVel = 1.0f;
	static int index = 1,cnt = 0;
	static float kp = 0.1,ki = 0.00001,posErr = 0,posErrLast = 0;
	if(index < 22)
	{
		if(cnt < pvtPid->desiredTime[index])
		{
			velPidOut = (pvtPid->desiredPos[index] - pvtPid->desiredPos[index-1]) / (pvtPid->desiredTime[index]);
			cnt++;
		}
		else
		{
			index++;
			cnt = 0;
		}
	}

	posErr = pvtPid->desiredPos[index] - posPid->actualPos;
	posPidOut = posErr * kp + (posErr - posErrLast) * ki;  
	posErrLast = posErr;
	
	if(index == 22)
	{
		index = 22;
//		velPidOut = 0;
		cnt = 0;
	}
	
	USART_OUT(USART3,(uint8_t*)"%d\t%d\t%d\t%d\r\n",(int)velPid->desiredVel[CMD],(int)velPid->speed,(int)pvtPid->desiredPos[index-1],(int)posPid->actualPos);	
	
//	DMA_Send_Data(signVel,(int)pvtPid->desiredPos);
	pvtPid->output = MaxMinLimit(velPidOut + posPidOut,pvtPid -> velLimit);

	return pvtPid->output;
}
/**
  * @brief  Homing mode
  * @param  None
  * @retval �����ֵ
  */

void HomingMode(DriverType *driver)
{
	float output;

	driver->velCtrl.desiredVel[SOFT] = driver->homingMode.vel;
	output = VelPidCtrl(&driver->velCtrl);
	
	driver->homingMode.output = MaxMinLimit(output,driver->homingMode.current);//����homeģʽʱ����ֵ
	
	if(fabsf(driver->velCtrl.speed) <=2){		//2
		driver->homingMode.cnt++;
	}else{
		driver->homingMode.cnt = 0;
	}
	
	if(driver->homingMode.cnt >= 500){									//500ms

		driver->posCtrl.actualPos=0.0f;				//
		driver->posCtrl.desiredPos = driver->posCtrl.actualPos + 8192.0f;
		//������
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
  * @retval �����ֵ
  */
void HomingModeInit(void)
{
}

/**
  * @brief  ���������ѹ
  * @param  None
  * @retval λ�û������ֵ
  */
float GetPosPidOut(void)
{
	return Driver[0].posCtrl.output;
}

/**
  * @brief  λ�û���ʼ��
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
	
	//�÷����ٶ�����
	driver->velCtrl.speed = (float)(motor->vel)*0.1365333f;					//1/60*8192/1000=0.136533
	//��λ�ò�ֳ����ٶ�����
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
  * @brief  ���������ѹ
  * @param  None
  * @retval �õ���ֵ
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
  * @brief  ���ʹ��
  * @param  n:�ĸ����  (0-7)
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
  * @brief  ���ʧ��
  * @param  n:�ĸ����  (0-7)
	* @retval None
  */
void MotorOff(int n)
{
  Driver[n].status = DISABLE;
}

/**
  * @brief  �ٶȻ�����
	* @param  vel���������ٶȴ�С
	* @param  tim���ٶ��л�ʱ��
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
