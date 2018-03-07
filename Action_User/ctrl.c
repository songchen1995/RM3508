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
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
DriverType Driver = {0};
/* Extern   variables ---------------------------------------------------------*/
extern MotorType Motor[8];
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/

/**
  * @brief  驱动器初始化
  * @param  None
  * @retval 
  */
void DriverInit(void)
{
  Driver.Status = DISABLE;
	Driver.VoltageOutput = 0.0f;
	Driver.Commutation.Mode = FOC_MODE;
	Driver.Command.CAN_status = 0;
	Driver.Encoder.Period = 8192;
	VelCtrlInit();
	PosCtrlInit();
	HomingModeInit();
  //配置初始状态
//  Driver.UnitMode = POSITION_CONTROL_MODE;
  Driver.UnitMode = HOMING_MODE;
	Driver.VelCtrl.Acc = 15.0f;
	Driver.VelCtrl.Dec = 15.0f;
	Driver.VelCtrl.DesiredVel = 1250.0f;
	Driver.PosCtrl.DesiredPos = 0.0f;
	
	Driver.HomingMode.Vel = -160.0f;
}

/**
  * @brief  MotorCtrl
	* @param  None
	* @retval None
  */
float PerCur[4] = {0.0f};

void MotorCtrl(void)
{
	CalculSpeed();
	
	switch(Driver.UnitMode)
	{
		case POSITION_CONTROL_MODE:
//			Driver.VoltageOutput = PosCtrl();
			Driver.VoltageOutput = VelCtrl(PosCtrl());
			break;
		case SPEED_CONTROL_MODE:
			Driver.VoltageOutput = VelCtrl(VelSlope(Driver.VelCtrl.DesiredVel));
			break;
		case HOMING_MODE:
			HomingMode();
			Driver.VoltageOutput = Driver.HomingMode.Output;
			break;
		default:break;
	}

  if(Driver.Status != ENABLE)
    Driver.VoltageOutput = 0.0f;
  
	PerCur[0] = Driver.VoltageOutput;
//	PerCur[0] = 0.0f;
	SetCur(PerCur);
	
//	DMA_Send_Data((int)(Motor[0].Vel) ,(int)(Driver.VoltageOutput*100.0f));
	DMA_Send_Data((int)(Driver.VelCtrl.Speed) ,(int)(Driver.PosCtrl.ActualPos));
//	DMA_Send_Data((int)(Driver.VelCtrl.Speed) ,(int)(Driver.VoltageOutput*100.0f));
	
}
/**
  * @brief  速度斜坡输入
  * @param  None
  * @retval 速度期望输出
  */
float VelSlope(float cmdVel)
{
	static float desiredVel = 0.0f;
	/*************计算加减速度斜坡**************/
	if(desiredVel < (cmdVel - Driver.VelCtrl.Acc)){
		desiredVel +=Driver.VelCtrl.Acc;
	}else if(desiredVel > (cmdVel + Driver.VelCtrl.Dec)){
		desiredVel -=Driver.VelCtrl.Dec;
	}else{
		desiredVel = cmdVel;
	}	
	return desiredVel;
}
/**
  * @brief  速度控制
  * @param  None
  * @retval 速度PID的输出
  */
float VelCtrl(float cmdVel)
{
	float velErr,velpidout;

	/*****************速度环PID*****************/
	velErr = cmdVel - Driver.VelCtrl.Speed;	
	Driver.VelCtrl.TemI += Driver.VelCtrl.Ki*velErr;
	Driver.VelCtrl.TemI = MaxMinLimit(Driver.VelCtrl.TemI,CURRENT_MAX);
	velpidout = Driver.VelCtrl.Kp * velErr + Driver.VelCtrl.TemI;
	
	Driver.VelCtrl.Output = MaxMinLimit(velpidout,CURRENT_MAX);
	
//	if(fabsf(Driver.VelCtrl.TemI) > fabsf(Driver.VelCtrl.Output))
//		Driver.VelCtrl.TemI = Driver.VelCtrl.Output;
	
	return Driver.VelCtrl.Output;
}

/**
  * @brief  速度环初始化
  * @param  None
  * @retval None 
  */
void VelCtrlInit(void)
{
	Driver.VelCtrl.Kp = 0.03f;
	Driver.VelCtrl.Ki = 0.0030f;//0.0004f;

	Driver.VelCtrl.TemI = 0.0f;
	Driver.VelCtrl.DesiredVel = 0.0f;
	Driver.VelCtrl.Acc = 0.0f;
	Driver.VelCtrl.Dec = 0.0f;
	Driver.VelCtrl.Output = 0.0f;
}
/**
  * @brief  限制输出幅值
  * @param  val：输入值
  * @retval 输出值
  */
float OutPutLim(float value)
{
	float outputMax,outputMin,outputBasic;
	/********************计算动态最大最小输出****************************/
	outputBasic = Driver.VelCtrl.Speed * EMF_CONSTANT;								//估算反电动势
	outputMax = outputBasic + VOL_AMP;									//输出幅度
	outputMin = outputBasic - VOL_AMP;	 								//需要根据速度与电压关系改变
	if(outputMax <  VOL_AMP) outputMax =  VOL_AMP;			//
	if(outputMin > -VOL_AMP) outputMin = -VOL_AMP;
		
	if(value < outputMin)	value = outputMin;						//
	if(value > outputMax)	value = outputMax;

	if(value > VOL_MAX) value = VOL_MAX;
	if(value <-VOL_MAX) value =-VOL_MAX;

//	CurrentOutput = (value - (float)velpms*0.04315f)*25.0f;
	if(value < 0)	  value -= VOL_BLIND_AREA;												//消除控制盲区0.3043f Vq发布给tim4
	else       	 		value += VOL_BLIND_AREA;

	return value;
}

/**
  * @brief  位置控制
  * @param  None
  * @retval 位置环PID的输出。
  */
/***************位置环变量********************/
int8_t		Sign = 0,SignDis = 0,SignVel = 0;
int32_t		Tim = 0,TimEnd = 0,TimAcc = 0,TimEve = 0;
int32_t 	Shape = 0,VelNow = 0;
float 		DelPos = 0.0f,DisAcc = 0.0f,DisDec = 0.0f,DisVelDec = 0.0f,TemAimPos = 0.0f,SetposOld = 0.0f,PoStart = 0.0f;

float PosCtrl(void)
{
	float posErr=0.0f,pospidout = 0.0f;		
	static float posErrLast=0.0f;

	/****************位置环路径规划****************/
	if(Driver.PosCtrl.DesiredPos != SetposOld){
		
		//防止给定参数为0
		if(Driver.VelCtrl.DesiredVel <= 0.001f)
			Driver.VelCtrl.DesiredVel = 1.0f;
		if(Driver.VelCtrl.Acc <= 0.00001f)
			Driver.VelCtrl.Acc = 0.003f;
			
		DelPos = Driver.PosCtrl.DesiredPos - Driver.PosCtrl.ActualPos;
		VelNow = Driver.VelCtrl.Speed;
		VelNow = MaxMinLimit(VelNow,Driver.VelCtrl.DesiredVel);

		
		if(DelPos<0) SignDis = -1;
		else				 SignDis =  1;
		
		if(VelNow<0) SignVel = -1;
		else				 SignVel =  1;
		
		DisDec = Driver.VelCtrl.DesiredVel*Driver.VelCtrl.DesiredVel/Driver.VelCtrl.Acc/2;
		DisVelDec = VelNow*VelNow/Driver.VelCtrl.Acc/2;
		DisAcc = DisDec - DisVelDec;
		
		if((SignDis*SignVel > 0)&&(SignDis*DelPos < DisVelDec)) 
			Sign = -1;
		else 													 
			Sign =  1;
		
		if((SignDis*VelNow > Driver.VelCtrl.DesiredVel)&&(Sign == 1)){
			Shape = 5;
			DisAcc = -DisAcc;
			TimAcc = (SignVel*VelNow - Driver.VelCtrl.DesiredVel)/Driver.VelCtrl.Acc;
			TimEve = (Sign*SignDis*DelPos - DisVelDec)/Driver.VelCtrl.DesiredVel;
			TimEnd = TimAcc + TimEve + Driver.VelCtrl.DesiredVel/Driver.VelCtrl.Acc;
			Tim = 0;
			PoStart = Driver.PosCtrl.ActualPos;
		}else if(Sign*SignDis*DelPos <= (DisAcc+DisDec)){
			Shape = 3;
			TimEnd = sqrt(4*(Sign*SignDis*DelPos+DisVelDec)/Driver.VelCtrl.Acc);
			Tim = Sign*SignDis*VelNow/Driver.VelCtrl.Acc;
			PoStart = Driver.PosCtrl.ActualPos - Sign*SignDis*DisVelDec;
		}else{
			Shape = 4;
			TimAcc = Driver.VelCtrl.DesiredVel/Driver.VelCtrl.Acc;
			TimEve = (Sign*SignDis*DelPos + DisVelDec - 2*DisDec)/Driver.VelCtrl.DesiredVel;
			TimEnd = TimEve + 2*TimAcc;
			Tim = Sign*SignDis*VelNow/Driver.VelCtrl.Acc;
			PoStart = Driver.PosCtrl.ActualPos - Sign*SignDis*DisVelDec;
		}
		Tim += 3;		
		SetposOld = Driver.PosCtrl.DesiredPos;
	}
	/*************************当前周期目标位置分情况计算*******************************/
	if(Shape == 3){		
		if(Tim<TimEnd/2)
			TemAimPos = PoStart + Sign*SignDis*0.5f*Driver.VelCtrl.Acc*Tim*Tim;
		else
			TemAimPos = Driver.PosCtrl.DesiredPos - Sign*SignDis*0.5f*Driver.VelCtrl.Acc*(TimEnd-Tim)*(TimEnd-Tim);
	}else if(Shape == 4){
		if(Tim<TimAcc)
			TemAimPos = PoStart + Sign*SignDis*0.5f*Driver.VelCtrl.Acc*Tim*Tim;
		else if(Tim < (TimAcc+TimEve)) 
			TemAimPos = PoStart + Sign*SignDis*(Driver.VelCtrl.DesiredVel*Tim - DisDec);		
		else
			TemAimPos = Driver.PosCtrl.DesiredPos - Sign*SignDis*0.5f*Driver.VelCtrl.Acc*(TimEnd-Tim)*(TimEnd-Tim);
	}else if(Shape ==5){
		if(Tim<TimAcc)
			TemAimPos = PoStart + Sign*SignDis*SignVel*VelNow*Tim - Sign*SignDis*0.5f*Driver.VelCtrl.Acc*Tim*Tim;
		else if(Tim < (TimAcc+TimEve)) 
			TemAimPos = PoStart + Sign*SignDis*DisAcc + Sign*SignDis*Driver.VelCtrl.DesiredVel*(Tim - TimAcc);
		else
			TemAimPos = Driver.PosCtrl.DesiredPos - Sign*SignDis*0.5f*Driver.VelCtrl.Acc*(TimEnd-Tim)*(TimEnd-Tim);
	}else{
		TemAimPos = Driver.PosCtrl.DesiredPos;
	}
	/******************************计算位置环输出**************************************/
	posErr=TemAimPos-Driver.PosCtrl.ActualPos;				
//	pospidout=posErr*0.0090f+0.009f*(posErr-posErrLast)-0.00000f*Driver.VelCtrl.Speed;					//	
	pospidout = posErr*Driver.PosCtrl.Kp + Driver.PosCtrl.Kd*(posErr-posErrLast) - 0.00000f*Driver.VelCtrl.Speed;		
	//	
	posErrLast = posErr;
	
	if(fabsf(posErr) <=200.0f)
		pospidout = 0.0f;
	
	
	/*******************动态限幅，控制输出********************/	
//	if((pospidout > VOL_MAX)||(pospidout < -VOL_MAX))		Tim-=1;			//运行时打开
	if(Tim>=TimEnd)	Tim = TimEnd;		
	
//	Driver.PosCtrl.Output = OutPutLim(pospidout);
	Driver.PosCtrl.Output = MaxMinLimit(pospidout,Driver.VelCtrl.DesiredVel);
	
//	Driver.PosCtrl.Output = pospidout;
	
	Tim+=1;	
	
	return Driver.PosCtrl.Output;
}

/**
  * @brief  Homing mode
  * @param  None
  * @retval 输出的值
  */
void HomingMode(void)
{
	static float posLast = 0.0f;
	float output;

	output = VelCtrl(Driver.HomingMode.Vel);
	
	Driver.HomingMode.Output = MaxMinLimit(output,0.8f);//限制home模式时电流值
	
	if(fabsf(Driver.PosCtrl.ActualPos - posLast) <=2){		//2
		Driver.HomingMode.Cnt++;
	}else{
		Driver.HomingMode.Cnt = 0;
	}
	posLast = Driver.PosCtrl.ActualPos;
	if(Driver.HomingMode.Cnt >= 500){									//500ms
//		Driver.HomingMode.InitPos = UpdateAbsPos();
//		Driver.HomingMode.InitPos = GetIncPos();
		Driver.PosCtrl.ActualPos=0.0f;				//
		Driver.PosCtrl.DesiredPos = Driver.PosCtrl.ActualPos;
		//清除输出
		Driver.HomingMode.Output = 0.0f;
		Driver.VelCtrl.Output = 0.0f;
		Driver.VoltageOutput = 0.0f;
		Driver.VelCtrl.TemI = 0.0f;
		Sign = 0;
		Driver.UnitMode = POSITION_CONTROL_MODE;
	}
}

/**
  * @brief  Homing mode Init
  * @param  None
  * @retval 输出的值
  */
void HomingModeInit(void)
{
	Driver.HomingMode.Vel = -3.0f;
	Driver.HomingMode.Cnt = 0;
}

/**
  * @brief  传递输出电压
  * @param  None
  * @retval 位置环输出的值
  */
float GetPosPidOut(void)
{
	return Driver.PosCtrl.Output;
}




/**
  * @brief  位置环初始化
  * @param  None
  * @retval None
  */
void PosCtrlInit(void)
{
//	Driver.PosCtrl.Kp = 0.031f;
//	Driver.PosCtrl.Kp = 0.011f;//静态时
//	Driver.PosCtrl.Kd = 0.061f;
	Driver.PosCtrl.Kp = 0.11f;//
	Driver.PosCtrl.Kd = 4.51f; 
	
	Driver.VelCtrl.DesiredVel = 10.0f;
	Driver.VelCtrl.Acc = 0.003f;
	Driver.VelCtrl.Dec = 0.003f;						
	
	CalculSpeed();

	Driver.PosCtrl.ActualPos = 0.0f;
	Driver.PosCtrl.DesiredPos = Driver.PosCtrl.ActualPos;
	SetposOld = Driver.PosCtrl.ActualPos;
}

/**
  * @brief  Calculate Speed
  * @param  None
  * @retval Subtraction number between every two times.
**/
float CalculSpeed(void)
{
	static int PosOld = 0;
	int PosNow = 0;
	int speed = 0.0f;
	PosNow = GetMotorPos(0);										//
	speed = (PosNow - PosOld);
	PosOld = PosNow;
	if(speed > (Driver.Encoder.Period/2)) speed -= Driver.Encoder.Period;
	if(speed <-(Driver.Encoder.Period/2)) speed += Driver.Encoder.Period;
	
	Driver.PosCtrl.ActualPos += speed;

//  T法测速	
//	if(Driver.Encoder.TimeMode.TimNum != 0){ 
//		if(speed >= 0.0f)
//			Driver.Encoder.TimeMode.Vel = 4000.0f/(float)(Driver.Encoder.TimeMode.TimNum);
//		else
//			Driver.Encoder.TimeMode.Vel = -1.0f*4000.0f/(float)(Driver.Encoder.TimeMode.TimNum);
//			
//	}
	
	//用反馈速度输入
	Driver.VelCtrl.Speed = (float)(Motor[0].Vel)*0.1365333f;					//1/60*8192/1000=0.136533
	//用位置差分出的速度输入
//	Driver.VelCtrl.Speed = speed;
	
//	Driver.VelCtrl.Speed = Driver.Encoder.TimeMode.Vel;
	return Driver.VelCtrl.Speed; 
}
/**
  * @brief  Get Speed
  * @param  None
  * @retval Speed
**/
float GetSpeed(void)
{
	return Driver.VelCtrl.Speed;
}

/**
  * @brief  传递输出电压
  * @param  None
  * @retval 得到的值
  */
float GetVelPidOut(void)
{
	return Driver.VelCtrl.Output;
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
  * @brief  电机使能
	* @param  None
	* @retval None
  */
void MotorOn(void)
{
  if(Driver.UnitMode == POSITION_CONTROL_MODE)
    Driver.PosCtrl.DesiredPos = Driver.PosCtrl.ActualPos;
  
  if(Driver.UnitMode == SPEED_CONTROL_MODE)
    Driver.VelCtrl.DesiredVel = Driver.VelCtrl.Speed;
  
  Driver.VelCtrl.TemI = 0.0f;
  
  Driver.Status = ENABLE;
}

/**
  * @brief  电机失能
	* @param  None
	* @retval None
  */
void MotorOff(void)
{
  Driver.Status = DISABLE;
}



/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
