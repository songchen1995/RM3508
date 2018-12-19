/**
  ******************************************************************************
  * @file    
  * @author  Tmax Sco
  * @version 
  * @date   
  * @brief   This file contains the headers of 
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ctrl_h
#define __ctrl_h

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stm32f4xx.h"
#include "rm_motor.h"
/* Exported types ------------------------------------------------------------*/

/** 
  * @brief  Encoder PulseMode type structure definition  
  * @note     
  */
typedef struct
{	
	int32_t Cnt;
	
	float Vel;
	
}PulseModeType;
/** 
  * @brief  Encoder TimeMode type structure definition  
  * @note     
  */
typedef struct
{
	int32_t Tim;	//���������100us�ĸ���
	
	float TimUite;
	
	int32_t TimNum;	//�����ʱ�� ��λ��us
	
	float Vel;
	
}TimeModeType;


/** 
  * @brief  Encoder type structure definition  
  * @note     
  */
typedef struct
{	
	TimeModeType  TimeMode;
	
	PulseModeType  PulseMode;
	
	int32_t period;
	
	int32_t Direct;
	
}EncoderType;

/** 
  * @brief  Commutation type structure definition  
  * @note     
  */
typedef struct
{	
	int32_t Mode;
	
}CommutationType;

/** 
  * @brief  Commutation type structure definition  
  * @note     
  */
typedef struct
{	
	int32_t can_status;
	
	int32_t canId;
	
}CommandType;

/** 
  * @brief  VelCtrl type structure definition  
  * @note     
  */
typedef struct
{
	float speed;
	
	float desiredVel[3];
	
	float velErr;
	
	float acc;
	
	float dec;
	
	float kp;
	
	float ki;
	
	float iOut;
	
	float output;
	
	float maxOutput;
	
}VelCtrlType;


/** 
  * @brief  PosCtrl type structure definition  
  * @note     
  */
typedef struct
{
	int actualPos;
	
	float desiredPos;
	
	float posErr,posErrLast;
	
	float posVel,acc;
	
	float basicPos;
	
	float kp;
	
	float kd;
	
	float output;
	
}PosCtrlType;


/** 
  * @brief  PosCtrl type structure definition  
  * @note     
  */
typedef struct
{	
	int desiredPos[3][20];
	
	uint8_t desiredTime;
	
	uint8_t cnt;
	
	uint8_t runMode;
	
	uint8_t size;
	
	uint32_t MP[2];//[]
	
	uint32_t executeFlag; 

	uint8_t motorNum;
	
	float output;
	
	float velOutput;
	
	float posOutput;
	
	float velLimit;
	
	float pulseMaxLimit;
	
	float pulseMinLimit;
	
	float posMec;
	
	float kp;
	
	float kd;
	
	float ki;
	
	uint8_t index;
	
	
}PTCtrlType;


/** 
  * @brief  HomingMode type structure definition  
  * @note     
  */
typedef struct
{
	float vel;
	
	float current;
	
	float initPos;
	
	int32_t cnt;
	
	float output;
	
}HomingModeType;

typedef struct
{
	float vel;
	
	float current;
	
	float actualPos;
	
	float desiredPos;
	
	int32_t cnt;
	
	float output;
	
}ZeroPosInitType;

/** 
  * @brief  Driver type structure definition  
  * @note     
  */
typedef struct
{
	uint32_t unitMode;
  
  int32_t time;
	
  FunctionalState status;
  
	float output;
	
	int encoder5012B;
	
	int target5012B;
	
	VelCtrlType velCtrl;
	
	PosCtrlType posCtrl;
	
	PTCtrlType ptCtrl;
	
	ZeroPosInitType zeroCtrl;
	
	HomingModeType homingMode;
	
	EncoderType  encoder;	
	
	CommutationType commutation;
	
	CommandType command;
	
}DriverType;


/** 
  * @brief  Control type structure definition  
  * @note     
  */
typedef struct
{
	float DesiredValue[8];
	
	float KP[4];
	
	float KI[4];

}CtrlType;


	
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/********************FOC����******************************/
#define	 VOL_AMP					12.0f				//Voltage amplitude �����ڵ����ĵ�ѹ��ֵԼ0.8--13.6A
#define  VOL_MAX 					18.00f			//��ѹ���ֵ
#define  VOL_BLIND_AREA		0.80f				//���ä����������������ʸ����ѹֵ
#define	 EMF_CONSTANT			0.50926f		//�綯�Ƴ��������������� = ��ѹʸ��(V)/�ٶ�(pulse/ms)

#define  CURRENT_MAX_3508    20.0f
#define  CURRENT_MAX_2006    6.0f

#define  VEL_MAX_3508				 1280.0f			//����ٶ�
#define  VEL_KP_3508   			 0.1f			  //�ٶȻ�Kp
#define  VEL_KI_3508				 0.001f			//�ٶȻ�Ki
#define  POS_KP_3508         0.11f
#define  POS_KD_3508         0.0f

#define  VEL_MAX_2006				 2400.0f			//����ٶ�
#define  VEL_KP_2006   			 0.05f			//�ٶȻ�Kp
#define  VEL_KI_2006				 0.003f			//�ٶȻ�Ki
#define  POS_KP_2006         0.027f
#define  POS_KD_2006         0.05f

//����������ģʽ
#define  SPEED_CONTROL_MODE				2
#define  POSITION_CONTROL_MODE		5
#define  TORQUE_CONTROL_MODE			3
#define  ZERO_POSITION_INIT_MODE       4
#define  HOMING_MODE							6
#define  PT_MODE									7

//����ʹ��б��ǰ����ٶ�
#define  CMD   0
#define  SOFT  1
#define  MAX_V 2

//����ģʽ
#define  BLDC_MODE			1
#define  FOC_MODE				2

#define  CAN_ID_NUM     5
//�Զ�5�ų�ʼ  ����Ϊ2.5����Ϊ1.5


/* Exported functions ------------------------------------------------------- */
float 	OutPutLim(float val);
float   VelSlope(VelCtrlType *velPid);
float   VelPidCtrl(VelCtrlType *velPid);
float   PosCtrl(PosCtrlType *posPid);
float PTCtrl(uint8_t motorNum, PTCtrlType *pvtPid, PosCtrlType *posPid, VelCtrlType *velPid);
//float 	VelCtrl(float cmdVel);
void 		VelCtrlInit(void);
void		PosCtrlInit(void);
//float	 	CalculSpeed(void);
float   CalculSpeed_Pos(DriverType *driver,MotorType *motor);
float 	GetVelPidOut(void);
float		GetSpeed(void);
float 	GetPosPidOut(void);
float 	MaxMinLimit(float val,float limit);
void 		DriverInit(void);
void    MotorCtrl(void);
void    HomingMode(DriverType *driver);
void 		HomingModeInit(void);
void    MotorOn(int n);
void    MotorOff(int n);
void    VelCtrlTest(float vel,int tim);
void 		ZeroPosInit(void);
void 		SetPtFlag(uint8_t motorNum, uint32_t flag);
uint8_t CheckPtFlag(uint8_t motorNum, uint32_t flag);
float PtVelSlope(uint8_t motorNum,VelCtrlType *velPid,PTCtrlType *ptPid);
#endif

/****************** (C) COPYRIGHT 2016 ACTION *****END OF FILE*************/

