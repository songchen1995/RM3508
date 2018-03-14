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
	
	int32_t Period;
	
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
	int32_t CAN_status;
	
}CommandType;

/** 
  * @brief  VelCtrl type structure definition  
  * @note     
  */
typedef struct
{
	float Speed;
	
	float DesiredVel;
	
	float Acc;
	
	float Dec;
	
	float Kp;
	
	float Ki;
	
	float TemI;
	
	float Output;
	
}VelCtrlType;


/** 
  * @brief  PosCtrl type structure definition  
  * @note     
  */
typedef struct
{
	float ActualPos;
	
	float DesiredPos;
	
	float BasicPos;
	
	float Kp;
	
	float Kd;
	
	float Output;
	
}PosCtrlType;

/** 
  * @brief  HomingMode type structure definition  
  * @note     
  */
typedef struct
{
	float Vel;
	
	float InitPos;
	
	int32_t Cnt;
	
	float Output;
	
}HomingModeType;

/** 
  * @brief  Driver type structure definition  
  * @note     
  */
typedef struct
{
	uint32_t UnitMode;
  
  int32_t time;
	
  FunctionalState Status;
  
	float VoltageOutput;
	
	VelCtrlType VelCtrl;
	
	PosCtrlType PosCtrl;
	
	HomingModeType HomingMode;
	
	EncoderType  Encoder;
	
	CommutationType Commutation;
	
	CommandType Command;
	
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
/*********************�����������******************************/
//#define	 VOL_AMP					0.90f				//Voltage amplitude �����ڵ����ĵ�ѹ��ֵԼ0.8--13.6A
//#define  VOL_MAX 					22.00f			//��ѹ���ֵ
//#define  VOL_BLIND_AREA		0.50f				//���ä����������������ʸ����ѹֵ
//#define	 EMF_CONSTANT			0.022926f		//�綯�Ƴ��������������� = ��ѹʸ��(V)/�ٶ�(pulse/ms)
/********************FOC����******************************/
#define	 VOL_AMP					1.10f				//Voltage amplitude �����ڵ����ĵ�ѹ��ֵԼ0.8--13.6A
#define  VOL_MAX 					18.00f			//��ѹ���ֵ
#define  VOL_BLIND_AREA		0.80f				//���ä����������������ʸ����ѹֵ
#define	 EMF_CONSTANT			0.020926f		//�綯�Ƴ��������������� = ��ѹʸ��(V)/�ٶ�(pulse/ms)
#define  CURRENT_MAX      20.0f

#define  VEL_MAX					1280.0f			//����ٶ�
#define  VEl_KP   				0.048f			//�ٶȻ�Kp
#define  VEL_KI						0.004f			//�ٶȻ�Ki

//����������ģʽ
#define  SPEED_CONTROL_MODE				2
#define  POSITION_CONTROL_MODE		5
#define  HOMING_MODE							6

//����ģʽ
#define  BLDC_MODE			1
#define  FOC_MODE				2

#define  CAN_ID_NUM     5
//�Զ�5�ų�ʼ  ����Ϊ2.5����Ϊ1.5


/* Exported functions ------------------------------------------------------- */
float 	OutPutLim(float val);
float 	VelSlope(float cmdVel);
float 	VelCtrl(float cmdVel);
void 		VelCtrlInit(void);
float 	PosCtrl(void);
void		PosCtrlInit(void);
float	 	CalculSpeed(void);
float 	GetVelPidOut(void);
float		GetSpeed(void);
float 	GetPosPidOut(void);
float 	MaxMinLimit(float val,float limit);
void 		DriverInit(void);
void    MotorCtrl(void);
void 		HomingMode(void);
void 		HomingModeInit(void);
void    MotorOn(void);
void    MotorOff(void);
void    VelCtrlTest(float vel,int tim);


#endif

/****************** (C) COPYRIGHT 2016 ACTION *****END OF FILE*************/

