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
	int32_t Tim;	//间隔包含的100us的个数
	
	float TimUite;
	
	int32_t TimNum;	//间隔总时间 单位：us
	
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
/*********************方波换相参数******************************/
//#define	 VOL_AMP					0.90f				//Voltage amplitude 作用于电流的电压幅值约0.8--13.6A
//#define  VOL_MAX 					22.00f			//电压最大值
//#define  VOL_BLIND_AREA		0.50f				//输出盲区，电机不动的最大矢量电压值
//#define	 EMF_CONSTANT			0.022926f		//电动势常数，电机本身参数 = 电压矢量(V)/速度(pulse/ms)
/********************FOC参数******************************/
#define	 VOL_AMP					1.10f				//Voltage amplitude 作用于电流的电压幅值约0.8--13.6A
#define  VOL_MAX 					18.00f			//电压最大值
#define  VOL_BLIND_AREA		0.80f				//输出盲区，电机不动的最大矢量电压值
#define	 EMF_CONSTANT			0.020926f		//电动势常数，电机本身参数 = 电压矢量(V)/速度(pulse/ms)
#define  CURRENT_MAX      20.0f

#define  VEL_MAX					1280.0f			//最大速度
#define  VEl_KP   				0.048f			//速度环Kp
#define  VEL_KI						0.004f			//速度环Ki

//驱动器工作模式
#define  SPEED_CONTROL_MODE				2
#define  POSITION_CONTROL_MODE		5
#define  HOMING_MODE							6

//换相模式
#define  BLDC_MODE			1
#define  FOC_MODE				2

#define  CAN_ID_NUM     5
//自动5号初始  电流为2.5其余为1.5


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

