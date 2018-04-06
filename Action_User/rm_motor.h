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
#ifndef __rm_motor_h
#define __rm_motor_h

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
/* Exported types ------------------------------------------------------------*/

/** 
  * @brief  �������  3508 or 2006
  */   
typedef enum
{ 
  RM_3508   = 1, 
  M_2006    = 2,
	NONE      = 3  //none��ʾû�нӵ��

}MotorType_TypeDef;

/** 
  * @brief  Robomaster Motor 3508/2006 type structure definition  
  * @note     
  */
typedef struct
{	
	int32_t pos,posLast;
	
	MotorType_TypeDef type;
	
	int32_t vel;		//������ص��ٶȣ���λ�� rpm
	
	int16_t cur;		//������صĵ���ֵ
	
	int8_t temp;		//������¶�
	
}MotorType;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
int32_t GetMotorPos(int i);

#endif

/****************** (C) COPYRIGHT 2016 ACTION *****END OF FILE*************/

