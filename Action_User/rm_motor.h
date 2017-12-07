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
  * @brief  Robomaster Motor 3508 type structure definition  
  * @note     
  */
typedef struct
{	
	int32_t Pos;
	
	int32_t Vel;		//������ص��ٶȣ���λ�� rpm
	
	int16_t Cur;		//������صĵ���ֵ
	
	int8_t Temp;		//������¶�
	
}MotorType;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
int32_t GetMotorPos(int i);

#endif

/****************** (C) COPYRIGHT 2016 ACTION *****END OF FILE*************/

