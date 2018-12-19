/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Template/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    13-April-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32f4xx.h"
#include "rm_motor.h"
#include "ctrl.h"
#include "comm.h"
#include "can.h"
#include "four_leg.h"
/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/************************************************************/
/****************������CAN1�ӿ�ģ��****start******************/

extern MotorType Motor[8];
extern DriverType Driver[8];


/**
  * @brief  CAN1�����жϣ�����C620����ķ�������
	* @param  None
	* @retval None
  */
void CAN1_RX0_IRQHandler(void)
{
	uint8_t buffer[8];
	uint8_t n = 0;
	uint32_t stdId=0;
  UnionDataType Msg;
	
	CAN_RxMsg(CAN1, &stdId,buffer,8);
	
	for(uint8_t i = 0; i < 8; i++)
		Msg.data8[i] = buffer[i];
	
	if((stdId==0x201)||(stdId==0x202)||(stdId==0x203)||(stdId==0x204)||
		 (stdId==0x205)||(stdId==0x206)||(stdId==0x207)||(stdId==0x208))
	{
		n = stdId - 0x201;
		Motor[n].pos = (Msg.data8[0]<<8)+Msg.data8[1];
		Motor[n].vel = (int16_t)(Msg.data8[2]<<8)+Msg.data8[3];
		Motor[n].cur = (Msg.data8[4]<<8)+Msg.data8[5];
		Motor[n].temp = Msg.data8[6];
	}

	CAN_ClearFlag(CAN1,CAN_FLAG_EWG);
	CAN_ClearFlag(CAN1,CAN_FLAG_EPV);
	CAN_ClearFlag(CAN1,CAN_FLAG_BOF);
	CAN_ClearFlag(CAN1,CAN_FLAG_LEC);
	
	CAN_ClearFlag(CAN1,CAN_FLAG_FMP0);
	CAN_ClearFlag(CAN1,CAN_FLAG_FF0);
	CAN_ClearFlag(CAN1,CAN_FLAG_FOV0);
	CAN_ClearFlag(CAN1,CAN_FLAG_FMP1);
	CAN_ClearFlag(CAN1,CAN_FLAG_FF1);
	CAN_ClearFlag(CAN1,CAN_FLAG_FOV1);
} 

//CAN2���տ�������
void CAN2_RX0_IRQHandler(void)
{
	uint8_t buffer[8];
	uint32_t StdId=0;
	UnionDataType Msg1;

	CAN_RxMsg(CAN2, &StdId,buffer,8);
	
	for(uint8_t i = 0; i < 8; i++)
		Msg1.data8[i] = buffer[i];

	for(int j = 0; j < 8; j++)
	{
		if(Motor[j].type == NONE)
			break;

		if(StdId == (0x300 + Driver[j].command.canId))
		{		
			if(!CheckPtFlag(j,RECEIVE_QN))
			{
				switch(Msg1.data32[0])
				{
					case 0x00004F4D:						//MO 
						if(Msg1.data32[1] == 1){
							MotorOn(j);
						}else{
							MotorOff(j);
						}
						break; 
					case 0x0000564A:						//JV
						Driver[j].velCtrl.desiredVel[CMD] = (float)(Msg1.data32[1])/1000.0f;
						Driver[j].velCtrl.desiredVel[CMD] = MaxMinLimit(Driver[j].velCtrl.desiredVel[CMD],Driver[j].velCtrl.desiredVel[MAX_V]);
						break;
					case 0x00004341:						//AC
						Driver[j].velCtrl.acc = (float)(Msg1.data32[1])/1000000.0f;
						Driver[j].posCtrl.acc = Driver[j].velCtrl.acc;
						break;
					case 0x00004344:						//DC
						Driver[j].velCtrl.dec = (float)(Msg1.data32[1])/1000000.0f;
						break;
					case 0x00005053:						//SP
						Driver[j].posCtrl.posVel  = (float)(Msg1.data32[1])/1000.0f;
						break;
					case 0x00004150:						//PA����λ��
						Driver[j].posCtrl.desiredPos = (float)(Msg1.data32[1]);
						break;
					case 0x00005250:						//PR���λ��
						Driver[j].posCtrl.desiredPos = Driver[j].posCtrl.actualPos + (float)(Msg1.data32[1]);
						break;
					case 0x00005450:  //PT
						SetPtFlag(j,RECEIVE_START_AND_MP);
						break;
					case 0x40004742:
						if(!CheckPtFlag(j,RECEIVE_QN))
						{
							SetPtFlag(j,RECEIVE_BEGIN);
						}
						break;
					case 0x40005149:						//IQ	 ��ȡ����
						Driver[j].command.can_status = 0x40005149;					
						break;
					case 0x40005856:						//VX   ��ȡ�ٶ�
						Driver[j].command.can_status = 0x40005856;
						break;
					case 0x40005850:						//PX   ��ȡλ��
						Driver[j].command.can_status = 0x40005850;
						break;
					case 0x4000534D:						//MS	 ׁȡ֧·
						Driver[j].command.can_status = 0x4000534D;					
						break;
					default:break;
				} 
			}
			PtCanHandler(j,Msg1);
		}
			else if(StdId == (0x300+0))
			{
				
				switch(Msg1.data32[0])
				{
					case 0x40005149:						//IQ	 ��ȡ����
						Driver[j].command.can_status = 0x40005149;					
						break;
					case 0x40005856:						//VX   ��ȡ�ٶ�
						Driver[j].command.can_status = 0x40005856;
						break;
					case 0x40005850:						//PX   ��ȡλ��
						Driver[j].command.can_status = 0x40005850;
						break;
					default:break;
				}
			}
	}	

	CAN_ClearFlag(CAN2,CAN_FLAG_EWG);
	CAN_ClearFlag(CAN2,CAN_FLAG_EPV);
	CAN_ClearFlag(CAN2,CAN_FLAG_BOF);
	CAN_ClearFlag(CAN2,CAN_FLAG_LEC);
	
	CAN_ClearFlag(CAN2,CAN_FLAG_FMP0);
	CAN_ClearFlag(CAN2,CAN_FLAG_FF0);
	CAN_ClearFlag(CAN2,CAN_FLAG_FOV0);
	CAN_ClearFlag(CAN2,CAN_FLAG_FMP1);
	CAN_ClearFlag(CAN2,CAN_FLAG_FF1);
	CAN_ClearFlag(CAN2,CAN_FLAG_FOV1);
}
/****************������CAN1�ӿ�ģ��****end******************/
/************************************************************/

/*************��ʱ��2******start************/
//ÿ1ms����һ��  ���ڶ�ȡ��������ֵ�ͼ�������


void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update)==SET)
  {	
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		
		MotorCtrl();
		
  }	 
}

//��ʱ��1  ��������ж�
void TIM1_UP_TIM10_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM1, TIM_IT_Update)==SET)    
	{                                                
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}

//��ʱ��8  �ұ������ж�
void TIM8_UP_TIM13_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM8, TIM_IT_Update)==SET)    
	{                                                
		TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
	}
}

/********************************************************/
/*****************��ͨ��ʱTIM5*****Start*****************/
void TIM5_IRQHandler(void)
{

	if(TIM_GetITStatus(TIM5, TIM_IT_Update)==SET)    
	{              
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	}
}

void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3, TIM_IT_Update)==SET)    
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);	
	}
}



//��ʱ��4  
void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4, TIM_IT_Update)==SET)
	{                                  
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
}


void UART5_IRQHandler(void)
{
//	uint8_t data = 0;
	if(USART_GetITStatus(UART5, USART_IT_RXNE)==SET)
	{
		USART_ClearITPendingBit( UART5,USART_IT_RXNE);
//		data=USART_ReceiveData(UART5);
	}
	
}


	uint8_t data = 0;
uint8_t rvData[20] = {0};
void USART3_IRQHandler(void)
{
//	static int32_t i = 0;
	if(USART_GetITStatus(USART3, USART_IT_RXNE)==SET)   
	{
		USART_ClearITPendingBit( USART3,USART_IT_RXNE);
		data=USART_ReceiveData(USART3);
		
		USART_CMD_Hander(USART3,data);

		
//		rvData[i] = data;
//		i++;
//		if(i>=20)
//			i = 0;
//		if(data == 'M')
//		{
//			USART_OUT(USART3,"NOT START WITH 'A'\r\n");
//		}
		
	}
	 
}


/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
   while (1)
   {
   }
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{

  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{

  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
 
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

