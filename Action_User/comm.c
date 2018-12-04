/**
  ******************************************************************************
  * @file    comunication 
  * @author  Tmax Sco
  * @version V1.0.0
  * @date    2017.12.31
  * @brief   ÓÃÓÚÓëÖ÷¿ØÍ¨ĞÅ£¬ÓëÏÂ¼¶C620µçµ÷Í¨ĞÅ
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/
#include "comm.h"
#include "can.h"
#include "ctrl.h"
#include "rm_motor.h"

extern MotorType Motor[8];
extern DriverType Driver[8];

/**
  * @brief  Éè¶¨µç»úµÄÔËĞĞµçÁ÷
  * @param 
  * @param 
  * @retval 
  */
void SetCur(float* cur)
{
  uint8_t mbox;	 
	CanTxMsg TxMessage;
	int16_t data[4] = {0};
	
	for(int i = 0; i < 4; i++)
		data[i] = (int16_t)(cur[i]);
	
	TxMessage.StdId=0x200;					     // standard identifier=0
	TxMessage.ExtId=0x200;					     // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			   // the type of frame for the message that will be transmitted
	TxMessage.DLC=8;
					 
 	TxMessage.Data[0] = (data[0]>>8)&0xff;
	TxMessage.Data[1] = data[0]&0xff;
	TxMessage.Data[2] = (data[1]>>8)&0xff;
	TxMessage.Data[3] = data[1]&0xff;
	TxMessage.Data[4] = (data[2]>>8)&0xff;
	TxMessage.Data[5] = data[2]&0xff;
	TxMessage.Data[6] = (data[3]>>8)&0xff;
	TxMessage.Data[7] = data[3]&0xff;
		
	mbox= CAN_Transmit(CAN1, &TxMessage);         //1.4us	
	while((CAN_TransmitStatus(CAN1, mbox)!= CAN_TxStatus_Ok));//µÈ´ı238us
}
	
/**
  * @brief  »ØÓ¦Ö÷¿ØÇëÇó
  * @attention CAN·¢ËÍÊ±¼ä½Ï³¤£¬´Ëº¯ÊıÒ»¶¨Òª·ÅÔÚµÍÓÅÏÈ¼¶ÈÎÎñÖĞ
  * @param 
  * @param 
  * @retval 
  */
void CANRespond(void)
{
	UnionDataType txData;

	for(int i = 0; i < 8; i++)
	{
		if(Motor[i].type == NONE)
			break;
		
		switch (Driver[i].command.can_status)
		{
			case 0:
				break;
			case 0x40005856: //VX   ¶ÁÈ¡ËÙ¶È
				txData.data32[0] = 0x00005856;
				txData.data32[1]  = (int32_t)(Driver[i].velCtrl.speed * 1000);
				CanSendData(Driver[i].command.canId,txData);
				Driver[i].command.can_status = 0;
				break;

			case 0x40005149: //IQ	 ¶ÁÈ¡µçÁ÷
				txData.data32[0] = 0x00005149;
				txData.dataf[1] = Motor[i].cur;
				CanSendData(Driver[i].command.canId,txData);
				Driver[i].command.can_status = 0;
				break;

			case 0x40005850: //PX   ¶ÁÈ¡Î»ÖÃ
				txData.data32[0] = 0x00005850;
				txData.data32[1]  = (int32_t)(Driver[i].posCtrl.actualPos);
				CanSendData(Driver[i].command.canId,txData);
				Driver[i].command.can_status = 0;
				break;
			case 0x40004742:
				if(CheckPtFlag(RECEIVE_BEGIN))
				{
					PtSecondBufferHandler();
				}
				Driver[i].command.can_status = 0;
				break;
			default: break;
		}
	}

}

/**
  * @brief  CanSendData
  * @param 
  * @param 
  * @retval 
  */
void CanSendData(int id,UnionDataType txData)
{
	uint8_t mbox;	 
	CanTxMsg TxMessage;

	TxMessage.StdId= (0x280+id); // standard identifier=0
	TxMessage.ExtId= (0x280+id);					     // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			   // the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

	for(int i=0; i<8; i++)
	{
		TxMessage.Data[i] = txData.data8[i];	     // Ö¡ĞÅÏ¢ 
	}
    		
	mbox= CAN_Transmit(CAN2, &TxMessage);         //1.4us	
	while((CAN_TransmitStatus(CAN2, mbox)!= CAN_TxStatus_Ok));//µÈ´ı238us
}

/**
  * @brief  CanSendData
  * @param 
  * @param 
  * @retval 
  */
void PtCanHandler(int id,UnionDataType RxData)
{
	static uint8_t status = 1;
	static uint8_t N = 0;
	if(!CheckPtFlag(RECEIVE_START_AND_MP|RECEIVE_QN))
	{
		status = 1;
	}
	switch(status)
	{
		case 1: 
			SetPtFlag(SECOND_BUFFER_LOADING_CAN_BUFFER);
			SetPtFlag(RECEIVE_START_AND_MP);
			Driver[0].ptCtrl.MP[1] = RxData.data32[1];
			status = 2;
			SetPtFlag(~RECEIVE_START_AND_MP);
			SetPtFlag(RECEIVE_QN);
			break;
		case 2:
			
			if(RxData.data32[0] & 0xB0000000)
			{
				Driver[0].ptCtrl.desiredPos[2][N] = ((RxData.data32[0] << 4) | ( RxData.data32[1] >> 28));  
				N++;
				if(N < Driver[0].ptCtrl.MP[1])
				{
					Driver[0].ptCtrl.desiredPos[2][N] = (RxData.data32[1] << 4) >> 4;  
					N++;
				}
				else//è‹¥æ˜¯åœ¨æ¥æ”¶æ•°ç»„çš„è¿‡ç¨‹å½“ä¸­é‡æ–°ä»STARTå¼€å§‹ï¼Œåˆ™äºŒçº§ç¼“å­˜å¯ä»¥è¢«æ“¦å†™
				{
					status = 0;
					N = 0;
					SetPtFlag(~RECEIVE_QN);
					SetPtFlag(RECEIVE_BEGIN);
					SetPtFlag(~SECOND_BUFFER_LOADING_CAN_BUFFER);
					SetPtFlag(FIRST_BUFFER_LOADING_SECOND_BUFFER);
					Driver[0].ptCtrl.MP[0] = Driver[0].ptCtrl.MP[1];	
					Driver[0].ptCtrl.MP[1] = 0;	
				}
			}	
			else //åšé”™è¯¯åˆ¤æ–­ç”¨
			{
		
			}				
			break;
		default:
			break;
	}
}

void PtSecondBufferHandler(void)
{
	if(CheckPtFlag(FIRST_BUFFER_LOADING_SECOND_BUFFER))
	{
		for(int i = 0; i< Driver[0].ptCtrl.MP[1]; i++)
		{
			Driver[0].ptCtrl.desiredPos[1][i] = Driver[0].ptCtrl.desiredPos[2][i];
			Driver[0].ptCtrl.desiredPos[2][i] = 0;
		}
		SetPtFlag(~RECEIVE_BEGIN);
		SetPtFlag(~FIRST_BUFFER_LOADING_SECOND_BUFFER);
		SetPtFlag(EXECUTOR_LOADING_FIRST_BUFFER);
	}
}

void PtFirstBufferHandler(void)//æ¥æ”¶å®Œä¸Šçº§æ•°ç»„åå°†ä¸Šçº§æ•°ç»„æ¸…ç©º
{
	if(!CheckPtFlag(BEGIN_MOTION))
	{
		if(CheckPtFlag(EXECUTOR_LOADING_FIRST_BUFFER))
		{
			for(int i = 0; i< Driver[0].ptCtrl.MP[1]; i++)
			{
				Driver[0].ptCtrl.desiredPos[0][i] = Driver[0].ptCtrl.desiredPos[1][i];
				Driver[0].ptCtrl.desiredPos[1][i] = 0;
			}
			Driver[0].ptCtrl.size =  Driver[0].ptCtrl.MP[0] >> 24;
			Driver[0].ptCtrl.runMode =  (Driver[0].ptCtrl.MP[0]<<8) >> 24;
			Driver[0].ptCtrl.desiredTime =  (Driver[0].ptCtrl.MP[0]<<16) >> 24;
			Driver[0].ptCtrl.MP[0] = 0;
			SetPtFlag(BEGIN_MOTION);
			SetPtFlag(~EXECUTOR_LOADING_FIRST_BUFFER);		
		}
		else
		{
			if(Driver[0].ptCtrl.runMode == CIRCULAR_MODE)//å¾ªç¯èµ°
			{
				SetPtFlag(BEGIN_MOTION);
				Driver[0].ptCtrl.index = 0;
				Driver[0].ptCtrl.cnt = 0;
			}
			else if(Driver[0].ptCtrl.runMode == SINGLE_MODE)//è¿è¡Œå®Œåä»¥åŸé€Ÿåº¦ç»§ç»­å‘å‰è·‘
			{
//				SetPtFlag(BEGIN_MOTION);
				
			}
			else if(Driver[0].ptCtrl.runMode == RUN_AND_STOP_MOTION_MODE)//è¿è¡Œå®Œåç«‹å³åœä¸‹ï¼›
			{
				Driver[0].ptCtrl.velOutput = 0;
				Driver[0].ptCtrl.posOutput = 0;
				Driver[0].ptCtrl.output = 0;
			}
		}
	}
}



/************************ (C) COPYRIGHT 2017 ACTION *****END OF FILE****/
