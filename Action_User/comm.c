/**
  ******************************************************************************
  * @file    comunication 
  * @author  Tmax Sco
  * @version V1.0.0
  * @date    2017.12.31
  * @brief   用于与主控通信，与下级C620电调通信
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
  * @brief  设定电机的运行电流
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
	while((CAN_TransmitStatus(CAN1, mbox)!= CAN_TxStatus_Ok));//等待238us
}
	
/**
  * @brief  回应主控请求
  * @attention CAN发送时间较长，此函数一定要放在低优先级任务中
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
			case 0x40005856: //VX   读取速度
				txData.data32[0] = 0x00005856;
				txData.data32[1]  = (int32_t)(Driver[i].velCtrl.speed * 1000);
				CanSendData(Driver[i].command.canId,txData);
				Driver[i].command.can_status = 0;
				break;

			case 0x40005149: //IQ	 读取电流
				txData.data32[0] = 0x00005149;
				txData.dataf[1] = Motor[i].cur;
				CanSendData(Driver[i].command.canId,txData);
				Driver[i].command.can_status = 0;
				break;

			case 0x40005850: //PX   读取位置
				txData.data32[0] = 0x00005850;
				txData.data32[1]  = (int32_t)(Driver[i].posCtrl.actualPos);
				CanSendData(Driver[i].command.canId,txData);
				Driver[i].command.can_status = 0;
				break;
			case 0x40004742:
				if(CheckPtFlag(RECEIVE_BEGIN))
				{
					PtSecondBufferHandle();
				}
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
		TxMessage.Data[i] = txData.data8[i];	     // 帧信息 
	}
    		
	mbox= CAN_Transmit(CAN2, &TxMessage);         //1.4us	
	while((CAN_TransmitStatus(CAN2, mbox)!= CAN_TxStatus_Ok));//等待238us
}

/**
  * @brief  CanSendData
  * @param 
  * @param 
  * @retval 
  */
void PtCanHandle(int id,UnionDataType RxData)
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
				else//鑻ユ槸鍦ㄦ帴鏀舵暟缁勭殑杩囩▼褰撲腑閲嶆柊浠嶴TART寮�濮嬶紝鍒欎簩绾х紦瀛樺彲浠ヨ鎿﹀啓
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
			else //鍋氶敊璇垽鏂敤
			{
		
			}				
			break;
		default:
			break;
	}
}

void PtSecondBufferHandle(void)
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

void PtFirstBufferHandler(void)//鎺ユ敹瀹屼笂绾ф暟缁勫悗灏嗕笂绾ф暟缁勬竻绌�
{
	if(CheckPtFlag(EXECUTOR_LOADING_FIRST_BUFFER))
	{
		for(int i = 0; i< Driver[0].ptCtrl.MP[1]; i++)
		{
			Driver[0].ptCtrl.desiredPos[0][i] = Driver[0].ptCtrl.desiredPos[1][i];
			Driver[0].ptCtrl.desiredPos[1][i] = 0;
		}
		Driver[0].ptCtrl.desiredTime =  Driver[0].ptCtrl.MP[0];
		Driver[0].ptCtrl.MP[0] = 0;
		SetPtFlag(NEW_DATA);
		SetPtFlag(~EXECUTOR_LOADING_FIRST_BUFFER);
	}
}

/************************ (C) COPYRIGHT 2017 ACTION *****END OF FILE****/
