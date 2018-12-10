#include "usart.h"
#include "math.h"
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stdarg.h"
#include "stdio.h"
#include <string.h>
#include "stm32f4xx_usart.h"
#include "stm32f4xx_dma.h"
#include "ctrl.h"

#define LENGTH 3


/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */



void USART1_Init(uint32_t BaudRate)
{
  GPIO_InitTypeDef 	GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef 	NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //ʹ��GPIOBʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1ʱ��
 
	//����3��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_USART1); //GPIOC10����ΪUSART1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1); //GPIOC11����ΪUSART1
	
	//USART1�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; //GPIOC10��GPIOC11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOB,&GPIO_InitStructure); //��ʼ��

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = BaudRate;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	

	//Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//��������ж�
  USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���1 	

}
void USART3_Init(uint32_t baudRate)
{
	USART_InitTypeDef USART_InitStructure;  
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	USART_InitStructure.USART_BaudRate = baudRate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//------------------------------------------------------------
	 
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

  /* Connect PXx to USARTx_Tx*/
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
  /* Connect PXx to USARTx_Rx*/
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11,  GPIO_AF_USART3);

  /* Configure USART Tx as alternate function  */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Configure USART Rx as alternate function  */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* USART configuration */
  USART_Init(USART3, & USART_InitStructure);
  
	//////////   ����UART5�ж�       ///////////////
	NVIC_InitStructure.NVIC_IRQChannel=USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_2);
	NVIC_Init(&NVIC_InitStructure);

	USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
	/* Enable USART */
	USART_Cmd(USART3, ENABLE);
 //------------------------------------------------------------
	//ʹ��USART3�����ж�,
 
}

//PC12:  UART5 Tx
//PD2 :  UART5 Rx
void UART5_Init(uint32_t BaudRate)
{
	USART_InitTypeDef USART_InitStructure;  
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	USART_InitStructure.USART_BaudRate = BaudRate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//------------------------------------------------------------
	 
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

  /* Connect PXx to USARTx_Tx*/
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
  /* Connect PXx to USARTx_Rx*/
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource2,  GPIO_AF_UART5);

  /* Configure USART Tx as alternate function  */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Configure USART Rx as alternate function  */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* USART configuration */
  USART_Init(UART5, & USART_InitStructure);
  
	//////////   ����UART5�ж�       ///////////////
	NVIC_InitStructure.NVIC_IRQChannel=UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_2);
	NVIC_Init(&NVIC_InitStructure);

	USART_ITConfig(UART5,USART_IT_RXNE,ENABLE);
	/* Enable USART */
	USART_Cmd(UART5, ENABLE);
 //------------------------------------------------------------
	//ʹ��USART5�����ж�,
 
}


#define NUM_SIZE  7
char USART_BUFF[200][NUM_SIZE]={0};

void USART3_DMA_Init(uint32_t BaudRate)
{
  GPIO_InitTypeDef 	GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef 	NVIC_InitStructure;
	DMA_InitTypeDef   DMA_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //ʹ��GPIOCʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//ʹ��USART3ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);

	
	//����3��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_USART3); //GPIOC10����ΪUSART3
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_USART3); //GPIOC11����ΪUSART3
	
	//USART3�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOC10��GPIOC11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOC,&GPIO_InitStructure); //��ʼ��

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = BaudRate;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;	//�շ�ģʽ
  USART_Init(USART3, &USART_InitStructure); //��ʼ������3
  USART_Cmd(USART3, ENABLE);  //ʹ�ܴ���3 	
	USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);
	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//��������ж�

	DMA_InitStructure.DMA_Channel = DMA_Channel_4;     
	DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t)&(USART3->DR); 				// peripheral address, = & USART3->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)USART_BUFF;							// memory address to save DMA data
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;               	// data dirction: peripheral to memory, ie receive maggage from peripheral
	DMA_InitStructure.DMA_BufferSize = 200*NUM_SIZE;                    		//the buffer size, in data unit
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//8 bit data
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;        //8 bit data  32??MCU?1?half-word?16 bits
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream3, &DMA_InitStructure);
	DMA_Cmd(DMA1_Stream3, ENABLE);

	DMA_ITConfig(DMA1_Stream3,DMA_IT_TC,ENABLE);  	

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;//DMA1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		//�����ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��NVIC�Ĵ���

	//Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//�����ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

} 

void DMA1_Stream3_IRQHandler(void) 
{
  if(DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3) != RESET)  
  {
		 DMA_Cmd(DMA1_Stream3, DISABLE ); 
     DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3);
		 DMA_SetCurrDataCounter(DMA1_Stream3,sizeof(USART_BUFF));
  }
}

u32 t_pow(u32 x,u32 n)
{
	u8 i=0;
	u32 result=1;
	for(i=0;i<n;i++)
	   result*=x;
	return result;
}
/*
* @value:Ҫ����DMA��ֵ
*   sign:�Ƿ������  0 ȫΪ�� 1������ 
*    end:������ 
*      p:ָ��ָ��������ƶ�����
*    num:����ĵڼ���
*/
void SendDMA(int value,char sign,char end,char DMA_buff[][NUM_SIZE],int num)
{
	char i=0;
	int temp_v=0;
	temp_v=value;
	
	/*****   �жϷ���   *****/
	if(sign){	
		if(value<0){
			DMA_buff[num][0]='-';
			temp_v*=-1;
		}else{
			DMA_buff[num][0]=' ';
		}
	}else{
		DMA_buff[num][0]=' ';
	}
	
	/*******     �ֽ���ֵ    ********/
	for(i=1;i<NUM_SIZE-1;i++)
	{
		DMA_buff[num][i]=temp_v/t_pow(10,NUM_SIZE-2-i)%10+'0';
	}
	DMA_buff[num][NUM_SIZE-1]=end;
		
}

/**************����DMA��������*****************/
void DMA_Send_Data(int dat1,int dat2)
{
	static uint16_t DMA_Send_count = 0;
	if(DMA_Send_count<=200)  //����λ���Լ��ٶ�
	{
		if(!(DMA_Send_count%2))
			SendDMA(dat1,1,'\t',USART_BUFF,DMA_Send_count++);					//����ʱ��10us
		else
			SendDMA(dat2,1,'\n',USART_BUFF,DMA_Send_count++);		            					 			  
	}
	if(DMA_Send_count==200)
	{																		//ʹ��DMA��������
		DMA_Cmd(DMA1_Stream3, ENABLE);		//1us		
		DMA_Send_count=0;
	}
}




 /****************************************************************************
* ��    �ƣ�void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,...)
* ��    �ܣ���ʽ�������������
* ��ڲ�����USARTx:  ָ������
			Data��   ��������
			...:     ��������
* ���ڲ�������
* ˵    ������ʽ�������������
        	"\r"	�س���	   USART_OUT(USART1, "abcdefg\r")   
			"\n"	���з�	   USART_OUT(USART1, "abcdefg\r\n")
			"%s"	�ַ���	   USART_OUT(USART1, "�ַ����ǣ�%s","abcdefg")
			"%d"	ʮ����	   USART_OUT(USART1, "a=%d",10)
* ���÷������� 
****************************************************************************/

void USART_OUT(USART_TypeDef* USARTx,const uint8_t *Data,...){ 
	const char *s;
    int d;
		uint32_t x;
    char buf[16];
    va_list ap;
    va_start(ap, Data);

	while(*Data!=0){				                          //�ж��Ƿ񵽴��ַ���������
		if(*Data==0x5c){									  //'\'
			switch (*++Data){
				case 'r':							          //�س���
					USART_SendData(USARTx, 0x0d);	   
					Data++;
					break;
				case 'n':							          //���з�
					USART_SendData(USARTx, 0x0a);	
					Data++;
					break;
				default:
					Data++;
				    break;
			}
			
			 
		}
		else if(*Data=='%'){									  //
			switch (*++Data){				
				case 's':										  //�ַ���
					s = va_arg(ap, const char *);
					for ( ; *s; s++) {
						USART_SendData(USARTx,*s);
						while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
          }
					Data++;
          break;
				case 'd':										  //ʮ����
					d = va_arg(ap, int);
					itoa(d, buf, 10);
					for (s = buf; *s; s++) {
						USART_SendData(USARTx,*s);
						while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
					}
					Data++;
                	break;
				case 'x':
					x = va_arg(ap, int);
				
					itoa((x >> 24), buf, 10);
					
					itoa(d, buf, 10);
//					USART_SendData(USARTx,(x >> 24));
//					USART_SendData(USARTx,(x << 8) >> 24);	
//					USART_SendData(USARTx,(x << 16) >> 24);
//					USART_SendData(USARTx,(x << 24) >> 24);
				break;
				default:
					Data++;
				    break;
			}		 
		}
		else USART_SendData(USARTx, *Data++);
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
	}
}

/******************************************************
		��������ת�ַ�������
        char *itoa(int value, char *string, int radix)
		radix=10 ��ʾ��10����	��ʮ���ƣ�ת�����Ϊ0;  

	    ����d=-379;
		ִ��	itoa(d, buf, 10); ��
		
		buf="-379"							   			  
**********************************************************/
char *itoa(int value, char *string, int radix)
{
    int     i, d;
    int     flag = 0;
    char    *ptr = string;

    /* This implementation only works for decimal numbers. */
    if (radix != 10)
    {
        *ptr = 0;
        return string;
    }

    if (!value)
    {
        *ptr++ = 0x30;
        *ptr = 0;
        return string;
    }

    /* if this is a negative value insert the minus sign. */
    if (value < 0)
    {
        *ptr++ = '-';

        /* Make the value positive. */
        value *= -1;
    }

    for (i = 1000000; i > 0; i /= 10)
    {
        d = value / i;

        if (d || flag)
        {
            *ptr++ = (char)(d + 0x30);
            value -= (d * i);
            flag = 1;
        }
    }

    /* Null terminate the string. */
    *ptr = 0;

    return string;

} 

extern DriverType Driver[8];

void USART_CMD_Hander(USART_TypeDef* USARTx,uint8_t data)
{
	static char buffer[20] = {0};
	static int bufferI = 0,reValue = 0,reFlag = 1;
	//ȥ���ո�
	if(data != 0x20)
	{		
		buffer[bufferI] = data;
		bufferI++;
	}
	
	if((buffer[bufferI-1]=='\n')||(buffer[bufferI-1]=='\r')||(buffer[bufferI-1]==';'))//�Ƿ�һ��ָ�����
	{
		if(bufferI == 1)
		{
			bufferI = 0;
		}
		else if(buffer[2]=='=')	//��ֵ�������
		{
			if(buffer[3] == '-')		//����
			{
				reFlag = -1;
				for(int i=4;i<(bufferI-1);i++)
				{
					reValue = reValue*10 + buffer[i] - '0';
				}
				reValue = reFlag * reValue;
			}
			else					//�Ǹ���
			{
				for(int i=3;i<(bufferI-1);i++)
				{
					reValue = reValue*10 + buffer[i] - '0';
				}			
			}
			
			if(strncmp(buffer,"JV", 2)==0)
			{
				Driver[0].velCtrl.desiredVel[CMD] = (float)reValue*0.001f;
				USART_OUT(USARTx,(uint8_t*)"JV=%d;\r\n",(int)(Driver[0].velCtrl.desiredVel[CMD]*1000.0f));
			}
			else if(strncmp(buffer,"AC", 2)==0)
			{
				Driver[0].velCtrl.acc = (float)(reValue)/1000000.0f;
				USART_OUT(USARTx,(uint8_t*)"AC=%d;\r\n",(int)(Driver[0].velCtrl.acc*1000000.0f));
			}
			else if(strncmp(buffer,"DC", 2)==0)
			{
				Driver[0].velCtrl.dec = (float)(reValue)/1000000.0f;
				USART_OUT(USARTx,(uint8_t*)"DC=%d;\r\n",(int)(Driver[0].velCtrl.dec*1000000.0f));
			}
			else if(strncmp(buffer,"SP", 2)==0)
			{
				Driver[0].velCtrl.desiredVel[CMD] = (float)(reValue)*0.001f;
				USART_OUT(USARTx,(uint8_t*)"SP=%d;\r\n",(int)(Driver[0].velCtrl.desiredVel[CMD]*1000.0f));
			}
			else if(strncmp(buffer,"PA", 2)==0)
			{
				Driver[0].posCtrl.desiredPos = (float)(reValue);
				USART_OUT(USARTx,(uint8_t*)"PA=%d;\r\n",(int)(Driver[0].posCtrl.desiredPos));
			}
			else if(strncmp(buffer,"PR", 2)==0)
			{
				Driver[0].posCtrl.desiredPos = (float)(reValue)+Driver[0].posCtrl.actualPos;
				USART_OUT(USARTx,(uint8_t*)"PA=%d;\r\n",(int)(reValue));
			}
			else
			{
				USART_OUT(USARTx,(uint8_t*)"INVALID WRITE CMD!!!\r\n");			
			}
		}
		else
		{
			if(strncmp(buffer,"VX", 2)==0)
			{
				USART_OUT(USARTx,(uint8_t*)"VX%d;\r\n",(int)(Driver[0].velCtrl.speed*1000.0f));
			}
			else if(strncmp(buffer,"PX", 2)==0)
			{
				USART_OUT(USARTx,(uint8_t*)"PX%d;\r\n",(int)(Driver[0].posCtrl.actualPos));				
			}
			else if(strncmp(buffer,"IQ", 2)==0)
			{
				USART_OUT(USARTx,(uint8_t*)"IQ invalid\r\n");
			}
			else
			{
				USART_OUT(USARTx,(uint8_t*)"INVALID READ CMD!!!\r\n");			
			}

		}
		//��ճ�ʼ����
		bufferI = 0;
		reValue = 0;
		reFlag = 1;
	}
 	else
	{
//			bufferI = 0;
//			USART_OUT(USARTx,"NOT START WITH 'A'\r\n");
	}
}


 
