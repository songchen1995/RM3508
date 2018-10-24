/**
 ******************************************************************************
 * @file	  tle5012.c
 * @author	Tmax Sco
 * @version V1.0
 * @date	  2018.4.9
 * @brief	  tle5012读取
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
/* Includes -------------------------------------------------------------------*/
#include "tle5012.h"
#include <stdint.h>
#include <stdlib.h>
#include <arm_math.h>
#include "gpio.h"
#include "usart.h"
#include "ctrl.h"
//#include "util.h"
//#include "motorconf.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_rcc.h"
/* Extern	 variables ---------------------------------------------------------*/
extern DriverType Driver[];

/**
 * 由于使用SPI单线模式，即数据线只有一条，
 * 进行发送和接受信息时，需要配置发送引脚的模式，使不冲突
 */
#define SPI_TX_OFF()            \
	do                            \
	{                             \
		GPIOA->MODER &= 0xFFFF3FFF; \
		GPIOA->MODER |= 0x00000000; \
	} while (0) // PA7--MOSI输入模式

#define SPI_TX_ON()             \
	do                            \
	{                             \
		GPIOA->MODER &= 0xFFFF3FFF; \
		GPIOA->MODER |= 0x00008000; \
	} while (0) // PA7--MOSI复用
	
#define NSS1_GPIO_Port GPIOA
#define NSS1_Pin GPIO_Pin_4

#define SPI3_NSS_PORT GPIOA
#define SPI3_NSS_PIN GPIO_Pin_15
	
#define TLE5012_CS_ENABLE() GPIO_ResetBits(NSS1_GPIO_Port, NSS1_Pin)
#define TLE5012_CS_DISABLE() GPIO_SetBits(NSS1_GPIO_Port, NSS1_Pin)
/* Private	variables ---------------------------------------------------------*/
TLE5012B_data_t TLE5012BData = {0};

/* Private	function pototype -------------------------------------------------*/

//static AS5045BSyncState_t TLE5012BCheckUpdateTimes(int posDel, float vel);

static void TLE5012B_AccessRegister(void);
static uint16_t TLE5012ReadAbsPos(void); 
static uint16_t TLE5012ReadValue(uint16_t u16RegValue);							//无safety word
static uint16_t TLE5012ReadReg(uint16_t commond, uint16_t *reData); //有safety word
static uint16_t TLE5012ReadRegUpdate(uint16_t commond, uint16_t *reData);

/**
 * @brief
 * @param  None
 * @retval
 */
// int16_t dataTest1 = 0x0000; //
// uint16_t dataTest = 0x0000; //
uint16_t safe = 0x0000;
uint16_t TLE5012RegValue[25] = {0};

void SpiInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//Ê¹ÄÜGPIOAÊ±ÖÓ
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  //GPIOFB3,4,5³õÊ¼»¯ÉèÖÃ
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;//PB3~5¸´ÓÃ¹¦ÄÜÊä³ö	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//¸´ÓÃ¹¦ÄÜ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//ÍÆÍìÊä³ö
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//ÉÏÀ­
  GPIO_Init(GPIOA, &GPIO_InitStructure);//³õÊ¼»¯


	
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource4,GPIO_AF_SPI1); //PB3¸´ÓÃÎª SPI1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1); //PB4¸´ÓÃÎª SPI1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1); //PB4¸´ÓÃÎª SPI1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1); //PB5¸´ÓÃÎª SPI1	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;//PB3~5¸´ÓÃ¹¦ÄÜÊä³ö	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//¸´ÓÃ¹¦ÄÜ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//ÍÆÍìÊä³ö
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//ÉÏÀ­
  GPIO_Init(GPIOA, &GPIO_InitStructure);//³õÊ¼»¯

	GPIO_SetBits(GPIOA,GPIO_Pin_4);
	
	SPI_InitTypeDef SPI_InitStruct;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_16b;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1,&SPI_InitStruct);
	
	SPI_Cmd(SPI1, ENABLE); 
}

//void TLE5012B_Init(void)
//{
//	
//	uint16_t data;

//	safe = TLE5012ReadReg(READ_ADC_X_CMD, &TLE5012RegValue[17]);
//	safe = TLE5012ReadReg(READ_ADC_Y_CMD, &TLE5012RegValue[18]);
//	safe = TLE5012ReadReg(READ_D_MAG_CMD, &TLE5012RegValue[20]);
//	safe = TLE5012ReadReg(READ_IIF_CNT_CMD, &TLE5012RegValue[21]);
//	safe = TLE5012ReadReg(READ_T25O_CMD, &TLE5012RegValue[22]);
//	data = MOD4_VALUE;
//}

void TLE5012B_UpateData(void)
{
	TLE5012B_AccessRegister();
	//To do: Add Code to compensate the mechanical Position error depending on speed
	//TLE5012B_SpdCompensate();
	//TLE5012BData.posMec = util_norm_float(TLE5012BData.pos15bit + TLE5012BData.spdCompen,
	//																			0, ENCODER_POS_PER_REV, ENCODER_POS_PER_REV);
}

int32_t TLE5012B_GetPos15bit(void)
{
	return TLE5012BData.pos15bit;
}



/**
 * 读取磁编码器的AVAL ASPD FYNC 寄存器
 * 并且存放到TLE5012BData结构体里
 */
static void TLE5012B_AccessRegister(void)
{
	TLE5012BData.pos15bit = ((int32_t)TLE5012ReadAbsPos()/4);
	TLE5012BData.posMec = TLE5012BData.pos15bit;
}

static uint8_t tle5012bMagSta;


/**
 * @brief
 * @param  None
 * @retval
 */
static uint16_t TLE5012ReadAbsPos(void)
{
	uint16_t safe = 0;
	uint16_t data = 0;
	safe = TLE5012ReadRegUpdate(READ_ANGLE_VALUE | UPD_CMD_1, &data);
//		safe = TLE5012ReadReg(READ_ANGLE_VALUE | UPD_CMD_0, &data);
	//	DMAPRINTF("%d\t",(safe>>12));
	return (data & 0x7fff);
}


uint16_t SPI1_ReadWriteByte(uint16_t TxData)
{		 			 
 
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}//µÈ´ý·¢ËÍÇø¿Õ  
	
	SPI_I2S_SendData(SPI1, TxData); //Í¨¹ýÍâÉèSPIx·¢ËÍÒ»¸öbyte  Êý¾Ý
		
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){} //µÈ´ý½ÓÊÕÍêÒ»¸öbyte  
 
	return SPI_I2S_ReceiveData(SPI1); //·µ»ØÍ¨¹ýSPIx×î½ü½ÓÊÕµÄÊý¾Ý	
 		    
}


/**
 * @brief  读取TLE5012的寄存器
 * @param  commond：命令字
 * @param  reData：数据指针
 * @retval safety word
 */
static uint16_t TLE5012ReadReg(uint16_t command, uint16_t *reData)
{
	uint16_t safetyWord = 0;
	
	TLE5012_CS_ENABLE();
	SPI1_ReadWriteByte(command);
//	if(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == SET)
//	{
//		SPI_I2S_SendData(SPI1,command);
////		if(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == SET)
//			SPI_I2S_ReceiveData(SPI1);
//	}

	//读取寄存器的值
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();	
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();	
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();	
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();	
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();	
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();	
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();	
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();	
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();	
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();	
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();	
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();	
//	if(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == SET)
//	{
//		SPI_I2S_SendData(SPI1,command);
//	}
	SPI_TX_OFF();
	*reData = SPI1_ReadWriteByte(0xffff);
	
//	if(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == SET)
//		*reData = SPI_I2S_ReceiveData(SPI1);

	{	
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();	
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();	
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();	
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();	
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();	
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();	
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();	
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();	
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();	
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();	
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();	
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();	
	}
	
//	if(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == SET)
	{		
//		SPI_I2S_SendData(SPI1,command);
//		safetyWord = SPI_I2S_ReceiveData(SPI1);
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();	
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();	
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();	
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();	
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();	
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();	
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();	
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();	
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();	
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();	
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();	
//		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();	
		}
	TLE5012_CS_DISABLE();
	SPI_TX_ON();

	return safetyWord;
}

/**
 * @brief  读取TLE5012的寄存器
 * @param  commond：命令字
 * @param  reData：数据指针
 * @retval safety word
 */
	uint16_t safetyWord = 0;
static uint16_t TLE5012ReadRegUpdate(uint16_t command, uint16_t *reData)
{

	TLE5012_CS_ENABLE();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	TLE5012_CS_DISABLE();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	TLE5012_CS_ENABLE();

	SPI1_ReadWriteByte(command);
	SPI_TX_OFF();
	*reData = SPI1_ReadWriteByte(command);
	safetyWord = SPI1_ReadWriteByte(command);

	TLE5012_CS_DISABLE();

	SPI_TX_ON();

	return safetyWord;
}

