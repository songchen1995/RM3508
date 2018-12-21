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
#include "timer.h"
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
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;
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

int32_t TLE5012B_GetPos14bit(void)
{
	return TLE5012BData.pos15bit / 4;
}

/**
 * 读取磁编码器的AVAL ASPD FYNC 寄存器
 * 并且存放到TLE5012BData结构体里
 */
static void TLE5012B_AccessRegister(void)
{
	TLE5012BData.pos15bit = ((int32_t)TLE5012ReadAbsPos());
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







#include "stdint.h"

//COMMAND_byte
#define EX 0x80
#define SB 0X10
#define SWOC 0x20
#define SM 0x30
#define RM 0x40
#define RR 0x50
#define WR 0x60
#define RT 0xF0
#define HR 0xD0
#define HS 0xE0
#define NOP 0x00

//MEMORY
/*********************************

**********************************/
#define WRITE_REG_MLX(address,val)  ((WR)|((val&0xff00) << 8) | (val & 0xff) << 16| (address << 26))//((WR)|((val&0xff00) << 8) | (val & 0xff) << 16| (address << 26))
#define READ_REG_MLX(address)        ((RR)| (address << 10))  
//val
#define  SPI_MODE_ONLY 0x0400
#define GAIN_SEL(val) ((val << 4) & 0x70) 
#define RES(z,y,x)  ((z << 9 | y << 7 | x << 5) & 0x7E0) 
#define DIG_FILT(val) ((val << 2) & 0x1C)
#define OSR(val)   (val)

#define Z_AXIS 0x008
#define Y_AXIS 0x004
#define X_AXIS 0x002
#define TEMPERATURE 0x01
#define BURST_SEL_Z 0x0200
#define BURST_SEL_Y 0x0100
#define BURST_SEL_X 0x0080
#define BURST_SEL_T 0x0040

#define TRIG_INT_SEL 0x8000
#define WOC_DIFF 0x1000
#define EXT_TRIG 0x0800
//address
#define ADDRESS0 0x00
#define ADDRESS1 0x01
#define ADDRESS2 0x02
#define ADDRESS3 0x03
#define ADDRESS4 0x04
#define ADDRESS5 0x05
#define ADDRESS6 0x06
#define ADDRESS7 0x07
#define ADDRESS8 0x08
#define ADDRESS9 0x09

void MLX90393_Init(void);
void MLX90393_ReadPos(void);
int32_t MLX90393_GetPosX(void);



static uint8_t posture[8];//Z,Y,X,T
// int16_t dataTest1 = 0x0000; //
// uint16_t dataTest = 0x0000; //
static uint8_t status  = 0;
static uint32_t write[3] ;
static uint32_t read[2] = {0};
static uint8_t write_buffer[10];
void MLX90393_Init(void)
{

	write_buffer[0] = 0x60;
	write_buffer[1] = ((1000)&0xFF00)>>8;
	write_buffer[2] = (1000)&0x00FF;	
	write_buffer[3] = ADDRESS7  << 2;
	TLE5012_CS_ENABLE();
	SPI_TX_ON();
	SPI1_ReadWriteByte(write_buffer[0]);
	SPI1_ReadWriteByte(write_buffer[1]);
	SPI1_ReadWriteByte(write_buffer[2]);
	SPI1_ReadWriteByte(write_buffer[3]);
	
	SPI_TX_OFF();
	safetyWord = SPI1_ReadWriteByte(NOP);
	TLE5012_CS_DISABLE();
	
	
//	write_buffer[0] = 0x60;
//	write_buffer[1] = ((TRIG_INT_SEL|EXT_TRIG)&0xFF00)>>8;
//	write_buffer[2] = ( TRIG_INT_SEL|EXT_TRIG)&0x00FF;
//	write_buffer[3] = ADDRESS1  << 2;
//	TLE5012_CS_ENABLE();
//	SPI_TX_ON();
//	SPI1_ReadWriteByte(write_buffer[0]);
//	SPI1_ReadWriteByte(write_buffer[1]);
//	SPI1_ReadWriteByte(write_buffer[2]);
//	SPI1_ReadWriteByte(write_buffer[3]);
//	SPI_TX_OFF();
//	safetyWord = SPI1_ReadWriteByte(NOP);
//	TLE5012_CS_DISABLE();

//	TLE5012_CS_ENABLE();
//	SPI_TX_ON();
//	write_buffer[0] = (RM|X_AXIS|Y_AXIS);
//	SPI1_ReadWriteByte(write_buffer[0]);
//	SPI_TX_OFF();
//	safetyWord = SPI1_ReadWriteByte(NOP);
//	TLE5012_CS_DISABLE();		

//	TLE5012_CS_ENABLE();
//	SPI_TX_ON();
//	write_buffer[0] = (SWOC|X_AXIS|Y_AXIS);
//	SPI1_ReadWriteByte(write_buffer[0]);
//	SPI_TX_OFF();
//	safetyWord = SPI1_ReadWriteByte(NOP);
//	TLE5012_CS_DISABLE();	
	

//	write_buffer[0] = 0x60;
//	write_buffer[1] = (((RES(0,0,0)))&0xFF00)>>8;
//	write_buffer[2] = (((RES(0,0,0)))&0x00FF);
//	write_buffer[3] = ADDRESS2  << 2;	
//	SPI_TX_ON();
//	SPI1_ReadWriteByte(write_buffer[0]);
//	SPI1_ReadWriteByte(write_buffer[1]);
//	SPI1_ReadWriteByte(write_buffer[2]);
//	SPI1_ReadWriteByte(write_buffer[3]);
//	SPI_TX_OFF();
//	safetyWord = SPI1_ReadWriteByte(NOP);	
//	
//	write_buffer[0] = 0x60;
//	write_buffer[1] = ((0x0C|GAIN_SEL(0))&0xFF00)>>8;
//	write_buffer[2] = (0x0C|GAIN_SEL(0))&0x00FF;
//	write_buffer[3] = ADDRESS0  << 2;	
//	SPI_TX_ON();
//	SPI1_ReadWriteByte(write_buffer[0]);
//	SPI1_ReadWriteByte(write_buffer[1]);
//	SPI1_ReadWriteByte(write_buffer[2]);
//	SPI1_ReadWriteByte(write_buffer[3]);
//	SPI_TX_OFF();
//	safetyWord = SPI1_ReadWriteByte(NOP);	

//	TLE5012_CS_DISABLE();
//	
//	SPI_TX_ON();

}
int a = 0;
void MLX90393_ReadPos(void)
{
	uint8_t statusByte;

	TLE5012_CS_ENABLE();
	SPI_TX_ON();
	write_buffer[0] = (SM|X_AXIS|Y_AXIS);
	SPI1_ReadWriteByte(write_buffer[0]);
	SPI_TX_OFF();
	safetyWord = SPI1_ReadWriteByte(NOP);
	TLE5012_CS_DISABLE();

	USART_OUT(USART3,(uint8_t*)"%d\t",(uint32_t)safetyWord);

	TIM_Delayms(TIM3,1);
	TLE5012_CS_ENABLE();
	write_buffer[0] = (RM|X_AXIS|Y_AXIS);
	SPI_TX_ON();
	SPI1_ReadWriteByte(write_buffer[0]);
	SPI_TX_OFF();
	statusByte = SPI1_ReadWriteByte(NOP);
//	TLE5012_CS_DISABLE();
	for(int i = 0; i < 4; i++)
	{
//		TLE5012_CS_ENABLE();
		posture[i] = SPI1_ReadWriteByte(NOP);
		TIM_Delayms(TIM3,1);
//		TLE5012_CS_DISABLE();
	}
	TLE5012_CS_DISABLE();
	SPI_TX_ON();
	USART_OUT(USART3,(uint8_t*)"%d\t%d\r\n",(uint16_t)(posture[0] * 256 + posture[1]),(uint16_t)(posture[2] * 256 + posture[3]));
}

void MLX90393_RMOnly(void)
{
	TLE5012_CS_ENABLE();
	SPI_TX_ON();
	write_buffer[0] = (RM|X_AXIS|Y_AXIS);
	SPI1_ReadWriteByte(write_buffer[0]);
	SPI_TX_OFF();
	safetyWord = SPI1_ReadWriteByte(NOP);
	TLE5012_CS_DISABLE();		
}

void MLX90393_ExitOnly(void)
{
	write_buffer[0] = EX;
	TLE5012_CS_ENABLE();
	SPI_TX_ON();
	SPI1_ReadWriteByte(write_buffer[0]);
	SPI_TX_OFF();
	safetyWord = SPI1_ReadWriteByte(NOP);
	TLE5012_CS_DISABLE();
}

void MLX90393_SWOCOnly(void)
{
	write_buffer[0] = (SWOC|X_AXIS|Y_AXIS);
	TLE5012_CS_ENABLE();
	SPI_TX_ON();
	SPI1_ReadWriteByte(write_buffer[0]);
	SPI_TX_OFF();
	safetyWord = SPI1_ReadWriteByte(NOP);
	TLE5012_CS_DISABLE();	
}


static int cnt = 0;
void MLX90393_ReadStaus(void)
{
//	cnt++;
//	if(cnt > 100)
//	{
//		MLX90393_RMOnly();
//		cnt = 0;
//	}
	TLE5012_CS_ENABLE();
	SPI_TX_ON();
	write_buffer[0] = (NOP);
	SPI1_ReadWriteByte(write_buffer[0]);
	SPI_TX_OFF();
	safetyWord = SPI1_ReadWriteByte(NOP);
	TLE5012_CS_DISABLE();
	USART_OUT(USART3,(uint8_t*)"%d\r\n",(int)safetyWord);
}





