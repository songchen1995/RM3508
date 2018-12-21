/**
 ******************************************************************************
 * @file	  tle5012.h
 * @author	Tmax Sco
 * @version V1.0
 * @date	  2018.4.9
 * @brief	  tle5012读取
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef TLE5012_H_
#define TLE5012_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "tle5012.h"
/* Exported define -----------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
#define TLE5012B_ZERO_MECPOS 80
/**
 * @defgroup TLE5012_define_Constant
 */

#define WRITE 0 //SPI Write Operate
#define READ 1	//SPI Read Operate

#define WR_REG 0x0000 /* Command word ���λΪ 0 д���� */
#define RD_REG 0x8000 /* Command word ���λΪ 1 ������ */

#define LockValue_LADDR 0x0000 /* Command word lock 4bit addresses 0x00:0x04,0x14:0x15,0x20,0x30 */
#define LockValue_HADDR 0x5000 /* Command word lock 4bit addresses 0x05:0x11 */

#define UPD_CMD_1 0x0400 /* Command word the 10th bit is 1, Access to values in update buffer */
#define UPD_CMD_0 0x0000

#define SAFETY_WORD 0X0001
#define NO_SAFETY_WORD 0X0000

#define READ_STAT_VALUE 0x8001
#define READ_ACSTAT_VALUE 0x8011
#define READ_ANGLE_VALUE 0x8021
#define READ_SPEED_VALUE 0x8031 //0x8031

#define WRITE_MOD1_VALUE 0x5061 //0_1010_0_000110_0001
#define MOD1_VALUE 0x0001				//default: 0x4001
#define WRITE_MOD2_VALUE 0x5081 //0_1010_0_001000_0001
#define MOD2_VALUE 0x0809				//default：0x0801  改变正方向
#define WRITE_MOD3_VALUE 0x5091 //0_1010_0_001001_0001
#define MOD3_VALUE 0x0000				//default: 0xFED0  改变ANG_BASE
#define WRITE_MOD4_VALUE 0x50E1 //0_1010_0_001110_0001
#define MOD4_VALUE 0x0080				//default: 0x0E20										//

#define WRITE_IFAB_VALUE 0x50D1
#define IFAB_VALUE 0x000D

#define READ_ADC_X_CMD 0xD101 //1_1010_0_010000_0001
#define READ_ADC_Y_CMD 0xD111 //1_1010_0_010001_0001
#define READ_D_MAG_CMD 0x8141 //1_0000_0_010000_0001
#define READ_IIF_CNT_CMD (RD_REG | LockValue_LADDR | UPD_CMD_0 | (IIF_CNT_ADDR << 4) | SAFETY_WORD)
#define READ_T25O_CMD (RD_REG | LockValue_LADDR | UPD_CMD_0 | (T25O_ADDR << 4) | SAFETY_WORD)
//#define WRITE_MOD4_CMD (WR_REG | LockValue_LADDR | UPD_CMD_0 | (MOD_4_ADDR << 4) | SAFETY_WORD)
/******* TLE5012 registerַ *******************************
STAT 			STATus register 00H
ACSTAT 		ACtivation STATus register 01H
AVAL 			Angle VALue register 02H
ASPD 			Angle SPeeD register 03H
AREV 			Angle REVolution register 04H
FSYNC 		Frame SYNChronization register 05H
MOD_1 		Interface MODe1 register 06H
SIL 			SIL register 07H
MOD_2 		Interface MODe2 register 08H
MOD_3 		Interface MODe3 register 09H
OFFX 			OFFset X 0AH
OFFY 			OFFset Y 0BH
SYNCH 		SYNCHronicity 0CH
IFAB 			IFAB register 0DH
MOD_4 		Interface MODe4 register 0EH
TCO_Y 		Temperature COefficient register 0FH
ADC_X 		ADC X-raw value 10H
ADC_Y 		ADC Y-raw value 11H
D_MAG       Angle vector MAGnitude 14H
T_RAW       Temperature sensor RAW-value 15H
IIF_CNT 	IIF CouNTer value 20H
T25O        Temperature 25 Offset value
*******************************************************/
#define STAT_ADDR 0x00
#define ACSTAT_ADDR 0x01
#define AVAL_ADDR 0x02
#define ASPD_ADDR 0x03
#define AREV_ADDR 0x04

#define FSYNC_ADDR 0x05
#define MOD_1_ADDR 0x06
#define SIL_ADDR 0x07
#define MOD_2_ADDR 0x08
#define MOD_3_ADDR 0x09
#define OFFX_ADDR 0x0A
#define OFFY_ADDR 0x0B
#define SYNCH_ADDR 0x0C
#define IFAB_ADDR 0x0D
#define MOD_4_ADDR 0x0E
#define TCO_Y_ADDR 0x0F
#define ADC_X_ADDR 0x10
#define ADC_Y_ADDR 0x11

#define D_MAG_ADDR 0x14
#define T_RAW_ADDR 0x15
#define IIF_CNT_ADDR 0x20
#define T25O_ADDR 0x30

typedef enum
{
	TLE5012B_UPDATE_0TIME,
	TLE5012B_UPDATE_1TIME,
	TLE5012B_UPDATE_2TIME,
	TLE5012B_UPDATE_3TIME,	
}TLE5012BSyncState_t;

typedef struct
{
	/**
	 * 读出的数据 使用的数据格式是TLE5012B芯片的表示方式
	 */

	// 15bits Data
	int32_t pos15bit;
	int32_t vel15bit;
	uint32_t framSync;

	/**
	 * 转换后的数据
	 */
	float spdCompen;
	float posMec;
	/**
	 * Other Data
	 */
	uint32_t safetyWord;
	float delayRatio;

	//spi读取错误时的错误计数
	uint32_t spiErrCnt;
	//磁编码器检测状态错误计数
	uint32_t MagErrCnt;
	
	uint32_t DelayCnt;
	
	uint32_t DelayPeriod;
	
	uint32_t lastUpdateTimes;
} TLE5012B_data_t;
/* Exported functions ------------------------------------------------------- */
void TLE5012B_Init(void);
void TLE5012B_UpateData(void);
int32_t TLE5012B_GetPos15bit(void);
int32_t TLE5012B_GetPos14bit(void);
void MLX90393_Init(void);
void MLX90393_ReadPos(void);
void MLX90393_ReadStaus(void);
void MLX90393_ExitOnly(void);
void MLX90393_SWOCOnly(void);
void SpiInit(void);
#endif

/************************ (C) COPYRIGHT 2017 ACTION *****END OF FILE****/
