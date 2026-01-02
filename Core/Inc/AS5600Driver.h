/*
 * AS5600Driver.h
 *
 *  Created on: Oct 25, 2025
 *      Author: Z440
 */



#ifndef INC_AS5600DRIVER_H_
#define INC_AS5600DRIVER_H_

#include <AS5600Driver_Config.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>
#include <stdlib.h>

#define __IO	volatile

#include <stdarg.h>
#include <stdint.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>


#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_tim.h"
#include "stm32f7xx_hal_tim.h"
#include "stm32f7xx_hal_tim_ex.h"
#include "stm32f7xx_hal_conf.h"







//Register adresses
#define AS5600_REGISTER_ZMCO 0x00
#define AS5600_REGISTER_ZPOS_HIGH 0x01
#define AS5600_REGISTER_ZPOS_LOW 0x02
#define AS5600_REGISTER_MPOS_HIGH 0x03
#define AS5600_REGISTER_MPOS_LOW 0x04
#define AS5600_REGISTER_MANG_HIGH 0x05
#define AS5600_REGISTER_MANG_LOW 0x06
#define AS5600_REGISTER_CONF_HIGH 0x07
#define AS5600_REGISTER_CONF_LOW 0x08

#define AS5600_REGISTER_RAW_ANGLE_HIGH 0x0C
#define AS5600_REGISTER_RAW_ANGLE_LOW 0x0D
#define AS5600_REGISTER_ANGLE_HIGH 0x0E
#define AS5600_REGISTER_ANGLE_LOW 0x0F

#define AS5600_REGISTER_STATUS 0x0B
#define AS5600_REGISTER_AGC 0x1A
#define AS5600_REGISTER_MAGNITUDE_HIGH 0x1B
#define AS5600_REGISTER_MAGNITUDE_LOW 0x1C
#define AS5600_REGISTER_BURN 0xFF

//Bit pos
#define AS5600_MD_BITSHIFT	5
#define AS5600_ML_BITSHIFT	4
#define AS5600_MH_BITSHIFT	3

//Bitmasks
#define 	AS5600_RAW_ANGLE_UPPER_8_BIT_MASK	0b00001111
#define 	AS5600_RAW_ANGLE_LOWER_8_BIT_MASK	0b11111111
#define		AS5600_RAW_ANGLE_16_BIT_MASK		0b0000111111111111

#define 	AS5600_MD_BIT_MASK	0b00100000
#define 	AS5600_ML_BIT_MASK	0b00010000
#define 	AS5600_MH_BIT_MASK	0b00001000



//Dutycycle defines
#define DUTYCYCLE_128_CLOCK 2.9418524477f  // 128/4351
#define DUTYCYCLE_4095_CLOCK 94.1162951f	//4095/4351

typedef enum{
	AS5600_OK,
	AS5600_NULL_POINTER,
	AS5600_CALLOC_FAIL,
	AS5600_I2C_COMM_ERROR,
	AS5600_PWM_MODE_NOT_INITIALIZED,
	AS5600_INPUT_PWM_FREQ_ERROR,
	AS5600_INPUT_PWM_DUTYCYCLE_ERROR,
	AS5600_DEVIDE_BY_ZERO,
	AS5600_INVALID_MAX_ANGLE,
	AS5600_INVALID_MIN_ANGLE,
	AS5600_INVALID_MIN_MAX_ANGLE,


	AS5600_OTHER_ERROR,
}eInfo;

typedef enum{
	NOM = 0,
	LPM1 = 1,
	LPM2 = 2,
	LPM3 = 3
}eConf_PowerMode;

typedef enum{
	OFF = 0,
	LSB1 = 1,
	LSB2 = 2,
	LSB3 = 3
}eConf_Hysteresis;

typedef enum{
	AnalogFullRange = 0,
	AnalogReducedRange = 1,
	PWM = 2
}eConf_OutputStage;

typedef enum{
	_115Hz = 0,
	_230Hz = 1,
	_460Hz = 2,
	_920Hz = 3
}eConf_PWMFrequency;

typedef enum{
	_16x = 0,
	_8x = 1,
	_4x = 2,
	_2x = 3
}eConf_SlowFilter;

typedef enum{
	SlowFilterOnly = 0,
	LSB6 = 1,
	LSB7 = 2,
	LSB9 = 3,
	LSB18 = 4,
	LSB21 = 5,
	LSB24 = 6,
	LSB10 = 7
}eConf_FastFilterThreshold;

typedef enum{
	Off = 0,
	On = 1,
}eConf_Watchdog;

typedef enum{
	Wait,
	WatchForOverflow,
	WatchForUnderflow,
}RotationStateMachine;


typedef struct{
	uint8_t MagnetTooStrong;
	uint8_t MagnetTooWeak;
	uint8_t MagnetDetected;

}AS5600Status_Typedef;

typedef struct{
	eConf_PowerMode PowerMode;
	eConf_Hysteresis Hysteresis;
	eConf_OutputStage OutputStage;
	eConf_PWMFrequency PWMFrequency;
	eConf_SlowFilter SlowFilter;
	eConf_FastFilterThreshold FastFilterTreshold;
	eConf_Watchdog Watchdog;

}AS5600Config_Typedef;

typedef struct{
	I2C_HandleTypeDef *hi2c;
	TIM_HandleTypeDef *htim;
	uint8_t I2CAddress;
	float MaxAngle;
	float MinAngle;
	uint16_t RawAngle;
	float Angle;
	float LastAngle;
	int64_t FullRotationCounter;
	float AbsolutePosition;
	RotationStateMachine State;

	AS5600Status_Typedef Status;
	AS5600Config_Typedef Config;

	uint32_t TIMFreq;

}AS5600Handle_Typedef;



void AS5600_Peripherial_ErrorHandler()
{

}

/* TIM2 init function */
void AS5600_TimerInit(AS5600Handle_Typedef *pAS)
{

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  //pAS->htim->Instance = TIM2;
  pAS->htim->Init.Prescaler = 0;
  pAS->htim->Init.CounterMode = TIM_COUNTERMODE_UP;
  pAS->htim->Init.Period = 4294967295;
  pAS->htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  pAS->htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(pAS->htim) != HAL_OK)
  {
	  AS5600_Peripherial_ErrorHandler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(pAS->htim, &sSlaveConfig) != HAL_OK)
  {
	  AS5600_Peripherial_ErrorHandler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(pAS->htim, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
	  AS5600_Peripherial_ErrorHandler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(pAS->htim, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
	  AS5600_Peripherial_ErrorHandler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(pAS->htim, &sMasterConfig) != HAL_OK)
  {
	  AS5600_Peripherial_ErrorHandler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}



/*
 * AS5600Driver.c
 *
 *  Created on: Oct 25, 2025
 *      Author: Füzi Gergő
 */

#if (config_USE_ERROR_HANDLING == 1)
#define AS5600_ERROR_HANDLE_CALLBACK(x, y) 		AS5600_ErrorHandler((x), (y))
#else
#define AS5600_ERROR_HANDLE_CALLBACK(x, y)
#endif
__attribute__((weak)) void AS5600_ErrorHandler(AS5600Handle_Typedef *pAS, eInfo error)
{
	switch(error)
	{
	case AS5600_OK:
		while(1);
		break;

	case AS5600_NULL_POINTER:
		while(1);
		break;

	case AS5600_CALLOC_FAIL:
		while(1);
		break;

	case AS5600_I2C_COMM_ERROR:
		while(1);
		break;

	case AS5600_PWM_MODE_NOT_INITIALIZED:
		while(1);
		break;

	case AS5600_INPUT_PWM_FREQ_ERROR:
		//while(1);
		break;


	case AS5600_INPUT_PWM_DUTYCYCLE_ERROR:
		while(1);
		break;

	case AS5600_DEVIDE_BY_ZERO:
		while(1);
		break;

	case AS5600_INVALID_MAX_ANGLE:
		while(1);
		break;

	case AS5600_INVALID_MIN_ANGLE:
		while(1);
		break;

	case AS5600_INVALID_MIN_MAX_ANGLE:
		while(1);
		break;

	case AS5600_OTHER_ERROR:
		while(1);
		break;


	}
}

float MapDutycycle2Angle(float Duty, float AngleMin, float AngleMax)
{


	float PosVal = Duty - (DUTYCYCLE_128_CLOCK);			//Offset (-2.941)
	PosVal = PosVal * (100.0f / DUTYCYCLE_4095_CLOCK);		//Scale (0-94.116 -> 0-100)
	PosVal = PosVal / 100.0f;								//Normalize (0-100 -> 0-1)


	float PosAngle = (PosVal * (AngleMax - AngleMin)) + AngleMin;
	return PosAngle;
}

/**
  * @brief
  *
  * @param
  *
  * @retval
  */
AS5600Handle_Typedef *AS5600_Create(//I2C_HandleTypeDef *hi2c,
									//TIM_HandleTypeDef *htim,
									uint8_t i2cAddr,
									float MaxAngle,
									float MinAngle)
{
	if(MaxAngle > 360.0f)
	{
		printf("Invalid max angle");
		AS5600_ERROR_HANDLE_CALLBACK(NULL, AS5600_INVALID_MAX_ANGLE);
		return NULL;
	}

	if(MaxAngle < 0.0f)
	{
		printf("Invalid min angle");
		AS5600_ERROR_HANDLE_CALLBACK(NULL, AS5600_INVALID_MIN_ANGLE);
		return NULL;
	}

	if(MinAngle > MaxAngle)
	{
		printf("Invalid min/max angle");
		AS5600_ERROR_HANDLE_CALLBACK(NULL, AS5600_INVALID_MIN_MAX_ANGLE);
		return NULL;
	}

	AS5600Handle_Typedef *pAS = (AS5600Handle_Typedef *)calloc(1, sizeof(AS5600Handle_Typedef));
	if (pAS == NULL)
	{
		printf("calloc fail");
		AS5600_ERROR_HANDLE_CALLBACK(NULL, AS5600_CALLOC_FAIL);
		return NULL;
	}


	/*
	if (htim == NULL)
	{
		pAS->htim = (TIM_HandleTypeDef *)calloc(1, sizeof(TIM_HandleTypeDef));
		if (pAS->htim == NULL)
		{
			free(pAS);
			printf("calloc fail");
			AS5600_ERROR_HANDLE_CALLBACK(NULL, AS5600_CALLOC_FAIL);
			return NULL;
		}
	}
	else
	{
		pAS->htim = htim;
		//memset(pAS->htim->Instance, 0, (sizeof(pAS->htim->Instance)));
		if(pAS->htim->Instance == TIM2)
		{
			//Reset the pheripherial if already initialized
			__HAL_RCC_TIM2_FORCE_RESET();
			//Wait some time
			HAL_Delay(5);
			//Release the reset bit
			__HAL_RCC_TIM2_RELEASE_RESET();
		}
	}





	pAS->hi2c = hi2c;
	*/
	pAS->I2CAddress = i2cAddr;
	pAS->MaxAngle = MaxAngle;
	pAS->MinAngle = MinAngle;


	return pAS;
}








/**
  * @brief
  *
  * @param
  *
  * @retval
  */
eInfo AS5600_AttachPeripheral (AS5600Handle_Typedef *pAS,
									I2C_HandleTypeDef *hi2c,
									TIM_HandleTypeDef *htim)
{
	if (htim != NULL)
	{
		pAS->htim = htim;
		//memset(pAS->htim->Instance, 0, (sizeof(pAS->htim->Instance)));
		if(pAS->htim->Instance == TIM2)
		{
			//Reset the pheripherial if already initialized
			__HAL_RCC_TIM2_FORCE_RESET();
			//Wait some time
			HAL_Delay(5);
			//Release the reset bit
			__HAL_RCC_TIM2_RELEASE_RESET();

			HAL_Delay(5);

			AS5600_TimerInit(pAS);

			uint32_t clockHz;
			clockHz = HAL_RCC_GetPCLK1Freq() / (pAS->htim->Init.ClockDivision + 1);
			clockHz *= 2;

			pAS->TIMFreq = clockHz;
		}
	}

	if (hi2c != NULL)
	{
		pAS->hi2c = hi2c;
	}
	return AS5600_OK;
}







/**
  * @brief
  *
  * @param
  *
  * @retval
  */
eInfo AS5600_Configure(AS5600Handle_Typedef *pAS,
							eConf_PowerMode PM,
							eConf_Hysteresis HYST,
							eConf_OutputStage OUTS,
							eConf_PWMFrequency PWMF,
							eConf_SlowFilter SF,
							eConf_FastFilterThreshold FTH,
							eConf_Watchdog WD)
{
	if(pAS == NULL)
	{
		AS5600_ERROR_HANDLE_CALLBACK(pAS, AS5600_NULL_POINTER);
		return AS5600_NULL_POINTER;
	}

	pAS->Config.PowerMode = PM;
	pAS->Config.Hysteresis = HYST;
	pAS->Config.OutputStage = OUTS;
	pAS->Config.PWMFrequency = PWMF;
	pAS->Config.SlowFilter = SF;
	pAS->Config.FastFilterTreshold = FTH;
	pAS->Config.Watchdog = WD;

	if (pAS->Config.OutputStage == PWM)
	{
		//AS5600_TimerInit(pAS);
		HAL_TIM_IC_Start(pAS->htim, TIM_CHANNEL_1);
		HAL_TIM_IC_Start(pAS->htim, TIM_CHANNEL_2);
	}

	uint16_t temp16_t = 0;

	temp16_t |= ((0x0001 & (uint16_t)pAS->Config.Watchdog) << 13);
	temp16_t |= ((0x0007 & (uint16_t)pAS->Config.FastFilterTreshold) << 10);
	temp16_t |= ((0x0003 & (uint16_t)pAS->Config.SlowFilter) << 8);
	temp16_t |= ((0x0003 & (uint16_t)pAS->Config.PWMFrequency) << 6);
	temp16_t |= ((0x0003 & (uint16_t)pAS->Config.OutputStage) << 4);
	temp16_t |= ((0x0003 & (uint16_t)pAS->Config.Hysteresis) << 2);
	temp16_t |= (0x0003 & (uint16_t)pAS->Config.PowerMode);

	uint8_t temp[2];

	temp[0] = (uint8_t)(0x00FF & (temp16_t >> 8));
	temp[1] = (uint8_t)(0x00FF & temp16_t);

	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Write(pAS->hi2c, (pAS->I2CAddress << 1), AS5600_REGISTER_CONF_HIGH, I2C_MEMADD_SIZE_8BIT, temp, 2, 1000);

	if (status != HAL_OK)
	{
		AS5600_ERROR_HANDLE_CALLBACK(pAS, AS5600_I2C_COMM_ERROR);
		return AS5600_I2C_COMM_ERROR;
	}

	return AS5600_OK;
}


/**
  * @brief
  *
  * @param
  *
  * @retval
  */
eInfo AS5600_UpdateStatus(AS5600Handle_Typedef *pAS)
{
	if(pAS == NULL)
	{
		AS5600_ERROR_HANDLE_CALLBACK(pAS, AS5600_NULL_POINTER);
		return AS5600_NULL_POINTER;
	}

	uint8_t temp[1];

	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Read(pAS->hi2c, (pAS->I2CAddress << 1), AS5600_REGISTER_STATUS, I2C_MEMADD_SIZE_8BIT, temp, 1, 1000);

	if (status != HAL_OK)
	{
		AS5600_ERROR_HANDLE_CALLBACK(pAS, AS5600_I2C_COMM_ERROR);
		return AS5600_I2C_COMM_ERROR;
	}

	pAS->Status.MagnetTooStrong = 0x0001 & (temp[0] >> AS5600_MH_BITSHIFT);
	pAS->Status.MagnetTooWeak 	= 0x0001 & (temp[0] >> AS5600_ML_BITSHIFT);
	pAS->Status.MagnetDetected 	= 0x0001 & (temp[0] >> AS5600_MD_BITSHIFT);

	return AS5600_OK;
}







/**
  * @brief
  *
  * @param
  *
  * @retval
  */
eInfo AS5600_ReadRawAngle_I2C(AS5600Handle_Typedef *pAS)
{
	if(pAS == NULL)
	{
		AS5600_ERROR_HANDLE_CALLBACK(pAS, AS5600_NULL_POINTER);
		return AS5600_NULL_POINTER;
	}

	uint8_t temp[2];

	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Read(pAS->hi2c, (pAS->I2CAddress << 1), AS5600_REGISTER_RAW_ANGLE_HIGH, I2C_MEMADD_SIZE_8BIT, temp, 2, 1000);

	if (status != HAL_OK)
	{
		AS5600_ERROR_HANDLE_CALLBACK(pAS, AS5600_I2C_COMM_ERROR);
		return AS5600_I2C_COMM_ERROR;
	}

	pAS->RawAngle = 0x0FFF & ((temp[0] << 8) & temp[1]);

	return AS5600_OK;
}







/**
  * @brief
  *
  * @param
  *
  * @retval
  */
eInfo AS5600_ReadAngle_I2C(AS5600Handle_Typedef *pAS)
{
	if(pAS == NULL)
	{
		AS5600_ERROR_HANDLE_CALLBACK(pAS, AS5600_NULL_POINTER);
		return AS5600_NULL_POINTER;
	}

	uint8_t temp[2];

	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Read(pAS->hi2c, (pAS->I2CAddress << 1), AS5600_REGISTER_ANGLE_HIGH, I2C_MEMADD_SIZE_8BIT, temp, 2, 1000);

	if(status != HAL_OK)
	{
		AS5600_ERROR_HANDLE_CALLBACK(pAS, AS5600_I2C_COMM_ERROR);
		return AS5600_I2C_COMM_ERROR;
	}
	pAS->Angle = 0x0FFF & ((temp[0] << 8) | temp[1]);

	return AS5600_OK;
}





/**
  * @brief
  *
  * @param
  *
  * @retval
  */
eInfo AS5600_ReadAngle_PWM(AS5600Handle_Typedef *pAS)
{
	if(pAS == NULL)
	{
		AS5600_ERROR_HANDLE_CALLBACK(pAS, AS5600_NULL_POINTER);
		return AS5600_NULL_POINTER;
	}

	if(pAS->Config.OutputStage != PWM)
	{
		AS5600_ERROR_HANDLE_CALLBACK(pAS, AS5600_PWM_MODE_NOT_INITIALIZED);
		return AS5600_PWM_MODE_NOT_INITIALIZED;
	}

	uint32_t CCR1_Value;
	uint32_t CCR2_Value;

	CCR1_Value = HAL_TIM_ReadCapturedValue(pAS->htim, TIM_CHANNEL_1);
	CCR2_Value = HAL_TIM_ReadCapturedValue(pAS->htim, TIM_CHANNEL_2);

	if(CCR1_Value == 0)
	{
		AS5600_ERROR_HANDLE_CALLBACK(pAS, AS5600_DEVIDE_BY_ZERO);
		return AS5600_DEVIDE_BY_ZERO;
	}


	uint32_t PWMFreq;

	PWMFreq = pAS->TIMFreq / (CCR1_Value);
	switch(pAS->Config.PWMFrequency)
	{
	case _115Hz:
		if(PWMFreq > 130 || PWMFreq < 100)
		{
			AS5600_ERROR_HANDLE_CALLBACK(pAS, AS5600_INPUT_PWM_FREQ_ERROR);
			return AS5600_INPUT_PWM_FREQ_ERROR;
		}
		break;

	case _230Hz:
		if(PWMFreq > 260 || PWMFreq < 200)
		{
			AS5600_ERROR_HANDLE_CALLBACK(pAS, AS5600_INPUT_PWM_FREQ_ERROR);
			return AS5600_INPUT_PWM_FREQ_ERROR;
		}
		break;

	case _460Hz:
		if(PWMFreq > 510 || PWMFreq < 410)
		{
			AS5600_ERROR_HANDLE_CALLBACK(pAS, AS5600_INPUT_PWM_FREQ_ERROR);
			return AS5600_INPUT_PWM_FREQ_ERROR;
		}
		break;

	case _920Hz:
		if(PWMFreq > 1020 || PWMFreq < 820)
		{
			AS5600_ERROR_HANDLE_CALLBACK(pAS, AS5600_INPUT_PWM_FREQ_ERROR);
			return AS5600_INPUT_PWM_FREQ_ERROR;
		}
		break;
	}



	float DutyCycle;
	DutyCycle = ((float)CCR2_Value/(float)CCR1_Value) * 100.0f;

	pAS->LastAngle = pAS->Angle;
	pAS->Angle = MapDutycycle2Angle(DutyCycle, pAS->MinAngle, pAS->MaxAngle);

	//HAL_RCC_GetSysClockFreq();

	return AS5600_OK;

}













/**
  * @brief
  *
  * @param
  *
  * @retval
  */
eInfo AS5600_UpdateAbsolutePosition(AS5600Handle_Typedef *pAS)
{
	if(pAS == NULL)
	{
		AS5600_ERROR_HANDLE_CALLBACK(pAS, AS5600_NULL_POINTER);
		return AS5600_NULL_POINTER;
	}


	float LowLimit = pAS->MinAngle + 60.0f;
	float HighLimit = pAS->MaxAngle - 60.0f;

	switch(pAS->State)
	{
	case Wait:
		if(pAS->Angle > HighLimit)
		{
			pAS->State = WatchForOverflow;
		}
		else if(pAS->Angle < LowLimit)
		{
			pAS->State = WatchForUnderflow;
		}

		break;

	case WatchForOverflow:
		if(pAS->Angle < LowLimit)
		{
			pAS->FullRotationCounter ++;
			pAS->State = WatchForUnderflow;
		}
		else if(pAS->Angle > LowLimit && pAS->Angle < HighLimit)
		{
			pAS->State = Wait;
		}

		break;

	case WatchForUnderflow:
			if(pAS->Angle > HighLimit)
			{
				pAS->FullRotationCounter --;
				pAS->State = WatchForOverflow;
			}
			else if(pAS->Angle > LowLimit && pAS->Angle < HighLimit)
			{
				pAS->State = Wait;
			}

			break;
	}

	pAS->AbsolutePosition = (pAS->FullRotationCounter * pAS->MaxAngle) + pAS->Angle;



	return AS5600_OK;
}







/**
  * @brief
  *
  * @param
  *
  * @retval
  */











/**
  * @brief
  *
  * @param
  *
  * @retval
  */



















#endif /* INC_AS5600DRIVER_H_ */
