/*
 * ControlTemp_Sensor.c
 *
 *  Created on: Mar 24, 2023
 *      Author: sujatha
 */

#include "ControlTemp_Sensor.h"
/* Private function prototypes -----------------------------------------------*/

static uint32_t CTS_SetFaultState(CTS_Handle_t *pHandle);

uint32_t CTS_SetFaultState(CTS_Handle_t *pHandle)
{
  uint16_t hFault;
#ifdef NULL_PTR_CHECK_CTS_TEMP_SENS
  if (MC_NULL == pHandle)
  {
	  hFault = MC_SW_ERROR;
  }
  else
  {
#endif
    if (pHandle->ctsAvTemp_d > pHandle->ctsOverTempThreshold)
    {
    	hFault = MC_CTRL_OVERTEMP_FAULT;
    }
    else if (pHandle->ctsAvTemp_d < pHandle->ctsOverTempDeactThreshold)
    {
    	hFault = MC_NO_ERROR;
    }
    else
    {
    	hFault = pHandle->ctsFaultState;
    }
#ifdef NULL_PTR_CHECK_CTS_TEMP_SENS
  }
#endif
  return (hFault);
}

/* Functions ---------------------------------------------------- */

/**
  * @brief Initializes temperature sensing conversions
  *
  * @param pHandle : Pointer on Handle structure of TemperatureSensor component
  *
  */
void CTS_Init(CTS_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_CTS_TEMP_SENS
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    if (REAL_SENSOR == pHandle->cSensorType)
    {

    }

#ifdef NULL_PTR_CHECK_CTS_TEMP_SENS
  }
#endif
}


/**
  * @brief Performs the temperature sensing average computation after an ADC conversion
  *
  * @param pHandle : Pointer on Handle structure of TemperatureSensor component
  *
  * @retval Fault status : Error reported in case of an over temperature detection
  */

__weak uint16_t CTS_CalcAvTemp(CTS_Handle_t *pHandle, uint16_t rawValue)
{
  uint16_t returnValue;
#ifdef NULL_PTR_CHECK_CTS_TEMP_SENS
  if (MC_NULL == pHandle)
  {
    returnValue = 0U;
  }
  else
  {
#endif
    if (REAL_SENSOR == pHandle->cSensorType)
    {
      uint16_t hAux=rawValue;

      if (0xFFFFU == hAux)
      {
        /* Nothing to do */
      }
      else
      {
        pHandle->ctsAvTemp_d += (hAux - pHandle->ctsAvTemp_d) >> 8U;
      }
      pHandle->ctsFaultState = CTS_SetFaultState(pHandle);
    }
    else  /* case VIRTUAL_SENSOR */
    {
      pHandle->ctsFaultState = MC_NO_ERROR;
    }
    returnValue = pHandle->ctsFaultState;
#ifdef NULL_PTR_CHECK_CTS_TEMP_SENS
  }
#endif
  return (returnValue);
}

/**
  * @brief  Returns latest averaged temperature expressed in Celsius degrees
  *
  * @param pHandle : Pointer on Handle structure of TemperatureSensor component
  *
  * @retval AverageTemperature : Latest averaged temperature measured (in Celsius degrees)
  */

__weak uint16_t CTS_GetAvTemp_C(CTS_Handle_t *pHandle)
{
  int16_t returnValue;
#ifdef NULL_PTR_CHECK_CTS_TEMP_SENS
  if (MC_NULL == pHandle)
  {
    returnValue = 0;
  }
  else
  {
#endif
    int32_t wTemp = 0;

    if (REAL_SENSOR == pHandle->cSensorType)
    {
      wTemp = (int32_t)pHandle->ctsAvTemp_d;
      wTemp -= (int32_t)pHandle->wV0;
      wTemp *= (int32_t)pHandle->ctsSensitivity;

#ifndef FULL_MISRA_C_COMPLIANCY_CTS_TEMP
      wTemp = (wTemp >> 16) + (int32_t)pHandle->hT0;
#else
      wTemp = (wTemp / 65536) + (int32_t)pHandle->hT0;
#endif
    }
    else
    {
      wTemp = (int32_t)pHandle->ctsExpectedTemp_C;
    }

    returnValue = (int16_t)wTemp;
#ifdef NULL_PTR_CHECK_CTS_TEMP_SENS
  }
#endif
  return (returnValue);
}


/**
  * @}
  */

/************************ (C) COPYRIGHT 2025 STMicroelectronics *****END OF FILE****/
