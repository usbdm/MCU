/*
 * Throttle.c
 *
 *  Created on: 13-Jun-2023
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "throttle.h"
static uint32_t throttle_startup_delay_counter = 0;
#define THROTTLE_STARTUP_IGNORE_COUNT  1000  // or time-based delay
/* Private function prototypes -----------------------------------------------*/
static uint32_t Throttle_SetFaultState(Throttle_Handle_t *pHandle);

/* Private functions ---------------------------------------------------------*/
uint32_t Throttle_SetFaultState(Throttle_Handle_t *pHandle)
{
  uint32_t hFault = MC_NO_ERROR ;
#ifdef NULL_PTR_CHECK_Throttle_SENS
  if (MC_NULL == pHandle)
  {
    hFault = MC_SW_ERROR;
  }
  else
  {
#endif

   if(throttle_startup_delay_counter < THROTTLE_STARTUP_IGNORE_COUNT)
   {
	 throttle_startup_delay_counter++;
	 return MC_NO_ERROR;  // Ignore faults during power-up
   }
    if(pHandle->Throttle_d > pHandle->ThrottleOvervoltageThreshold)
    {
      hFault = MC_THROTTLE_FAULT_HIGH ;
    }
    else if(pHandle->Throttle_d < pHandle->ThrottleUndervoltageThreshold)
    {
     hFault = MC_THROTTLE_FAULT_LOW;
    }
    else
    {
      hFault = MC_NO_ERROR; 
    }
  return hFault;
}

/* Functions ---------------------------------------------------- */

__weak void Throttle_Init(Throttle_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_Throttle_SENS
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    if (REAL_SENSOR == pHandle->SensorType)
    {
      /* Need to be register with RegularConvManager */

    }

#ifdef NULL_PTR_CHECK_Throttle_SENS
  }
#endif
}

__weak uint32_t Throttle_CalcAvPercentage(Throttle_Handle_t *pHandle, uint16_t rawValue)
{
  uint32_t returnValue;
#ifdef NULL_PTR_CHECK_Throttle_SENS
  if (MC_NULL == pHandle)
  {
    returnValue = 0U;
  }
  else
  {
#endif
    if (REAL_SENSOR == pHandle->SensorType)
    {
      uint16_t hAux = rawValue;
      if (0xFFFFU == hAux)
      {
        /* Nothing to do */
      }
      else
      {
         pHandle->Throttle_d += (hAux - pHandle->Throttle_d) >> 8U;
      }

      pHandle->FaultState = Throttle_SetFaultState(pHandle);
    }
    returnValue = pHandle->FaultState;
#ifdef NULL_PTR_CHECK_Throttle_SENS
  }
#endif
  return (returnValue);
}

__weak uint8_t Throttle_GetAvPercentage_d(const Throttle_Handle_t *pHandle)
{
 uint8_t l_throttleposition_u8 = 0;
#ifdef NULL_PTR_CHECK_Throttle_SENS
 if (MC_NULL == pHandle)
  {
    return 0U;
  }
#endif

  if(pHandle->Throttle_d < pHandle->ThrottleUndervoltageThreshold)
  {
	 return 0U;
  }
  else if(pHandle->Throttle_d > pHandle->ThrottleOvervoltageThreshold)
  {
  	 return 100U;
  }
  else
  {
    if(pHandle->ThrottleOvervoltageThreshold != pHandle-> ThrottleUndervoltageThreshold)
    {
    	l_throttleposition_u8 = (uint8_t)(((pHandle->Throttle_d-pHandle->ThrottleUndervoltageThreshold)*100)/   \
		                (pHandle->ThrottleOvervoltageThreshold - pHandle->ThrottleUndervoltageThreshold));

         return l_throttleposition_u8;
    }
    else
    {
    	return 0xFFU;   // for the wrong threshold configuration
    }
  }
}

__weak uint16_t Throttle_GetVoltage_d(const Throttle_Handle_t *pHandle)
{
 return (pHandle->Throttle_d);
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/

