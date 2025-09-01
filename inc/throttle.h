/*
 * Throttle.h
 *
 *  Created on: 20-Mar-2023
 *
 */

#ifndef INC_THROTTLE_H_
#define INC_THROTTLE_H_

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "regular_conversion_manager.h"

typedef struct
{
  SensorType_t SensorType;    /**< Type of instanced temperature.
                                    This parameter can be REAL_SENSOR or VIRTUAL_SENSOR */

  
  uint16_t Throttle_d;          /**< It contains latest available average throttle voltage.
                                    This parameter is expressed in u16Volts */

  

  uint16_t FaultState;        /**< Contains latest Fault code.
                                    This parameter is set to MC_OVER_TEMP or MC_NO_ERROR */

 

  uint16_t ThrottleOvervoltageThreshold; /**< Throttle threshold below which an active over throttle voltage fault is cleared.
                                         This parameter is expressed in u16Celsius through formula:
                                         hOverTempDeactThreshold =
                                         (V0[V]+dV/dT[V/°C]*(OverTempDeactThresh[°C] - T0[°C]))* 65536 /
                                         MCU supply voltage*/

  uint16_t ThrottleUndervoltageThreshold; /**< Throttle threshold above which an active under throttle voltage fault is cleared.
                                         This parameter is expressed in u16Celsius through formula:
                                         hOverTempDeactThreshold =
                                         (V0[V]+dV/dT[V/°C]*(OverTempDeactThresh[°C] - T0[°C]))* 65536 /
                                         MCU supply voltage*/
 
  


} Throttle_Handle_t;

/* Initialize throttle voltage sensing parameters */
void Throttle_Init(Throttle_Handle_t *pHandle);

/* Clear static average throttle value */
void Throttle_Clear(Throttle_Handle_t *pHandle);

/* Throttle voltage sensing computation */
uint32_t Throttle_CalcAvPercentage(Throttle_Handle_t *pHandle, uint16_t rawValue);

/* Get averaged throttle measurement expressed in u8Percentage */
uint8_t Throttle_GetAvPercentage_d(const Throttle_Handle_t *pHandle);

uint16_t Throttle_GetVoltage_d(const Throttle_Handle_t *pHandle);

#ifdef __cplusplus
}
#endif /* __cpluplus */


#endif /* INC_THROTTLE_H_ */
