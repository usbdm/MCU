/*
 * ControlTemp_ControlTemp_Sensor.h
 *
 *  Created on: Mar 24, 2023
 *      Author: sujatha
 */

#ifndef CONTROLTEMP_SENSOR_H_
#define CONTROLTEMP_SENSOR_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */


typedef struct
{
  SensorType_t cSensorType;    /**< Type of instanced temperature.
                                    This parameter can be REAL_SENSOR or VIRTUAL_SENSOR */

  
  uint16_t ctsAvTemp_d;          /**< It contains latest available average Vbus.
                                    This parameter is expressed in u16Celsius */

  uint16_t ctsExpectedTemp_d;    /**< Default set when no sensor available (ie virtual sensor) */

  uint16_t ctsExpectedTemp_C;    /**< Default value when no sensor available (ie virtual sensor).
                                    This parameter is expressed in Celsius */

  uint16_t ctsFaultState;        /**< Contains latest Fault code.
                                    This parameter is set to MC_OVER_TEMP or MC_NO_ERROR */

 
  uint16_t ctsOverTempThreshold; /**< Represents the over voltage protection intervention threshold.
                                    This parameter is expressed in u16Celsius through formula:
                                    hOverTempThreshold =
                                    (V0[V]+dV/dT[V/°C]*(OverTempThreshold[°C] - T0[°C]))* 65536 /
                                    MCU supply voltage */
  uint16_t ctsOverTempDeactThreshold; /**< Temperature threshold below which an active over temperature fault is cleared.
                                         This parameter is expressed in u16Celsius through formula:
                                         hOverTempDeactThreshold =
                                         (V0[V]+dV/dT[V/°C]*(OverTempDeactThresh[°C] - T0[°C]))* 65536 /
                                         MCU supply voltage*/
  int16_t ctsSensitivity;        /**< NTC sensitivity
                                    This parameter is equal to MCU supply voltage [V] / dV/dT [V/°C] */
  uint32_t wV0;                /**< V0 voltage constant value used to convert the temperature into Volts.
                                    This parameter is equal V0*65536/MCU supply
                                    Used in through formula: V[V]=V0+dV/dT[V/°C]*(T-T0)[°C] */
  uint16_t hT0;                /**< T0 temperature constant value used to convert the temperature into Volts
                                    Used in through formula: V[V]=V0+dV/dT[V/°C]*(T-T0)[°C] */
  uint8_t convHandle;          /*!< handle to the regular conversion */

} CTS_Handle_t;

/* Initialize temperature sensing parameters */
void CTS_Init(CTS_Handle_t *pHandle);

/* Get averaged temperature measurement expressed in u16Celsius */
uint16_t CTS_GetAvTemp(CTS_Handle_t *pHandle);

/* Get averaged temperature measurement expressed in Celsius degrees */
uint16_t CTS_GetAvTemp_C(CTS_Handle_t *pHandle);

static inline uint16_t CTS_GetAvTemp_d(CTS_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_CTS_TEMP_SENS
  return ((MC_NULL == pHandle) ? 0U : pHandle->ctsAvTemp_d);
#else
  return (pHandle->ctsAvTemp_d);
#endif
}

/* Get the temperature measurement fault status */
static inline uint16_t CTS_CheckTemp(const CTS_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_CTS_TEMP_SENS
  return ((MC_NULL == pHandle) ? 0U : pHandle->ctsFaultState);
#else
  return (pHandle->ctsFaultState);
#endif
}

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* INC_CONTROLTEMP_SENSOR_H_ */
