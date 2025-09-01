
/**
  ******************************************************************************
  * @file    mc_config_common.c
  * @author  Motor Control SDK Team,ST Microelectronics
  * @brief   Motor Control Subsystem components configuration and handler structures.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044,the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

//cstat -MISRAC2012-Rule-21.1
#include "main.h" //cstat !MISRAC2012-Rule-21.1
//cstat +MISRAC2012-Rule-21.1
#include "mc_type.h"
#include "parameters_conversion.h"
#include "mc_parameters.h"
#include "mc_config_common.h"

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */

/* USER CODE BEGIN Additional define */

/* USER CODE END Additional define */

/**
  * temperature sensor parameters Motor 1.
  */
RegConv_t TempRegConv_M1 =
{
  .regADC                = ADC1,
  .channel               = MC_ADC_CHANNEL_5,
  .samplingTime          = M1_TEMP_SAMPLING_TIME,
  .data                  = 0
};

NTC_Handle_t TempSensor_M1 =
{
  .bSensorType             = REAL_SENSOR,
  .hOverTempThreshold      = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d),
  .hOverTempDeactThreshold = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d - OV_TEMPERATURE_HYSTERESIS_d),
  .hSensitivity            = (int16_t)(ADC_REFERENCE_VOLTAGE/dV_dT),
  .wV0                     = (uint16_t)((V0_V * 65536) / ADC_REFERENCE_VOLTAGE),
  .hT0                     = T0_C,
};

/**
  * Bus voltage sensor parameters Motor 1.
  */
RegConv_t VbusRegConv_M1 =
{
  .regADC                   = ADC1,
  .channel                  = MC_ADC_CHANNEL_1,
  .samplingTime             = M1_VBUS_SAMPLING_TIME,
  .data                     = 1 + (uint16_t)((NOMINAL_BUS_VOLTAGE_V * 65536) / (ADC_REFERENCE_VOLTAGE / VBUS_PARTITIONING_FACTOR))
};

RDivider_Handle_t BusVoltageSensor_M1 =
{
  ._Super =
  {
    .SensorType               = REAL_SENSOR,
    .ConversionFactor         = (uint16_t)(ADC_REFERENCE_VOLTAGE / VBUS_PARTITIONING_FACTOR),
  },

  .OverVoltageThreshold       = OVERVOLTAGE_THRESHOLD_d,
  .OverVoltageThresholdLow    = OVERVOLTAGE_THRESHOLD_d,
  .OverVoltageHysteresisUpDir = true,
  .UnderVoltageThreshold      =  UNDERVOLTAGE_THRESHOLD_d,
};

/**
  * Throttle sensor parameters Motor 1
  */

RegConv_t ThrottleRegConv_M1 =
{
  .regADC = ADC2,
  .channel = MC_ADC_CHANNEL_5,
  .samplingTime = M1_THROTTLE_SAMPLING_TIME,
  .data = 0
};

Throttle_Handle_t ThrottleSensor_M1 =
{
  .SensorType               = REAL_SENSOR,
  .ThrottleOvervoltageThreshold = (uint16_t)THROTTLE_OVERERVOLTAGE_d,
  .ThrottleUndervoltageThreshold = (uint16_t)THROTTLE_UNDERVOLTAGE_d,
};

/**
  * Control Temperature sensor parameters Motor 1
 */
RegConv_t CtsTempRegConv_M1 =
{
.regADC = ADC2,
.channel = MC_ADC_CHANNEL_11,
.samplingTime = M1_CONTROLTEMP_SAMPLING_TIME,
};

CTS_Handle_t ControlTempSensor_M1 =
{
  .cSensorType = REAL_SENSOR,
  .ctsOverTempThreshold      = (uint16_t)(OV_CTRLTEMP_THRESHOLD_d),
  .ctsOverTempDeactThreshold = (uint16_t)(OV_CTRLTEMP_THRESHOLD_d - OV_CTRLTEMP_HYSTERESIS_d),
  .ctsSensitivity            = (int16_t)(ADC_REFERENCE_VOLTAGE/dV_dT),
  .wV0                     = (uint16_t)(V0_V *65536/ ADC_REFERENCE_VOLTAGE),
  .hT0                     = T0_C,
};

PWMC_Handle_t *pwmcHandle[NBR_OF_MOTORS];

/* USER CODE BEGIN Additional configuration */

/* USER CODE END Additional configuration */

/******************* (C) COPYRIGHT 2025 STMicroelectronics *****END OF FILE****/

