
/**
  ******************************************************************************
  * @file    mc_tasks.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implements tasks definition
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
//cstat -MISRAC2012-Rule-21.1
#include "main.h"
//cstat +MISRAC2012-Rule-21.1
#include "mc_type.h"
#include "mc_math.h"
#include "motorcontrol.h"
#include "regular_conversion_manager.h"
#include "mc_interface.h"
#include "digital_output.h"
#include "pwm_common.h"
#include "mc_tasks.h"
#include "parameters_conversion.h"
#include "mc_app_hooks.h"
#include "can.h"
#include "throttle.h"
#include "vehicle_directionctrl.h"
#include "motor_fault.h"
#include "mc_config_common.h"


/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private define */

/* Private define ------------------------------------------------------------*/
/* Un-Comment this macro define in order to activate the smooth
   braking action on over voltage */
/* #define  MC.SMOOTH_BRAKING_ACTION_ON_OVERVOLTAGE */

/* USER CODE END Private define */
#define VBUS_TEMP_ERR_MASK (MC_OVER_VOLT| MC_UNDER_VOLT| MC_CTRL_OVERTEMP_FAULT |  \
							MC_THROTTLE_FAULT_HIGH | MC_THROTTLE_FAULT_LOW | MC_MOTORSTUCK_FAULT)

/* Private variables----------------------------------------------------------*/

static uint16_t hMFTaskCounterM1 = 0; //cstat !MISRAC2012-Rule-8.9_a
static volatile uint16_t hBootCapDelayCounterM1 = ((uint16_t)0);
static volatile uint16_t hStopPermanencyCounterM1 = ((uint16_t)0);

static volatile uint8_t bMCBootCompleted = ((uint8_t)0);

#define M1_CHARGE_BOOT_CAP_TICKS          (((uint16_t)SYS_TICK_FREQUENCY * (uint16_t)10) / 1000U)
#define M1_CHARGE_BOOT_CAP_DUTY_CYCLES ((uint32_t)0.000\
                                      * ((uint32_t)PWM_PERIOD_CYCLES / 2U))
#define M2_CHARGE_BOOT_CAP_TICKS         (((uint16_t)SYS_TICK_FREQUENCY * (uint16_t)10) / 1000U)
#define M2_CHARGE_BOOT_CAP_DUTY_CYCLES ((uint32_t)0\
                                      * ((uint32_t)PWM_PERIOD_CYCLES2 / 2U))

/* USER CODE BEGIN Private Variables */
/* USER CODE END Private Variables */

/* Private functions ---------------------------------------------------------*/
void TSK_MediumFrequencyTaskM1(void);
void TSK_MF_StopProcessing(uint8_t motor);
MCI_Handle_t *GetMCI(uint8_t bMotor);
void TSK_SafetyTask_PWMOFF(uint8_t motor);
void SetOverCurrentFault(void);

/* USER CODE BEGIN Private Functions */

/* USER CODE END Private Functions */
/**
  * @brief  It initializes the whole MC core according to user defined
  *         parameters.
  * @param  pMCIList pointer to the vector of MCInterface objects that will be
  *         created and initialized. The vector must have length equal to the
  *         number of motor drives.
  * @retval None
  */
__weak void MCboot( MCI_Handle_t *pMCIList[NBR_OF_MOTORS] )
{
  /* USER CODE BEGIN MCboot 0 */

  /* USER CODE END MCboot 0 */

  if (MC_NULL == pMCIList)
  {
    /* Nothing to do */
  }
  else
  {

    bMCBootCompleted = (uint8_t )0;

    /*************************************************/
    /*    FOC initialization         */
    /*************************************************/
    pMCIList[M1] = &Mci[M1];
    FOC_Init();

    /* USER CODE BEGIN MCboot 1 */

    /* USER CODE END MCboot 1 */

    /******************************************************/
    /*   PID component initialization: speed regulation   */
    /******************************************************/
    PID_HandleInit(&PIDSpeedHandle_M1);

    /********************************************************/
    /*   Bus voltage sensor component initialization        */
    /********************************************************/
    (void)RCM_RegisterRegConv(&VbusRegConv_M1);
    RVBS_Init(&BusVoltageSensor_M1);

    /*******************************************************/
    /*   Temperature measurement component initialization  */
    /*******************************************************/
    (void)RCM_RegisterRegConv(&TempRegConv_M1);
    NTC_Init(&TempSensor_M1);

    /*******************************************************/
    /*  Throttle initialization  */
    /*******************************************************/
     (void)RCM_RegisterRegConv(&ThrottleRegConv_M1);
 	Throttle_Init(&ThrottleSensor_M1);

    /*******************************************************/
    /*  Control Temp initialization  */
    /*******************************************************/
     (void)RCM_RegisterRegConv(&CtsTempRegConv_M1);
     CTS_Init(&ControlTempSensor_M1);


    /* Applicative hook in MCBoot() */
    MC_APP_BootHook();

    /**************************************/
    /*    Start timers synchronously      */
    /**************************************/
    startTimers();

    /* USER CODE BEGIN MCboot 2 */

    /* USER CODE END MCboot 2 */

    bMCBootCompleted = 1U;
  }
}

/**
 * @brief Runs all the Tasks of the Motor Control cockpit
 *
 * This function is to be called periodically at least at the Medium Frequency task
 * rate (It is typically called on the Systick interrupt). Exact invokation rate is
 * the Speed regulator execution rate set in the Motor Contorl Workbench.
 *
 * The following tasks are executed in this order:
 *
 * - Medium Frequency Tasks of each motors
 * - Safety Task
 * - Power Factor Correction Task (if enabled)
 * - User Interface task.
 */
__weak void MC_RunMotorControlTasks(void)
{
  if (0U == bMCBootCompleted)
  {
    /* Nothing to do */
  }
  else
  {
	    /* ** Medium Frequency Tasks ** */
	/* USER CODE BEGIN MC_Scheduler 0 */

	/* USER CODE END MC_Scheduler 0 */

	    if(hMFTaskCounterM1 > 0u)
	    {
	      hMFTaskCounterM1--;
	    }
	    else
	    {

	      TSK_MediumFrequencyTaskM1();

	      /* Applicative hook at end of Medium Frequency for Motor 1 */
	      MC_APP_PostMediumFrequencyHook_M1();

	      /* USER CODE BEGIN MC_Scheduler 1 */

	      /* USER CODE END MC_Scheduler 1 */

	      hMFTaskCounterM1 = (uint16_t)MF_TASK_OCCURENCE_TICKS;
	    }
	    if(hBootCapDelayCounterM1 > 0U)
	    {
	      hBootCapDelayCounterM1--;
	    }
	    else
	    {
	      /* Nothing to do */
	    }
	    if(hStopPermanencyCounterM1 > 0U)
	    {
	      hStopPermanencyCounterM1--;
	    }
	    else
	    {
	      /* Nothing to do */
	    }
	  /* USER CODE BEGIN MC_Scheduler 2 */

	  /* USER CODE END MC_Scheduler 2 */

	    /* Safety task is run after Medium Frequency task so that
	     * it can overcome actions they initiated if needed */
	    TSK_SafetyTask();
	  }
}

/**
  * @brief  It set a counter intended to be used for counting the delay required
  *         for drivers boot capacitors charging of motor 1
  * @param  hTickCount number of ticks to be counted
  * @retval void
  */
__weak void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount)
{
   hBootCapDelayCounterM1 = hTickCount;
}

/**
  * @brief  Use this function to know whether the time required to charge boot
  *         capacitors of motor 1 has elapsed
  * @param  none
  * @retval bool true if time has elapsed, false otherwise
  */
__weak bool TSK_ChargeBootCapDelayHasElapsedM1(void)
{
  bool retVal = false;
  if (((uint16_t)0) == hBootCapDelayCounterM1)
  {
    retVal = true;
  }
  return (retVal);
}

/**
  * @brief  It set a counter intended to be used for counting the permanency
  *         time in STOP state of motor 1
  * @param  hTickCount number of ticks to be counted
  * @retval void
  */
__weak void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount)
{
  hStopPermanencyCounterM1 = hTickCount;
}

/**
  * @brief  Use this function to know whether the permanency time in STOP state
  *         of motor 1 has elapsed
  * @param  none
  * @retval bool true if time is elapsed, false otherwise
  */
__weak bool TSK_StopPermanencyTimeHasElapsedM1(void)
{
  bool retVal = false;
  if (((uint16_t)0) == hStopPermanencyCounterM1)
  {
    retVal = true;
  }
  return (retVal);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif

/**
  * @brief  Executes the Motor Control duties that require a high frequency rate and a precise timing
  *
  *  This is mainly the FOC current control loop. It is executed depending on the state of the Motor Control
  * subsystem (see the state machine(s)).
  *
  * @retval Number of the  motor instance which FOC loop was executed.
  */

__weak uint8_t TSK_HighFrequencyTask(void)
{
  uint8_t bMotorNbr;
  bMotorNbr = 0;

  /* USER CODE BEGIN HighFrequencyTask 0 */

  /* USER CODE END HighFrequencyTask 0 */
  FOC_HighFrequencyTask(bMotorNbr);

  /* USER CODE BEGIN HighFrequencyTask 1 */

  /* USER CODE END HighFrequencyTask 1 */

  return (bMotorNbr);

}

/**
  * @brief  Executes safety checks (e.g. bus voltage and temperature) for all drive instances.
  *
  * Faults flags are updated here.
  */
__weak void TSK_SafetyTask(void)
{
  /* USER CODE BEGIN TSK_SafetyTask 0 */

  /* USER CODE END TSK_SafetyTask 0 */
  if (1U == bMCBootCompleted)
  {
    TSK_SafetyTask_PWMOFF(M1);
  /* USER CODE BEGIN TSK_SafetyTask 1 */

  /* USER CODE END TSK_SafetyTask 1 */
  }
  else
  {
    /* Nothing to do */
  }
}

/**
  * @brief  Safety task implementation if  MC.ON_OVER_VOLTAGE == TURN_OFF_PWM
  * @param  bMotor Motor reference number defined
  *         \link Motors_reference_number here \endlink
  * @retval None
  */

__weak void TSK_SafetyTask_PWMOFF(uint8_t bMotor)
{
  /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 0 */

  /* USER CODE END TSK_SafetyTask_PWMOFF 0 */
  uint32_t CodeReturn = MC_NO_ERROR;
  uint8_t lbmotor = M1;
  const uint16_t errMask[NBR_OF_MOTORS] = {VBUS_TEMP_ERR_MASK};

  if(M1 == bMotor)
  {
	  uint16_t rawValueM1 = RCM_GetRegularConv(&TempRegConv_M1);
	  CodeReturn |= errMask[bMotor] & NTC_CalcAvTemp(&TempSensor_M1, rawValueM1);
  }
  else
  {
    /* Nothing to do */
  }
  /* Due to warning array subscript 1 is above array bounds of PWMC_Handle_t *[1] [-Warray-bounds] */

  CodeReturn |= PWMC_IsFaultOccurred(pwmcHandle[lbmotor]);     /* check for fault. It return MC_OVER_CURR or MC_NO_FAULTS
                                                 //      (for STM32F30x can return MC_OVER_VOLT in case of HW Overvoltage) */
  if (M1 == bMotor)
   {

     uint16_t rawValueM1 =  RCM_GetRegularConv(&VbusRegConv_M1);
     CodeReturn |= errMask[bMotor] & RVBS_CalcAvVbus(&BusVoltageSensor_M1, rawValueM1);
   }
   else
   {
     /* Nothing to do */
   }
   if (M1 == bMotor)
   {
       uint16_t rawValueM1 =  RCM_GetRegularConv(&ThrottleRegConv_M1);
       CodeReturn |= errMask[bMotor] & Throttle_CalcAvPercentage(&ThrottleSensor_M1, rawValueM1);
   }
   else
   {
       /* Nothing to do */
   }

   MCI_FaultProcessing(&Mci[bMotor], CodeReturn, ~CodeReturn); /* Process faults */

  if (MCI_GetFaultState(&Mci[bMotor]) != (uint32_t)MC_NO_FAULTS)
  {
    PWMC_SwitchOffPWM(pwmcHandle[bMotor]);
    FOC_Clear(bMotor);

    /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 1 */

    /* USER CODE END TSK_SafetyTask_PWMOFF 1 */
  }
  else
  {
    /* no errors */
  }

  /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 3 */

  /* USER CODE END TSK_SafetyTask_PWMOFF 3 */
}

/**
  * @brief  Puts the Motor Control subsystem in in safety conditions on a Hard Fault
  *
  *  This function is to be executed when a general hardware failure has been detected
  * by the microcontroller and is used to put the system in safety condition.
  */
__weak void TSK_HardwareFaultTask(void)
{
  /* USER CODE BEGIN TSK_HardwareFaultTask 0 */

  /* USER CODE END TSK_HardwareFaultTask 0 */
   FOC_Clear(M1);
  MCI_FaultProcessing(&Mci[M1], MC_SW_ERROR, 0);

  /* USER CODE BEGIN TSK_HardwareFaultTask 1 */

  /* USER CODE END TSK_HardwareFaultTask 1 */
}

 /**
  * @brief  Locks GPIO pins used for Motor Control to prevent accidental reconfiguration
  */

__weak void mc_lock_pins (void)
{
	LL_GPIO_LockPin(CURRENT_SHUNT_W_N_GPIO_Port, CURRENT_SHUNT_W_N_Pin);
	LL_GPIO_LockPin(CURRENT_SHUNT_W_OUT_GPIO_Port, CURRENT_SHUNT_W_OUT_Pin);
	LL_GPIO_LockPin(Controller_temp_sense_GPIO_Port, Controller_temp_sense_Pin);
	LL_GPIO_LockPin(Vbatt_measure_GPIO_Port, Vbatt_measure_Pin);
	LL_GPIO_LockPin(CURRENT_SHUNT_W_P_GPIO_Port, CURRENT_SHUNT_W_P_Pin);
	LL_GPIO_LockPin(CURRENT_SHUNT_V_P_GPIO_Port, CURRENT_SHUNT_V_P_Pin);
	LL_GPIO_LockPin(CURRENT_SHUNT_U_GPIO_Port, CURRENT_SHUNT_U_Pin);
	LL_GPIO_LockPin(CURRENT_SHUNT_U_N_GPIO_Port, CURRENT_SHUNT_U_N_Pin);
	LL_GPIO_LockPin(CURRENT_SHUNT_U_OUT_GPIO_Port, CURRENT_SHUNT_U_OUT_Pin);
	LL_GPIO_LockPin(CURRENT_SHUNT_V_OUT_GPIO_Port, CURRENT_SHUNT_V_OUT_Pin);
	LL_GPIO_LockPin(CURRENT_SHUNT_V_N_GPIO_Port, CURRENT_SHUNT_V_N_Pin);
	LL_GPIO_LockPin(HALL_A_GPIO_Port, HALL_A_Pin);
	LL_GPIO_LockPin(HALL_B_GPIO_Port, HALL_B_Pin);
	LL_GPIO_LockPin(HALL_C_GPIO_Port, HALL_C_Pin);
	LL_GPIO_LockPin(PWM_H_U_GPIO_Port, PWM_H_U_Pin);
	LL_GPIO_LockPin(PWM_H_V_GPIO_Port, PWM_H_V_Pin);
	LL_GPIO_LockPin(PWM_H_W_GPIO_Port, PWM_H_W_Pin);
	LL_GPIO_LockPin(PWM_L_V_GPIO_Port, PWM_L_V_Pin);
	LL_GPIO_LockPin(PWM_L_W_GPIO_Port, PWM_L_W_Pin);
	LL_GPIO_LockPin(PWM_L_U_GPIO_Port, PWM_L_U_Pin);
}


/* USER CODE BEGIN mc_task 0 */

/* USER CODE END mc_task 0 */

/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
