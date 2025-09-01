
/**
  ******************************************************************************
  * @file    drive_parameters.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the parameters needed for the Motor Control SDK
  *          in order to configure a motor drive.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DRIVE_PARAMETERS_H
#define DRIVE_PARAMETERS_H

/************************
 *** Motor Parameters ***
 ************************/

/******** MAIN AND AUXILIARY SPEED/POSITION SENSOR(S) SETTINGS SECTION ********/

/*** Speed measurement settings ***/
#define MAX_APPLICATION_SPEED_RPM       450 /*!< rpm, mechanical */
#define MIN_APPLICATION_SPEED_RPM       0 /*!< rpm, mechanical,
                                                           absolute value */
#define M1_SS_MEAS_ERRORS_BEFORE_FAULTS 3 /*!< Number of speed
                                                             measurement errors before
                                                             main sensor goes in fault */
/****** Hall sensors ************/

#define HALL_AVERAGING_FIFO_DEPTH        16 /*!< depth of the FIFO used to
                                                           average mechanical speed in
                                                           0.1Hz resolution */
#define HALL_MTPA  false

/* USER CODE BEGIN angle reconstruction M1 */
#define REV_PARK_ANGLE_COMPENSATION_FACTOR 0
/* USER CODE END angle reconstruction M1 */

/**************************    DRIVE SETTINGS SECTION   **********************/
/* PWM generation and current reading */

#define PWM_FREQUENCY   15000
#define PWM_FREQ_SCALING 1

#define LOW_SIDE_SIGNALS_ENABLING        LS_PWM_TIMER
#define SW_DEADTIME_NS                   600 /*!< Dead-time to be inserted
                                                           by FW, only if low side
                                                           signals are enabled */

/* Torque and flux regulation loops */
#define REGULATION_EXECUTION_RATE     1    /*!< FOC execution rate in
                                                           number of PWM cycles */

 /* Gains values for torque and flux control loops */
#define PID_TORQUE_KP_DEFAULT               400
#define PID_TORQUE_KI_DEFAULT               300
#define PID_TORQUE_KD_DEFAULT               100
#define PID_FLUX_KP_DEFAULT                 200
#define PID_FLUX_KI_DEFAULT                 300
#define PID_FLUX_KD_DEFAULT                 100

/* Torque/Flux control loop gains dividers*/
#define TF_KPDIV                            128
#define TF_KIDIV                            256
#define TF_KDDIV                            256
#define TF_KPDIV_LOG                        LOG2((128))
#define TF_KIDIV_LOG                        LOG2((256))
#define TF_KDDIV_LOG                        LOG2((256))


#define TFDIFFERENTIAL_TERM_ENABLING  DISABLE

/* Speed control loop */
#define SPEED_LOOP_FREQUENCY_HZ       (uint16_t )1000 /*!<Execution rate of speed
                                                      regulation loop (Hz) */

#define PID_SPEED_KP_DEFAULT          4000/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KI_DEFAULT          352/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KD_DEFAULT          0/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
/* Speed PID parameter dividers */
#define SP_KPDIV                      16
#define SP_KIDIV                      16384
#define SP_KDDIV                      16
#define SP_KPDIV_LOG                  LOG2((16))
#define SP_KIDIV_LOG                  LOG2((16384))
#define SP_KDDIV_LOG                  LOG2((16))

/* USER CODE BEGIN PID_SPEED_INTEGRAL_INIT_DIV */
#define PID_SPEED_INTEGRAL_INIT_DIV 1 /*  */
/* USER CODE END PID_SPEED_INTEGRAL_INIT_DIV */

#define SPD_DIFFERENTIAL_TERM_ENABLING DISABLE
#define IQMAX_A							110

/* Default settings */
#define DEFAULT_CONTROL_MODE           MCM_TORQUE_MODE
#define DEFAULT_TARGET_SPEED_RPM       450
#define DEFAULT_TARGET_SPEED_UNIT      (DEFAULT_TARGET_SPEED_RPM*SPEED_UNIT/U_RPM)
#define DEFAULT_TORQUE_COMPONENT_A       2
#define DEFAULT_FLUX_COMPONENT_A         0

/**************************    FIRMWARE PROTECTIONS SECTION   *****************/
#define OV_VOLTAGE_THRESHOLD_V          (65u)   /*!< Over-voltage
                                                         threshold */
#define UD_VOLTAGE_THRESHOLD_V          (48u) /*!< Under-voltage
                                                          threshold */

#define THROTTLE_UNDERVOLTAGE_V         0.3      /* Under voltage limit for throttle */

#define THROTTLE_OVERVOLTAGE_V          2.2      /* Over voltage limit for throttle*/
#ifdef NOT_IMPLEMENTED

#define ON_OVER_VOLTAGE                 TURN_OFF_PWM /*!< TURN_OFF_PWM,
                                                         TURN_ON_R_BRAKE or
                                                         TURN_ON_LOW_SIDES */
#endif /* NOT_IMPLEMENTED */

#define OV_TEMPERATURE_THRESHOLD_C      150 /*!< Celsius degrees */
#define OV_TEMPERATURE_HYSTERESIS_C     80 /*!< Celsius degrees */

#define OV_CTRLTEMP_THRESHOLD_C         60
#define OV_CTRLTEMP_HYSTERESIS_C        10
#define HW_OV_CURRENT_PROT_BYPASS       DISABLE /*!< In case ON_OVER_VOLTAGE
                                                          is set to TURN_ON_LOW_SIDES
                                                          this feature may be used to
                                                          bypass HW over-current
                                                          protection (if supported by
                                                          power stage) */

#define OVP_INVERTINGINPUT_MODE         INT_MODE
#define OVP_INVERTINGINPUT_MODE2        INT_MODE
#define OVP_SELECTION                   COMP_Selection_COMP1
#define OVP_SELECTION2                  COMP_Selection_COMP1

/******************************   START-UP PARAMETERS   **********************/

#define TRANSITION_DURATION            25  /* Switch over duration, ms */

/******************************   BUS VOLTAGE Motor 1  **********************/
#define  M1_VBUS_SAMPLING_TIME  LL_ADC_SAMPLING_CYCLE(47)
/******************************   Temperature sensing Motor 1  **********************/
#define  M1_TEMP_SAMPLING_TIME  LL_ADC_SAMPLING_CYCLE(47)
/******************************   Current sensing Motor 1   **********************/
#define ADC_SAMPLING_CYCLES (6 + SAMPLING_CYCLE_CORRECTION)
/******************************   ControlTemp sensing Motor 1   **********************/
#define  M1_CONTROLTEMP_SAMPLING_TIME  LL_ADC_SAMPLING_CYCLE(47)
/******************************   ControlTemp sensing Motor 1   **********************/
#define  M1_THROTTLE_SAMPLING_TIME  LL_ADC_SAMPLING_CYCLE(47)

#define OC_CURRENTTHRESHOLD      28000

//#define DACENABLED
#define DACDISABLED

/******************************   ADDITIONAL FEATURES   **********************/

/*** On the fly start-up ***/

/**************************
 *** Control Parameters ***
 **************************/

/* ##@@_USER_CODE_START_##@@ */
/* ##@@_USER_CODE_END_##@@ */

#endif /*DRIVE_PARAMETERS_H*/
/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
