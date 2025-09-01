/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include "motorcontrol.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#define HALL_A_Pin GPIO_PIN_6
#define HALL_A_GPIO_Port GPIOB
#define HALL_B_Pin GPIO_PIN_7
#define HALL_B_GPIO_Port GPIOB
#define HALL_C_Pin GPIO_PIN_8
#define HALL_C_GPIO_Port GPIOB
/*----------------------------PWM------------------------------*/
#define PWM_L_U_Pin GPIO_PIN_13
#define PWM_L_U_GPIO_Port GPIOC
#define PWM_H_U_Pin GPIO_PIN_0
#define PWM_H_U_GPIO_Port GPIOC
#define PWM_H_V_Pin GPIO_PIN_1
#define PWM_H_V_GPIO_Port GPIOC
#define PWM_H_W_Pin GPIO_PIN_2
#define PWM_H_W_GPIO_Port GPIOC
#define PWM_L_V_Pin GPIO_PIN_14
#define PWM_L_V_GPIO_Port GPIOB
#define PWM_L_W_Pin GPIO_PIN_9
#define PWM_L_W_GPIO_Port GPIOB

/* ----------------------------ADC------------------------------*/
#define Vbatt_measure_Pin GPIO_PIN_0
#define Vbatt_measure_GPIO_Port GPIOA

#define CURRENT_SHUNT_U_OUT_Pin GPIO_PIN_2
#define CURRENT_SHUNT_U_OUT_GPIO_Port GPIOA

#define Controller_temp_sense_Pin GPIO_PIN_4
#define Controller_temp_sense_GPIO_Port GPIOA

#define CURRENT_SHUNT_V_OUT_Pin GPIO_PIN_6
#define CURRENT_SHUNT_V_OUT_GPIO_Port GPIOA

#define CURRENT_SHUNT_W_OUT_Pin GPIO_PIN_1
#define CURRENT_SHUNT_W_OUT_GPIO_Port GPIOB

#define Motor_temp_sense_Pin GPIO_PIN_3
#define Motor_temp_sense_GPIO_Port GPIOC

#define Throttle_Pin GPIO_PIN_4
#define Throttle_GPIO_Port GPIOC

/* ----------------------------COMP------------------------------*/
#define CURRENT_SHUNT_U_Pin GPIO_PIN_1
#define CURRENT_SHUNT_U_GPIO_Port GPIOA

#define CURRENT_SHUNT_V_P_Pin GPIO_PIN_7
#define CURRENT_SHUNT_V_P_GPIO_Port GPIOA

#define CURRENT_SHUNT_W_P_Pin GPIO_PIN_0
#define CURRENT_SHUNT_W_P_GPIO_Port GPIOB

/* ----------------------------OPAMP------------------------------*/
#define CURRENT_SHUNT_U_OUT_Pin GPIO_PIN_2
#define CURRENT_SHUNT_U_OUT_GPIO_Port GPIOA

#define CURRENT_SHUNT_U_N_Pin GPIO_PIN_3
#define CURRENT_SHUNT_U_N_GPIO_Port GPIOA

#define CURRENT_SHUNT_V_N_Pin GPIO_PIN_5
#define CURRENT_SHUNT_V_N_GPIO_Port GPIOA

#define CURRENT_SHUNT_V_OUT_Pin GPIO_PIN_6
#define CURRENT_SHUNT_V_OUT_GPIO_Port GPIOA

#define CURRENT_SHUNT_W_OUT_Pin GPIO_PIN_1
#define CURRENT_SHUNT_W_OUT_GPIO_Port GPIOB

#define CURRENT_SHUNT_W_N_Pin GPIO_PIN_2
#define CURRENT_SHUNT_W_N_GPIO_Port GPIOB

#define FDCAN1_TX_Pin_GPIO_PIN_12
#define FDCAN1_TX_Pin_GPIO_Port GPIOA

#define FDCAN1_RX_Pin_GPIO_PIN_11
#define FDCAN1_RX_Pin_GPIO_Port GPIOA


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

void DebugToggle(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWM_L_U_Pin GPIO_PIN_13
#define PWM_L_U_GPIO_Port GPIOC
#define RST_Pin GPIO_PIN_10
#define RST_GPIO_Port GPIOG
#define PWM_H_U_Pin GPIO_PIN_0
#define PWM_H_U_GPIO_Port GPIOC
#define PWM_H_V_Pin GPIO_PIN_1
#define PWM_H_V_GPIO_Port GPIOC
#define PWM_H_W_Pin GPIO_PIN_2
#define PWM_H_W_GPIO_Port GPIOC
#define Motor_temp_sense_Pin GPIO_PIN_3
#define Motor_temp_sense_GPIO_Port GPIOC
#define Vbatt_measure_Pin GPIO_PIN_0
#define Vbatt_measure_GPIO_Port GPIOA
#define CURRENT_SHUNT_U_Pin GPIO_PIN_1
#define CURRENT_SHUNT_U_GPIO_Port GPIOA
#define CURRENT_SHUNT_U_OUT_Pin GPIO_PIN_2
#define CURRENT_SHUNT_U_OUT_GPIO_Port GPIOA
#define CURRENT_SHUNT_U_N_Pin GPIO_PIN_3
#define CURRENT_SHUNT_U_N_GPIO_Port GPIOA
#define Controller_temp_sense_Pin GPIO_PIN_4
#define Controller_temp_sense_GPIO_Port GPIOA
#define CURRENT_SHUNT_V_N_Pin GPIO_PIN_5
#define CURRENT_SHUNT_V_N_GPIO_Port GPIOA
#define CURRENT_SHUNT_V_OUT_Pin GPIO_PIN_6
#define CURRENT_SHUNT_V_OUT_GPIO_Port GPIOA
#define CURRENT_SHUNT_V_P_Pin GPIO_PIN_7
#define CURRENT_SHUNT_V_P_GPIO_Port GPIOA
#define Throttle_Pin GPIO_PIN_4
#define Throttle_GPIO_Port GPIOC
#define CURRENT_SHUNT_W_P_Pin GPIO_PIN_0
#define CURRENT_SHUNT_W_P_GPIO_Port GPIOB
#define CURRENT_SHUNT_W_OUT_Pin GPIO_PIN_1
#define CURRENT_SHUNT_W_OUT_GPIO_Port GPIOB
#define CURRENT_SHUNT_W_N_Pin GPIO_PIN_2
#define CURRENT_SHUNT_W_N_GPIO_Port GPIOB
#define Eco_Pin GPIO_PIN_11
#define Eco_GPIO_Port GPIOB
#define City_Pin GPIO_PIN_12
#define City_GPIO_Port GPIOB
#define PWM_L_V_Pin GPIO_PIN_14
#define PWM_L_V_GPIO_Port GPIOB
#define Brake_Pin GPIO_PIN_15
#define Brake_GPIO_Port GPIOB
#define Boost_Pin GPIO_PIN_6
#define Boost_GPIO_Port GPIOC
#define Cruise_Pin GPIO_PIN_7
#define Cruise_GPIO_Port GPIOC
#define Acc_Pin GPIO_PIN_8
#define Acc_GPIO_Port GPIOC
#define Reverse_Pin GPIO_PIN_8
#define Reverse_GPIO_Port GPIOA
#define Antitheft_Pin GPIO_PIN_9
#define Antitheft_GPIO_Port GPIOA
#define Shutdown_Pin GPIO_PIN_10
#define Shutdown_GPIO_Port GPIOA
#define UART_TX_Pin GPIO_PIN_10
#define UART_TX_GPIO_Port GPIOC
#define UART_RX_Pin GPIO_PIN_11
#define UART_RX_GPIO_Port GPIOC
#define PWM_L_W_Pin GPIO_PIN_9
#define PWM_L_W_GPIO_Port GPIOB


/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
