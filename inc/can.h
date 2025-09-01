/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.h
  * @brief   This file contains all the function prototypes for
  *          the fdcan.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */



/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#define CAN_FLAG_BRAKE_PRESSED       (1 << 0)
#define CAN_FLAG_DIRECTION_REVERSE   (1 << 1)
#define CAN_FLAG_INTERNAL			 (1 << 4)

// Bit manipulation macros
#define SET_FLAG(byte, flag)     ((byte) |= (flag))
#define CLEAR_FLAG(byte, flag)   ((byte) &= ~(flag))
#define IS_FLAG_SET(byte, flag)  ((byte) & (flag))
void CAN_Task(void);

typedef struct
{
    uint32_t message_id;
    uint8_t  dlc;              // Data Length Code
    uint16_t period_ms;        // Transmission period in ms
    bool     enabled;          // Whether this message is currently active
} CanMessageConfig_t;

void Can_Task(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

