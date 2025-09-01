/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
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
/* Includes ------------------------------------------------------------------*/
#include <digital_input.h>
#include "can.h"
#include "mc_type.h"
#include "mc_interface.h"
#include "mc_api.h"
#include "drive_parameters.h"
#include "bus_voltage_sensor.h"
#include "mc_config.h"
#include "vehicle_directionctrl.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_TxHeaderTypeDef TxHeader1;
extern FDCAN_RxHeaderTypeDef RxHeader1;
#define ONESECOND							(1000u)
#define CAN_DLC								(0x8u)

#define CAN_PUT_U16(arr, index, val)  \
    do {                              \
        arr[index]     = (val) & 0xFF;      \
        arr[index + 1] = ((val) >> 8) & 0xFF; \
    } while (0)

/* USER CODE BEGIN 0 */
typedef enum {
    CAN_TX_MCU_PARAMETERS         = 0x322u,
    CAN_TX_MCU_PARAMETERS_1       = 0x323u,
    CAN_TX_MCU_PARAMETERS_2       = 0x324u,
    CAN_TX_MCU_PARAMETERS_4       = 0x327u,
    CAN_TX_MCU_PARAMETERS_5       = 0x329u,
    CAN_TX_MCU_FAULT_MESSAGE      = 0x326u,
    CAN_TX_DEBUG_MSG_2            = 0x501u,
} CanTxMessageId_e;

typedef enum {
    CAN_RX_MCU_RECEIVE_MESSAGE_3       = 0x402u,
} CanRxMessageId_e;

typedef enum {
	CAN_TX_MSG_MCU_PARAMETERS = 0,
	CAN_TX_MSG_MCU_PARAMETERS_1,
	CAN_TX_MSG_MCU_PARAMETERS_2,
	CAN_TX_MSG_MCU_PARAMETERS_4,
	CAN_TX_MSG_MCU_PARAMETERS_5,
	CAN_TX_MSG_FAULT,
	CAN_TX_MSG_DEBUG_2,
	CAN_TX_MSG_COUNT // Total number of messages
} CanTxMsgId_e;

typedef enum {
	CAN_RX_MSG_MCU_RECEIVE_MESSAGE_3 = 0,
	CAN_RX_MSG_COUNT // Total number of messages
} CanRxsgId_e;

static uint8_t g_can_tx_messages_u8aa[CAN_TX_MSG_COUNT][CAN_DLC];
static uint8_t g_canset_u8=0;
static uint8_t RxData[8];
static uint8_t g_sw_major_version_u8='7';
static uint8_t g_sw_minor_version_u8='0';
static uint8_t g_sw_patch_version_u8='0';
static uint16_t g_counter_u16=0;
extern PWMC_R3_2_Handle_t PWM_Handle_M1;

static void CanEncodeMessage322(void);
static void CanEncodeMessage323(void);
static void CanEncodeMessage324(void);
static void CanEncodeMessage326(void);
static void CanEncodeMessage327(void);
static void CanEncodeMessage329(void);
static void CanEncodeMessage501(void);

/*
  * Function to transmit CAN at every 225ms
 */
void CanTransmit(void)
{
	uint32_t l_txFiforequest_u32=0;

if(g_canset_u8==1)
{
	CanEncodeMessage322();
	CanEncodeMessage323();
	CanEncodeMessage324();

	if(HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1)>0)
	 {
		 TxHeader1.Identifier = CAN_TX_MCU_PARAMETERS;
		 if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader1,g_can_tx_messages_u8aa[CAN_TX_MSG_MCU_PARAMETERS]) != HAL_OK)
		 {
			Error_Handler();
		 }
	 }
	else
	 {
	   l_txFiforequest_u32 = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
	   if(HAL_FDCAN_IsTxBufferMessagePending(&hfdcan1,l_txFiforequest_u32))
	   {

	   }
	 }
	if(HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1)>0)
	{
	   TxHeader1.Identifier = CAN_TX_MCU_PARAMETERS_1;
	   if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader1,g_can_tx_messages_u8aa[CAN_TX_MSG_MCU_PARAMETERS_1]) != HAL_OK)
		 {
			  Error_Handler();
		 }
	}
	else
	{
		l_txFiforequest_u32 = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
		if(HAL_FDCAN_IsTxBufferMessagePending(&hfdcan1,l_txFiforequest_u32))
		 {

		 }
	}

	if(HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1)>0)
	{
	   TxHeader1.Identifier = CAN_TX_MCU_PARAMETERS_2;
	   if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader1, g_can_tx_messages_u8aa[CAN_TX_MSG_MCU_PARAMETERS_2]) != HAL_OK)
		 {
			  Error_Handler();
		 }
	}
	else
	{
		l_txFiforequest_u32 = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
		if(HAL_FDCAN_IsTxBufferMessagePending(&hfdcan1,l_txFiforequest_u32))
		 {
		 }
	}
}

if(g_canset_u8==2)
{
	CanEncodeMessage326();
	CanEncodeMessage329();
	CanEncodeMessage327();

	if(HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1)>0)
		{
		   TxHeader1.Identifier = CAN_TX_MCU_FAULT_MESSAGE;
		   if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader1,g_can_tx_messages_u8aa[CAN_TX_MSG_FAULT]) != HAL_OK)
			 {
				  Error_Handler();
			 }
		}

	else
	{
		l_txFiforequest_u32 = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
		if(HAL_FDCAN_IsTxBufferMessagePending(&hfdcan1,l_txFiforequest_u32))
		 {

		 }
	}

	if(HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1)>0)
		{
		   TxHeader1.Identifier = CAN_TX_MCU_PARAMETERS_5;
		   if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader1, g_can_tx_messages_u8aa[CAN_TX_MSG_MCU_PARAMETERS_5]) != HAL_OK)
			 {
				  Error_Handler();
			 }
		}

	else
	{
		l_txFiforequest_u32 = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
		if(HAL_FDCAN_IsTxBufferMessagePending(&hfdcan1,l_txFiforequest_u32))
		 {

		 }
	}

	if(HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1)>0)
		{
		   TxHeader1.Identifier = CAN_TX_MCU_PARAMETERS_4;
		   if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader1, g_can_tx_messages_u8aa[CAN_TX_MSG_MCU_PARAMETERS_4]) != HAL_OK)
			 {
				  Error_Handler();
			 }
		}
		else
		{
			l_txFiforequest_u32 = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
			if(HAL_FDCAN_IsTxBufferMessagePending(&hfdcan1,l_txFiforequest_u32))
			 {

			 }
		}


		if(HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1)>0)
		{
		   TxHeader1.Identifier = CAN_TX_DEBUG_MSG_2;
		   if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader1, g_can_tx_messages_u8aa[CAN_TX_MSG_DEBUG_2]) != HAL_OK)
			{
				  Error_Handler();
			}
		}
		else
		{
			l_txFiforequest_u32 = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
			if(HAL_FDCAN_IsTxBufferMessagePending(&hfdcan1,l_txFiforequest_u32))
			 {
			 }
		}
}  /*End of Canset2 */


if(g_canset_u8==3)
{
	CanEncodeMessage501();
	if(HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1)>0)
	{
	   TxHeader1.Identifier = CAN_TX_DEBUG_MSG_2;
	   if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader1, g_can_tx_messages_u8aa[CAN_TX_MSG_DEBUG_2]) != HAL_OK)
		{
			  Error_Handler();
		}
	}
	else
	{
		l_txFiforequest_u32 = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
		if(HAL_FDCAN_IsTxBufferMessagePending(&hfdcan1,l_txFiforequest_u32))
		 {
		 }
	}

 } /*end of canset3 */

}
/**
  * @brief Receives CAN Messaages
  *
  * This function receives CAN messages
  */
/* USER CODE BEGIN 1 */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{

  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    /* Retrieve Rx messages from RX FIFO0 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1, RxData) != HAL_OK)
    {
      Error_Handler();
    }
   switch (RxHeader1.Identifier)
   {
     case CAN_RX_MCU_RECEIVE_MESSAGE_3:

       if (RxData[5] == 1)
        {
              MC_AcknowledgeFaultMotor1();
        }
        break;
     default:
    	 break;
   }
  }

  if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
   {
 	  /* Notification Error */
 	  Error_Handler();
   }
}

/*
 * Can Task configured for 5ms.
 */

void CAN_Task(void)
{
	g_counter_u16++;
	if(g_counter_u16 >= ONESECOND)
	{
		DebugToggle();
		g_canset_u8++;
        CanTransmit();
		if(g_canset_u8 >=3 )
		{
		 g_canset_u8=0;
		}
		g_counter_u16=0;
	}
}

static uint8_t heartbeatcounter=0;

static void CanEncodeMessage322(void)
{
    uint16_t voltage; //= VBS_GetAvBusVoltage_V(&BusVoltageSensor_M1);
    uint8_t temp_ctrl = CTS_GetAvTemp_C(pControlTemperatureSensor[0]);
    qd_t i_ref = MCI_GetIqdref(&Mci[0]);


    heartbeatcounter++;
    if (heartbeatcounter >= 16) {
        heartbeatcounter = 0;
    }

    CAN_PUT_U16(g_can_tx_messages_u8aa[CAN_TX_MSG_MCU_PARAMETERS], 0, voltage);
    g_can_tx_messages_u8aa[CAN_TX_MSG_MCU_PARAMETERS][2] = temp_ctrl;
    /*******************4. Phase C offset ***************/
    g_can_tx_messages_u8aa[CAN_TX_MSG_MCU_PARAMETERS][7] = heartbeatcounter & 0x0F;

    CAN_PUT_U16(g_can_tx_messages_u8aa[CAN_TX_MSG_MCU_PARAMETERS_1], 3, i_ref.d + 32767);
    CAN_PUT_U16(g_can_tx_messages_u8aa[CAN_TX_MSG_MCU_PARAMETERS_1], 5, i_ref.q + 32767);

    if (MC_GetDriveinput(&Mci[0])) {
               SET_FLAG(g_can_tx_messages_u8aa[CAN_TX_MSG_MCU_PARAMETERS_1][7], CAN_FLAG_INTERNAL); // EXTERNAL command
     } else {
               CLEAR_FLAG(g_can_tx_messages_u8aa[CAN_TX_MSG_MCU_PARAMETERS_1][7], CAN_FLAG_INTERNAL);  // INTERNAL command
    }
}

static void CanEncodeMessage323(void)
{
	qd_t i_present = MCI_GetIqd(&Mci[0]);
    CAN_PUT_U16(g_can_tx_messages_u8aa[CAN_TX_MSG_MCU_PARAMETERS_1], 0, i_present.d + 32767);
    CAN_PUT_U16(g_can_tx_messages_u8aa[CAN_TX_MSG_MCU_PARAMETERS_1], 2, i_present.q + 32767);

	/*******Valpha and VBeta *****************/
	alphabeta_t l_v_alphabeta_s = MCI_GetValphabeta(&Mci[0]);
	CAN_PUT_U16(g_can_tx_messages_u8aa[CAN_TX_MSG_MCU_PARAMETERS_1], 4, l_v_alphabeta_s.alpha+32767);
	CAN_PUT_U16(g_can_tx_messages_u8aa[CAN_TX_MSG_MCU_PARAMETERS_1], 6, l_v_alphabeta_s.beta+32767);
}

static void CanEncodeMessage324(void)
{
	g_can_tx_messages_u8aa[CAN_TX_MSG_MCU_PARAMETERS_2][0] = Throttle_GetAvPercentage_d(pThrottleSensor[0]);
	g_can_tx_messages_u8aa[CAN_TX_MSG_MCU_PARAMETERS_2][1] = DirCtrl_GetRequestedMode();  // Vehicle mode placeholder, eco, normal, sports

    // Rotor position in dpp
    int16_t rotor_pos = MCI_GetElAngledpp(&Mci[0]);
    CAN_PUT_U16(g_can_tx_messages_u8aa[CAN_TX_MSG_MCU_PARAMETERS_2], 5, rotor_pos + 32767);

    if (DigitalInput_IsBrakePressed()) {
           SET_FLAG(g_can_tx_messages_u8aa[CAN_TX_MSG_MCU_PARAMETERS_2][7], CAN_FLAG_BRAKE_PRESSED);
     } else {
           CLEAR_FLAG(g_can_tx_messages_u8aa[CAN_TX_MSG_MCU_PARAMETERS_2][7], CAN_FLAG_BRAKE_PRESSED);
     }
    // Set or clear direction
    if(DirCtrl_GetRequestedDirection())
     {
      SET_FLAG(g_can_tx_messages_u8aa[CAN_TX_MSG_MCU_PARAMETERS_2][7], CAN_FLAG_DIRECTION_REVERSE);
     }
     else
     {
      CLEAR_FLAG(g_can_tx_messages_u8aa[CAN_TX_MSG_MCU_PARAMETERS_2][7], CAN_FLAG_DIRECTION_REVERSE);
     }

    g_can_tx_messages_u8aa[CAN_TX_MSG_MCU_PARAMETERS_2][2] = g_sw_major_version_u8;
    g_can_tx_messages_u8aa[CAN_TX_MSG_MCU_PARAMETERS_2][3] = g_sw_minor_version_u8;
    g_can_tx_messages_u8aa[CAN_TX_MSG_MCU_PARAMETERS_2][4] = g_sw_patch_version_u8;
}


static void CanEncodeMessage327(void)
{
	ab_t l_currents_ab={0,0};
	int16_t l_temp_ic_s16=0;
    uint16_t speed = SPEED_UNIT_2_RPM(MC_GetMecSpeedAverageMotor1()); // Temporarily removing the 32767 offset value for testing

	/******************** Message Id 0x327 ***************************/
	/********** Transmitting Ia, Ib, Ic over CAN **********/
	/*******************1.Transmit Ia *********************/
	l_currents_ab = MCI_GetIab(&Mci[0]);
	/*To add offset and make positive from negative values */
	CAN_PUT_U16(g_can_tx_messages_u8aa[CAN_TX_MSG_MCU_PARAMETERS_4], 0, l_currents_ab.a+32767);
	/*****************2.Transmit Ib ***************/
	CAN_PUT_U16(g_can_tx_messages_u8aa[CAN_TX_MSG_MCU_PARAMETERS_4], 2, l_currents_ab.b+32767);
 /*********************3.Transmit Ic ******************/
	l_temp_ic_s16 = Mci[0].pPWM->Ic;
	CAN_PUT_U16(g_can_tx_messages_u8aa[CAN_TX_MSG_MCU_PARAMETERS_4], 4, l_temp_ic_s16 +32767);

	CAN_PUT_U16(g_can_tx_messages_u8aa[CAN_TX_MSG_MCU_PARAMETERS_4], 6, speed);

}

static void CanEncodeMessage329(void)
{
	uint8_t l_statemode_u8=0;
	uint16_t l_phasevoltage_u16=0;
	uint16_t l_motorpower_u16=0;
	uint16_t l_phasecurrent_u16=0;
	qd_t l_v_dq_s= MCI_GetVqd(&Mci[0]);

	l_statemode_u8 = MC_GetSTMStateMotor1(); // this returns  START = 4, RUN = 6, etc.
	l_phasecurrent_u16  =  (MC_GetPhaseCurrentAmplitudeMotor1() + 32767);
	l_phasevoltage_u16  = (MC_GetPhaseVoltageAmplitudeMotor1() + 32767);
	l_motorpower_u16  = (MC_GetAveragePowerMotor1_F()*10);

	g_can_tx_messages_u8aa[CAN_TX_MSG_MCU_PARAMETERS_5][0] = l_statemode_u8 ;
	CAN_PUT_U16(g_can_tx_messages_u8aa[CAN_TX_MSG_MCU_PARAMETERS_5], 1, l_phasecurrent_u16);
	CAN_PUT_U16(g_can_tx_messages_u8aa[CAN_TX_MSG_MCU_PARAMETERS_5], 3, l_phasevoltage_u16);
	CAN_PUT_U16(g_can_tx_messages_u8aa[CAN_TX_MSG_MCU_PARAMETERS_5], 5, l_motorpower_u16);
	CAN_PUT_U16(g_can_tx_messages_u8aa[CAN_TX_MSG_MCU_PARAMETERS_5], 4, l_v_dq_s.q + 32767);
}

static void CanEncodeMessage326(void)
{
	uint32_t l_presentfaultstate_u32=0;
	uint32_t l_pastfaultstate_u32=0;
		/*************1. Present Fault *******************/
		l_presentfaultstate_u32 = MCI_GetCurrentFaults(&Mci[0]);
		g_can_tx_messages_u8aa[CAN_TX_MSG_FAULT][0] =  (uint8_t) (l_presentfaultstate_u32 & 0xff);
		g_can_tx_messages_u8aa[CAN_TX_MSG_FAULT][1]  = (uint8_t)((l_presentfaultstate_u32 & 0xff00)>> 8) ;
		g_can_tx_messages_u8aa[CAN_TX_MSG_FAULT][2]  = (uint8_t)((l_presentfaultstate_u32 & 0xff0000)>> 16) ;
		g_can_tx_messages_u8aa[CAN_TX_MSG_FAULT][3]  = (uint8_t)((l_presentfaultstate_u32 & 0xff000000)>> 24) ;

		/*************2. Past Fault *******************/
		l_pastfaultstate_u32 = MCI_GetOccurredFaults(&Mci[0]);
		g_can_tx_messages_u8aa[CAN_TX_MSG_FAULT][4] =  (uint8_t)(l_pastfaultstate_u32 & 0xff);
		g_can_tx_messages_u8aa[CAN_TX_MSG_FAULT][5]  = (uint8_t)((l_pastfaultstate_u32 & 0xff00)>> 8) ;
		g_can_tx_messages_u8aa[CAN_TX_MSG_FAULT][6]  = (uint8_t)((l_pastfaultstate_u32 & 0xff0000)>> 16) ;
		g_can_tx_messages_u8aa[CAN_TX_MSG_FAULT][7]  = (uint8_t)((l_pastfaultstate_u32 & 0xff000000)>> 24) ;
}


static void CanEncodeMessage501(void)
{
	uint16_t l_temp_phaseaoffset_u16=0;
	uint16_t l_temp_phaseboffset_u16=0;
	uint16_t l_temp_phasecoffset_u16 = PWM_Handle_M1.PhaseCOffset;
	qd_t l_v_dq_s;

	/***********3. PhaseAoffset ************************/
	l_temp_phaseaoffset_u16 = PWM_Handle_M1.PhaseAOffset;
	CAN_PUT_U16(g_can_tx_messages_u8aa[CAN_TX_MSG_DEBUG_2], 0, l_temp_phaseaoffset_u16);
	/***********4. PhaseBoffset ************************/
	l_temp_phaseboffset_u16 = PWM_Handle_M1.PhaseBOffset;
	CAN_PUT_U16(g_can_tx_messages_u8aa[CAN_TX_MSG_DEBUG_2], 2, l_temp_phaseboffset_u16);

	CAN_PUT_U16(g_can_tx_messages_u8aa[CAN_TX_MSG_MCU_PARAMETERS], 4, l_temp_phasecoffset_u16);

	/******************** Message Id 0x325 ***************************/

	l_v_dq_s = MCI_GetVqd(&Mci[0]);
	CAN_PUT_U16(g_can_tx_messages_u8aa[CAN_TX_MSG_MCU_PARAMETERS], 4, l_v_dq_s.d + 32767);

}


