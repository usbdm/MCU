/*
 * motor_faults.c
 *
 *  Created on: Jul 28, 2025
 *      Author: dell
 */
#include <stdlib.h>
#include "mc_api.h"
#include "mc_type.h"

#define IQ_STUCK_THRESHOLD_A    	 (6000u)
#define SPEED_STUCK_THRESHOLD   	 (10u)
#define STUCK_FAULT_COUNT_THRESHOLD  (4000u) // 0.5ms * 4000 = 2ms

static uint32_t stuck_counter; // Time the condition holds

uint32_t Check_Hall_Stuck_Fault(void)
{
	qd_t tempiq;
	int16_t motor_speed_rpm=0;
	tempiq = MC_GetIqdrefMotor1();
	motor_speed_rpm = SPEED_UNIT_2_RPM(MC_GetMecSpeedAverageMotor1());
    if ((tempiq.q> IQ_STUCK_THRESHOLD_A) &&
        (abs(motor_speed_rpm) < SPEED_STUCK_THRESHOLD))
    {
        stuck_counter++;

        if (stuck_counter >= STUCK_FAULT_COUNT_THRESHOLD)
        {
           return (MC_MOTORSTUCK_FAULT);
        }
    }
    else
    {
        stuck_counter = 0;
    }
  return 0; // no fault
}
