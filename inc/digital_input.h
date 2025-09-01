/*
 * digital_input.h
 *
 *  Created on: 27-Mar-2023
 *      Author: sujatha
 */

#ifndef INC_DIGITAL_INPUT_H_
#define INC_DIGITAL_INPUT_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdbool.h>  // For bool type

/* Includes ------------------------------------------------------------------*/
bool DigitalInput_IsReversePressed(void);
bool DigitalInput_IsForwardPressed(void);
bool DigitalInput_IsBrakePressed(void);
bool DigitalInput_IsBrakeReleased(void);
bool DigitalInput_IsEcoModePressed(void);
bool DigitalInput_IsNormalModePressed(void);
bool DigitalInput_IsSportModePressed(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_DIGITAL_INPUT_H_ */


