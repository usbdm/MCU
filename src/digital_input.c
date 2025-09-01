/*
 * Digital_Input.c
 *
 *  Created on: 18-Mar-2023
 *
 */
#include "main.h"
#include "digital_input.h"

#define REVERSE_GPIO_PORT   GPIOA
#define REVERSE_GPIO_PIN    GPIO_PIN_8

#define BRAKE_GPIO_PORT     GPIOB
#define BRAKE_GPIO_PIN      GPIO_PIN_15

#define ECO_GPIO_PORT       GPIOB
#define ECO_GPIO_PIN        GPIO_PIN_11

#define NORMAL_GPIO_PORT    GPIOC
#define NORMAL_GPIO_PIN     GPIO_PIN_6


/**
 * @brief  Reads the status of the reverse direction input.
 *
 * @details This function checks the digital input pin connected to the reverse
 *          direction switch and returns its logic level. It is typically called
 *          periodically by higher-level modules like the direction control state machine.
 *
 * @note    This function assumes that the input GPIO has already been initialized
 *          elsewhere (e.g., during board initialization).
 *
 * @return  true  - if the reverse switch is active (logic HIGH or LOW based on design)
 * @return  false - if the forward switch is active
 */

bool DigitalInput_IsReversePressed(void)
{
	return (HAL_GPIO_ReadPin(REVERSE_GPIO_PORT, REVERSE_GPIO_PIN) == GPIO_PIN_SET);
}

/**
 * @brief  Reads the status of the reverse direction input.
 *
 * @details This function checks the digital input pin connected to the reverse
 *          direction switch and returns its logic level. It is typically called
 *          periodically by higher-level modules like the direction control state machine.
 *
 * @note    This function assumes that the input GPIO has already been initialized
 *          elsewhere (e.g., during board initialization).
 *
 * @return  true  - if the reverse switch is active (logic HIGH or LOW based on design)
 * @return  false - if the forward switch is active
 */
bool DigitalInput_IsBrakePressed(void)
{
	return (HAL_GPIO_ReadPin(BRAKE_GPIO_PORT, BRAKE_GPIO_PIN) == GPIO_PIN_SET);
}
/**
 * @brief  Reads the status of the reverse direction input.
 *
 * @details This function checks the digital input pin connected to the reverse
 *          direction switch and returns its logic level. It is typically called
 *          periodically by higher-level modules like the direction control state machine.
 *
 * @note    This function assumes that the input GPIO has already been initialized
 *          elsewhere (e.g., during board initialization).
 *
 * @return  true  - if the reverse switch is active (logic HIGH or LOW based on design)
 * @return  false - if the forward switch is active
 */
bool DigitalInput_IsEcoModePressed(void)
{
	return (HAL_GPIO_ReadPin(ECO_GPIO_PORT, ECO_GPIO_PIN) == GPIO_PIN_SET);
}
/**
 * @brief  Reads the status of the reverse direction input.
 *
 * @details This function checks the digital input pin connected to the reverse
 *          direction switch and returns its logic level. It is typically called
 *          periodically by higher-level modules like the direction control state machine.
 *
 * @note    This function assumes that the input GPIO has already been initialized
 *          elsewhere (e.g., during board initialization).
 *
 * @return  true  - if the reverse switch is active (logic HIGH or LOW based on design)
 * @return  false - if the forward switch is active
 */
bool DigitalInput_IsNormalModePressed(void)
{
	return (HAL_GPIO_ReadPin(NORMAL_GPIO_PORT, NORMAL_GPIO_PIN) == GPIO_PIN_RESET);
}

bool DigitalInput_IsSportModePressed(void)
{
	return (HAL_GPIO_ReadPin(NORMAL_GPIO_PORT, NORMAL_GPIO_PIN) == GPIO_PIN_SET);
}





