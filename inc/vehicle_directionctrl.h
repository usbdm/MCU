/*
 * vehicle_directionctrl.h
 *
 *  Created on: Jun 15, 2025
 *      Author: dell
 */

#ifndef VEHICLE_DIRECTIONCTRL_H_
#define VEHICLE_DIRECTIONCTRL_H_

/** @brief Drive direction */
typedef enum {
    DIRCTRL_DIR_FWD,
    DIRCTRL_DIR_REV
} DirCtrl_Direction;

/** @brief Drive mode */
typedef enum {
    DIRCTRL_MODE_ECO,
    DIRCTRL_MODE_NORMAL,
    DIRCTRL_MODE_SPORT
} DirCtrl_Mode;

/** @brief Internal state machine state */
typedef enum {
    DIRCTRL_STATE_IDLE,
    DIRCTRL_STATE_DRIVE,
	DIRCTRL_STATE_SMOOTH_STOP
} DirCtrl_State;

/**
 * @brief Initialize the direction control module.
 *
 * Call this once before entering main loop. Resets state machine to IDLE.
 */
void DirCtrl_Init(void);

/**
 * @brief Run the main state machine loop.
 *
 * Call periodically (e.g. every 1ms) from main loop or RTOS task.
 */
void DirCtrl_Run(void);

/**
 * @brief Get the currently active drive direction.
 *
 * @return DIRCTRL_DIR_FWD or DIRCTRL_DIR_REV
 */
DirCtrl_Direction DirCtrl_GetCurrentDirection(void);

/**
 * @brief Set the requested direction from user input (e.g. switch)
 *
 * This sets the "intent", not the actual running direction.
 * Direction will only change after braking to 0.
 */
void DirCtrl_SetRequestedDirection(DirCtrl_Direction direction);

/**
 * @brief Set the requested drive mode from user input (ECO/NORMAL/SPORT)
 */
void DirCtrl_SetDriveMode(DirCtrl_Mode mode);

/**
 * @brief Set current throttle value (0.0 to 1.0)
 */
void DirCtrl_SetThrottle(float throttle);

/**
 * @brief Set brake button input (true = pressed)
 */
void DirCtrl_SetBrake(bool brake_pressed);

DirCtrl_Direction DirCtrl_GetRequestedDirection(void);

DirCtrl_Mode DirCtrl_GetRequestedMode(void);

void StopMotorSmoothlyTask(void);
void StopMotorSmoothlyInit(void);

#endif /* VEHICLE_DIRECTIONCTRL_H_ */
