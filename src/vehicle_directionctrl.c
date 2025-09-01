#include <stdlib.h>
#include "mc_type.h"
#include "mc_api.h"
#include "vehicle_directionctrl.h"
#include "digital_input.h"
#include "throttle.h"
#include "mc_config.h"
#include "parameters_conversion.h"
#include "can.h"

/** Defines *****/
#define THROTTLE_MIN_START  			(10u)
#define MOTOR_STOP_RPM_THRESHOLD    	(0u)

#define ECO_MODE_SPEEDLIMIT				(250u)   // 90% of 220 = 198
#define NORMAL_MODE_SPEEDLIMIT			(350u)   // 90% of 350 = 351
#define SPORTS_MODE_SPEEDLIMIT			(420u)   // 90% of 450 = 432
#define DEFAULT_SPEED_LIMIT				 ECO_MODE_SPEEDLIMIT
#define REVERSE_MODE_SPEEDLIMIT			(300u)

#define ECOMODE_MI_MAX					(16700u)  // 15k is 260 rpm as per cubemonitor
#define NORMALMODE_MI_MAX				(22200u)  // 20k is 350 rpm
#define SPORTSMODE_MI_MAX				(24000u)  // 22k is 390 rpm

#define VQ_RAMP_UP_STEP     			(10u)   // Faster ramp-up
#define VQ_RAMP_DOWN_STEP   			(1u)   // Slower ramp-down 1u is mandatory to use

#define REVERSE_MI_MAX					(12000u)


static inline int16_t ABS16(int16_t x)
{
    return (x < 0) ? -x : x;
}

#define MIN(a, b) ((a) < (b) ? (a) : (b))

typedef struct {
    DirCtrl_Direction current_dir;
    DirCtrl_Direction requested_dir;
    DirCtrl_Mode current_mode;
    DirCtrl_Mode requested_mode;
    DirCtrl_State state;
    bool reverse_requested;
    DirCtrl_Mode drive_mode;
    uint8_t throttle;
    bool brake_pressed;
} DirCtrl_Context;

static DirCtrl_Context ctx = {
    .current_dir       = DIRCTRL_DIR_FWD,
    .requested_dir     = DIRCTRL_DIR_FWD,
    .state             = DIRCTRL_STATE_IDLE,
    .reverse_requested = false,
    .drive_mode        = DIRCTRL_MODE_NORMAL,
    .throttle          = 0,
    .brake_pressed     = false,
	.current_mode = DIRCTRL_MODE_ECO
};

typedef struct
{
    int16_t speed_limit_fwd;
    int16_t speed_limit_rev;
    int16_t iq_max;
    uint16_t vq_limit;
    // Add more mode-specific parameters as needed
} DirCtrl_ModeParams_t;

// Persistent overspeed flag
static int16_t iq_max = IQMAX;

int16_t current_speed_rpm=0;
static uint16_t vq_effective=0;
static uint16_t vq_target=0;

//--------------------------------------------------
// Internal helper prototypes (low-level motor control)
//--------------------------------------------------
static void Motor_Run(DirCtrl_Direction dir, DirCtrl_Mode current_mode,
               uint8_t throttle_percent);
static bool IsMotorStopped(void);
static int16_t Calculate_iq_from_throttle(uint8_t throttle_percent, bool is_reverse,
		DirCtrl_Mode current_mode,bool brake_pressed,
		int16_t current_speed_rpm);
void StopMotorSmoothlyTask(void);
static DirCtrl_ModeParams_t DirCtrl_GetParamsForMode(DirCtrl_Mode mode);
void Update_VqRamp(uint16_t vq_target);

void DirCtrl_Init(void)
{
    ctx.current_dir       = DIRCTRL_DIR_FWD;
    ctx.requested_dir     = DIRCTRL_DIR_FWD;
    ctx.state             = DIRCTRL_STATE_IDLE;
    ctx.reverse_requested = false;
    ctx.drive_mode        = DIRCTRL_MODE_NORMAL;
    ctx.throttle          = 0;
}


void StopMotorSmoothlyTask(void)
{
	// Apply updated iq
	MC_ProgramTorqueRampMotor1(0,200);
}

/**
 * @brief Top-level motor control state machine.
 *
 * This function manages transitions between IDLE, DRIVE, and BRAKING states.
 * It checks throttle, brake, and direction switch inputs, and ensures safe
 * transitions between forward and reverse modes by forcing a complete stop
 * before direction change.
 *
 * Called periodically at fixed intervals (e.g., 1ms).
 *
 * Inputs:
 *  - Throttle value (0.0 to 1.0)
 *  - Brake button (boolean)
 *  - Direction switch (FWD or REV)
 *  - Drive mode switch (Eco, Normal, Sport)
 *
 * Outputs:
 *  - Updates motor speed and direction
 *  - May trigger braking or drive mode change
 *
 * Safety:
 *  - Prevents sudden direction reversal
 *  - Handles mode transitions smoothly
 */
static int16_t speed_rpm=0;

void DirCtrl_Run(void)
{
	// 1. Update requested direction based on GPIO input
	ctx.requested_dir = DirCtrl_GetRequestedDirection();

    //2. Update drive mode based on the switch status
     ctx.current_mode = DirCtrl_GetRequestedMode();
    //3 Get the throttle position

    ctx.throttle  = Throttle_GetAvPercentage_d(&ThrottleSensor_M1);

    ctx.brake_pressed  = DigitalInput_IsBrakePressed();

    speed_rpm = SPEED_UNIT_2_RPM(MC_GetMecSpeedAverageMotor1());
    DirCtrl_ModeParams_t mode_params = DirCtrl_GetParamsForMode(ctx.current_mode);

    if(speed_rpm == 0) {
    	CircleLimitationM1.MaxModule = mode_params.vq_limit;
    }

    // 4. State machine
    switch (ctx.state)
    {
        case DIRCTRL_STATE_IDLE:

        	if ((ctx.current_dir != ctx.requested_dir) && IsMotorStopped())
        	{
        	    ctx.current_dir = ctx.requested_dir;
        	}

            if (ctx.throttle > THROTTLE_MIN_START && !ctx.brake_pressed)
            {
                if (ctx.current_dir != ctx.requested_dir)
                {
                    ctx.reverse_requested = true;
                    ctx.state = DIRCTRL_STATE_SMOOTH_STOP;
                }
                else
                {
                    ctx.state = DIRCTRL_STATE_DRIVE;
                }
            }
            break;

        case DIRCTRL_STATE_DRIVE:
        {

		  // --- Smooth stop when throttle is low or brake is pressed ---
   		   if (ctx.brake_pressed || ctx.throttle < THROTTLE_MIN_START || (ctx.current_dir != ctx.requested_dir))
			{
   			   if(ctx.current_dir != ctx.requested_dir){
   				ctx.reverse_requested = true;
   			   }

   			   ctx.state = DIRCTRL_STATE_SMOOTH_STOP;
			}
			else
			{
				Motor_Run(ctx.current_dir, ctx.current_mode,ctx.throttle); // Normal operation
			}
        }
		break;

		case DIRCTRL_STATE_SMOOTH_STOP:
		{
			 StopMotorSmoothlyTask();
			 if (ctx.throttle > THROTTLE_MIN_START && !ctx.brake_pressed && !ctx.reverse_requested)
			 {
				if (!IsMotorStopped())
				 {
					 ctx.state = DIRCTRL_STATE_DRIVE;  // Resume directly
				 }
			 }
			 else
			 {
			 // If throttle is not pressed and Iq is zero + motor is stopped, go to IDLE
				 if(IsMotorStopped())
				 {
					 if (ctx.reverse_requested)
					 {
					   ctx.current_dir = ctx.requested_dir;
					   ctx.reverse_requested = false;
					 }
					 ctx.state = DIRCTRL_STATE_IDLE;
					 MC_StopMotor1();
				 }
			 }
		}
        break;

		 default:
            ctx.state = DIRCTRL_STATE_IDLE;
            break;
    }
}
/**
 * @brief Runs the motor in the specified direction and drive mode.
 *
 * Computes target speed from throttle and applies acceleration limits
 * based on selected drive mode.
 *
 * @param dir   - Desired direction (FWD or REV)
 * @param mode  - Drive mode (ECO / NORMAL / SPORT)
 * @param throttle - Throttle value (0.0 to 1.0)
 *
 * Assumptions:
 *  - Called only when motor is in DRIVE state
 *  - Direction must already match hardware setup
 */

static void Motor_Run(DirCtrl_Direction dir, DirCtrl_Mode current_mode, uint8_t throttle_percent)
{
	int16_t iq_ref = 0;
	bool brake_status=0;
	qd_t iq_cmd = {0};
	  // D-Q command structure (ST MCSDK uses this)

	brake_status = DigitalInput_IsBrakePressed();
	current_speed_rpm = SPEED_UNIT_2_RPM(MC_GetMecSpeedAverageMotor1());
	// Direction check
	bool is_reverse_cmd = (dir == DIRCTRL_DIR_REV);

  // Calculate Torque current
	iq_ref = Calculate_iq_from_throttle(throttle_percent, is_reverse_cmd, current_mode,
			brake_status, current_speed_rpm);

  // Set D = 0, Q = Iq_ref
	iq_cmd.d = 0;
	iq_cmd.q = iq_ref;

	// Send the command to motor controller
	MC_ProgramTorqueRampMotor1(iq_cmd.q, 200);

	// start the motor
	if ((MC_GetSTMStateMotor1() == IDLE) && (!brake_status))
	{
		MC_StartMotor1();
	}
}


static int16_t Calculate_iq_from_throttle(uint8_t throttle_percent, bool is_reverse,
		DirCtrl_Mode current_mode,bool brake_pressed,
		int16_t current_speed_rpm)
{
    uint32_t temp_mi=0;
    int16_t iq_target=0;


    // If brake is pressed, do not generate any torque
	if(brake_pressed)
	  return 0;

    // Clamp input
    if (throttle_percent >= 100)
        throttle_percent = 100;

    throttle_percent = 25;

    // this condition will ensure that the vehicle can start even under loaded conditions
    // once the vehicle starts then the parameters can be updated based on the mode and rpm basis

       if(!is_reverse){
    	   DirCtrl_ModeParams_t mode_params = DirCtrl_GetParamsForMode(current_mode);
    	   temp_mi = ((uint32_t)mode_params.vq_limit * throttle_percent);
       } else{
    	   temp_mi = ((uint32_t)REVERSE_MI_MAX * throttle_percent);
       }

     vq_target = (uint16_t)(temp_mi/100);
	   Update_VqRamp(vq_target);
	   CircleLimitationM1.MaxModule = vq_effective;

	// Hold the max iq throughout the modes only vary the MI
	 if(throttle_percent < 30){
    	 iq_target = ((int32_t)throttle_percent *iq_max*80)/(30*100);
     } else{
    	 iq_target =  ((int32_t)iq_max * 80)/100 + ((int32_t)throttle_percent - 30)*iq_max * 20 / (70 * 100) ;
     }

      if(is_reverse) {
    	  iq_target = -iq_target;
      }

  //    iq_target = 26000;
      return iq_target;
}

static bool IsMotorStopped(void)
{
	int16_t l_ctxmotorspeed_s16=0;
	l_ctxmotorspeed_s16 = SPEED_UNIT_2_RPM(MC_GetMecSpeedAverageMotor1());
	return (ABS16(l_ctxmotorspeed_s16) <= MOTOR_STOP_RPM_THRESHOLD);
}

DirCtrl_Mode DirCtrl_GetRequestedMode(void)
{
    if (DigitalInput_IsEcoModePressed())
    {
      return DIRCTRL_MODE_ECO;
    }
    else if (DigitalInput_IsNormalModePressed())
    {
      return DIRCTRL_MODE_NORMAL;
    }
    else if (DigitalInput_IsSportModePressed())
    {
      return DIRCTRL_MODE_SPORT;
    }
    else
    {
      return DIRCTRL_MODE_NORMAL; // Default fallback it will never reach here.
    }
}

DirCtrl_Direction DirCtrl_GetRequestedDirection(void)
{
	 if(DigitalInput_IsReversePressed())
	 {
	   return DIRCTRL_DIR_REV;
	 }
	 else
	 {
	   return DIRCTRL_DIR_FWD;
	 }
}

static DirCtrl_ModeParams_t DirCtrl_GetParamsForMode(DirCtrl_Mode mode)
{
    DirCtrl_ModeParams_t params;

    switch (mode)
    {
        case DIRCTRL_MODE_ECO:
            params.speed_limit_fwd = ECO_MODE_SPEEDLIMIT;
            params.speed_limit_rev = REVERSE_MODE_SPEEDLIMIT;
            params.iq_max          = 1000;
            params.vq_limit        = ECOMODE_MI_MAX;
            break;

        case DIRCTRL_MODE_NORMAL:
            params.speed_limit_fwd = NORMAL_MODE_SPEEDLIMIT;
            params.speed_limit_rev = REVERSE_MODE_SPEEDLIMIT;
            params.iq_max          = 2000;
            params.vq_limit        = NORMALMODE_MI_MAX;
            break;

        case DIRCTRL_MODE_SPORT:
            params.speed_limit_fwd = SPORTS_MODE_SPEEDLIMIT;
            params.speed_limit_rev = REVERSE_MODE_SPEEDLIMIT;
            params.iq_max          = 2000;
            params.vq_limit        = SPORTSMODE_MI_MAX;
            break;

        default:
            params.speed_limit_fwd = DEFAULT_SPEED_LIMIT;
            params.speed_limit_rev = DEFAULT_SPEED_LIMIT;
            params.iq_max          = 2000;
            params.vq_limit        = SPORTSMODE_MI_MAX;
            break;
    }

    return params;
}

/* Vq ramp update instead of sudden fall */

void Update_VqRamp(uint16_t vq_target)
{
	if(vq_effective < vq_target)
	 {
	        // Ramp up quickly
	        vq_effective += MIN(VQ_RAMP_UP_STEP, vq_target - vq_effective);
	  }
	else if (vq_effective > vq_target)
	  {
	        // Ramp down slowly
	        vq_effective -= MIN(VQ_RAMP_DOWN_STEP, vq_effective - vq_target);
	  }
}



