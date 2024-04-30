#ifndef _GIMBAL_TASK_H__
#define _GIMBAL_TASK_H__

#include "stdbool.h"
#include "stdint.h"

#include "dev_motor.h"

#include "remote_control.h"

#include "behavior_task.h"

typedef struct EngineerGimbal
{
    engineer_behavior_e behavior;
    engineer_behavior_e last_behavior;

    uint8_t reset_step;
    bool reset_success;

    rfl_angle_s set_gimbal_angle;

    rfl_motor_s yaw_motor;

    uint32_t pitch_pwm_clk_freq;
    uint32_t pitch_pwm_freq;
    uint32_t pitch_pwm_reload;
    uint32_t pitch_pwm_compare;

    const remote_control_s *rc;

} engineer_gimbal_s;

extern void gimbal_task(void *pvParameters);
extern const engineer_gimbal_s *getGimbalDataPointer(void);

extern bool *getGimbalResetStatus(void);
extern float getGimbalYawAngle(rfl_angle_format_e angle_format);

#define ENGINEER_GIMBAL_RESET_STEP_HOMING (0u)
#define ENGINEER_GIMBAL_RESET_STEP_STARTING (1u)

#define GIMBAL_YAW_HOMING_STEP_ANGLE (1.0f)
#define GIMBAL_YAW_HOMING_TORQUE_THRESHOLD (3.6f)
#define GIMBAL_YAW_HOMING_ANGLE (70.0f)
#define GIMBAL_YAW_START_ANGLE (-90.0f)

#define ENGINEER_MOVE_BEHAVIOR_GIMBAL_SET_ANGLE (GIMBAL_YAW_START_ANGLE)
#define ENGINEER_OPERATION_BEHAVIOR_GIMBAL_SET_ANGLE (0.0f)

#define ENGINEER_GIMBAL_YAW_MAX_ANGLE (GIMBAL_YAW_HOMING_ANGLE)
#define ENGINEER_GIMBAL_YAW_MIN_ANGLE (GIMBAL_YAW_START_ANGLE)
#define ENGINEER_GIMBAL_YAW_INITIAL_MAX_ANGLE (360.0f)
#define ENGINEER_GIMBAL_YAW_INITIAL_MIN_ANGLE (-360.0f)

#define GIMBAL_MOTORS_CAN_ORDINAL (4)
#define GIMBAL_MOTORS_CAN_SLAVE_ID (0x200)

#define ENGINEER_GIMBAL_YAW_RM_M2006_ANGLE_PID_KP (0.72f)
#define ENGINEER_GIMBAL_YAW_RM_M2006_ANGLE_PID_KI (0.0f)
#define ENGINEER_GIMBAL_YAW_RM_M2006_ANGLE_PID_KD (0.0072f)
#define ENGINEER_GIMBAL_YAW_RM_M2006_ANGLE_PID_MAX_IOUT (0.0f)
#define ENGINEER_GIMBAL_YAW_RM_M2006_ANGLE_PID_MAX_OUT (16.0f)
#define ENGINEER_GIMBAL_YAW_RM_M2006_SPEED_PID_KP (1500.0f)
#define ENGINEER_GIMBAL_YAW_RM_M2006_SPEED_PID_KI (1.0f)
#define ENGINEER_GIMBAL_YAW_RM_M2006_SPEED_PID_KD (0.0f)
#define ENGINEER_GIMBAL_YAW_RM_M2006_SPEED_PID_MAX_IOUT (4000.0f)
#define ENGINEER_GIMBAL_YAW_RM_M2006_SPEED_PID_MAX_OUT (10000.0f)

#define ENGINEER_GIMBAL_PITCH_PWM_CONPARE_VALUE_MAX (359582u)
#define ENGINEER_GIMBAL_PITCH_PWM_CONPARE_VALUE_MIN (270125u)
#define ENGINEER_GIMBAL_PITCH_PWM_CONPARE_VALUE_MID (307604u)

#endif /* _GIMBAL_TASK_H__ */
