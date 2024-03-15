#ifndef _CHASSIS_TASK_H__
#define _CHASSIS_TASK_H__

#include "stdint.h"

#include "remote_control.h"

#include "dev_motor.h"

#include "algo_filter.h"

#include "kine_stable_chassis.h"

#include "INS_task.h"

typedef enum
{
    CHASSIS_MODE_NO_FORCE, // 无力, 跟没上电一样
    CHASSIS_MODE_STOP,     // 待着不动，锁死
    CHASSIS_MODE_FOLLOW,   // 跟随控制
} chassis_mode_e;

typedef struct EngineerChassis
{
    rfl_chassis_s model;
    chassis_mode_e mode;

    const INS_t *ins;
    rfl_angle_s yaw;
    float wheel_speed[4];

    const RC_ctrl_t *rc;                    /*遥控器链路控制数据*/
    ramp_function_source_t speed_ramper[3]; /*速度斜坡化滤波器*/
    float set_speed_vector[3];              /*底盘预期速度vx vy wz*/
    rfl_angle_s set_angle;
    float *wheel_set_speed;

    rfl_motor_s motor[4];

} engineer_chassis_s;

extern void chassis_task(void *pvParameters);

extern engineer_chassis_s *getChassisDataPointer(void);

#define CHASSIS_RC_DEADLINE (3)
#define CHASSIS_VX_RC_CONTROL_MAX (3.0f)
#define CHASSIS_VY_RC_CONTROL_MAX (3.0f)
#define CHASSIS_WZ_RC_CONTROL_MAX (3.0f)

#define WHEEL_RADIUS (0.0765f)

#define YAW_CONTROL_SEN (0.1f)

#define MOTORS_CAN_ORDINAL (3)
#define MOTORS_CAN_SLAVE_ID (0x200)

#endif /* _CHASSIS_TASK_H__ */
