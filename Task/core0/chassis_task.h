#ifndef _CHASSIS_TASK_H__
#define _CHASSIS_TASK_H__

#include "stdint.h"

#include "dev_motor.h"

#include "algo_filter.h"

#include "kine_stable_chassis.h"

#include "remote_control.h"

#include "INS_task.h"
#include "behavior_task.h"

typedef struct EngineerChassis
{
    bool started;

    engineer_behavior_e behavior;
    engineer_behavior_e last_behavior;

    rfl_chassis_s model;

    const INS_t *ins;
    rfl_angle_s yaw;
    float wheel_speed[4];

    float lidar_obstacle_distance;
    float lidar_obstacle_surface_angle;

    const remote_control_s *rc;             // 遥控器数据
    ramp_function_source_t speed_ramper[3]; // 速度斜坡化滤波器
    float set_speed_vector[3];              // 底盘预期速度vx vy wz
    rfl_angle_s set_control_angle;
    rfl_angle_s set_control_angle_memory;
    float *wheel_set_speed;
    float follow_offset;

    rfl_motor_s motor[4];

} engineer_chassis_s;

extern void chassis_task(void *pvParameters);
extern engineer_chassis_s *getChassisDataPointer(void);

extern bool *getChassisStartedFlag(void);

extern float getChassisFollowOffsetMemory(void);

#define CHASSIS_CONTROL_TIME (0.002f)

#define CHASSIS_VX_MAX (3.0f)
#define CHASSIS_VY_MAX (2.0f)
#define CHASSIS_WZ_MAX (RAD_PI)

#define CHASSIS_WHEEL_RADIUS (0.0765f)

#define CHASSIS_MOTORS_CAN_ORDINAL (3)
#define CHASSIS_MOTORS_CAN_SLAVE_ID (0x200)

#define ENGINEER_CHASSIS_RM_M3508_SPEED_PID_KP (2400.0f)
#define ENGINEER_CHASSIS_RM_M3508_SPEED_PID_KI (24.0f)
#define ENGINEER_CHASSIS_RM_M3508_SPEED_PID_KD (0.0f)
#define ENGINEER_CHASSIS_RM_M3508_SPEED_PID_MAX_IOUT (10000.0f)
#define ENGINEER_CHASSIS_RM_M3508_SPEED_PID_MAX_OUT (16000.0f)

/**
 * @brief 激光雷达发射面到底盘边沿距离+底盘边沿到小资源岛距离
 */
#define SILVER_MINING_X_DISTANCE_OFFSET (0.394f)
/**
 * @brief 激光雷达测得小资源岛角度平行补偿
 */
#define SILVER_MINING_YAW_ANGLE_OFFSET (1.15f)

#define SILVER_MINING_X_ALIGN_KP (3.0f)
#define SILVER_MINING_YAW_ALIGN_KP (0.006f)

#endif /* _CHASSIS_TASK_H__ */
