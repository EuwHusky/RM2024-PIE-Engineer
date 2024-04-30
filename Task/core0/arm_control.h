#ifndef _ARM_CONTROL_H__
#define _ARM_CONTROL_H__

#include "stdint.h"

#include "arm_task.h"

/**
 * @brief 根据机械臂控制模式 配置电机控制模式 更新机械臂控制量
 */
extern void arm_mode_control(engineer_scara_arm_s *scara_arm);

/* ================================================= 手动控制参数 ================================================= */

#if USE_JOINTS_CONTROL
#define JOINT_1_CONTROL_SEN (0.0002f)
#define JOINT_2_CONTROL_SEN (0.04f)
#define JOINT_3_CONTROL_SEN (0.04f)
#define JOINT_4_CONTROL_SEN (0.04f)
#define JOINT_5_CONTROL_SEN (0.04f)
#define JOINT_6_CONTROL_SEN (0.04f)
#endif

#define POSE_X_CONTROL_SEN (0.001f)
#define POSE_Y_CONTROL_SEN (0.001f)
#define POSE_Z_CONTROL_SEN (0.001f)
#define POSE_AY_CONTROL_SEN (0.005f)
#define POSE_AP_CONTROL_SEN (0.005f)
#define POSE_AR_CONTROL_SEN (0.005f)

#define SILVER_MINING_X_CONTROL_SEN (0.0002f)
#define SILVER_MINING_Y_CONTROL_SEN (0.0002f)
#define SILVER_MINING_Z_CONTROL_SEN (0.0002f)
#define SILVER_MINING_AY_CONTROL_SEN (0.0006f)
#define SILVER_MINING_AP_CONTROL_SEN (0.0006f)
#define SILVER_MINING_AR_CONTROL_SEN (0.0006f)

/* =============================================== 自动控制 通用参数 =============================================== */

/**
 * @brief 距离最小允许偏差 控制距离时预期值与测量值相差小于此值时可认为控制已达到预期
 * @单位 米 m
 */
#define TOLERABLE_DISTANCE_DEVIATION (0.004f)

/**
 * @brief 角度最小允许偏差 控制角度时预期值与测量值相差小于此值时可认为控制已达到预期
 * @单位 弧度 radian
 */
#define TOLERABLE_ANGLE_DEVIATION (0.04f)

/**
 * @brief 距离偏离检测阈值 控制距离时预期值与测量值相差大于此值时可认为控制已偏离预期
 * @单位 米 m
 */
#define DISTANCE_DEVIATION_THRESHOLD (0.01f)

/**
 * @brief 角度偏离检测阈值 控制角度时预期值与测量值相差大于此值时可认为控制已偏离预期
 * @单位 弧度 radian
 */
#define ANGLE_DEVIATION_THRESHOLD (0.1f)

/**
 * @brief 作业模式默认抬升高度
 * @note 此值将影响自动取矿辅助UI的位置 修改此项后需要重新调整相关UI的参数
 */
#define OPERATION_MODE_DEFAULT_Z (ENGINEER_ARM_Z_MAX_DISTANCE / 5.0f)

/* =============================================== 自动控制 复位参数 =============================================== */

#define JOINT_1_HOMING_STEP_ANGLE (0.01f)
#define JOINT_1_HOMING_TORQUE_THRESHOLD (2.0f)
#define JOINT_1_HOMING_ANGLE (-13.021768f)

#define JOINT_2_START_ANGLE (0.0f)

#define JOINT_3_START_ANGLE (0.0f)

#define JOINT_4_HOMING_STEP_ANGLE (0.005f)
#define JOINT_4_HOMING_TORQUE_THRESHOLD (1.5f)
#define JOINT_4_HOMING_ANGLE (-124.480934f)
#define JOINT_4_START_ANGLE (0.0f)

#define JOINT_5_HOMING_STEP_ANGLE (0.005f)
#define JOINT_5_HOMING_TORQUE_THRESHOLD (5.0f)
#define JOINT_5_HOMING_ANGLE (-85.0f)
#define JOINT_5_START_ANGLE (0.0f)

#define JOINT_6_START_ANGLE (0.0f)

/* =============================================== 自动控制 银矿参数 =============================================== */

#define SILVER_MINING_STEP_INIT (0u)
#define SILVER_MINING_STEP_READY (1u)
#define SILVER_MINING_STEP_GRAB (2u)
#define SILVER_MINING_STEP_WAIT (3u)
#define SILVER_MINING_STEP_OK (4u)

/* =============================================== 自动控制 存矿参数 =============================================== */

#define STORAGE_PUSH_STEP_INIT (0u)
#define STORAGE_PUSH_STEP_START (1u)
#define STORAGE_PUSH_STEP_SLOT (2u)
#define STORAGE_PUSH_STEP_PUSH_IN (3u)

/* =============================================== 自动控制 取矿参数 =============================================== */

#define STORAGE_POP_STEP_INIT (0u)
#define STORAGE_POP_STEP_START (1u)
#define STORAGE_POP_STEP_SLOT (2u)
#define STORAGE_POP_STEP_POP_OUT (3u)

#endif /* _ARM_CONTROL_H__ */
