#ifndef _ARM_MOTOR_H__
#define _ARM_MOTOR_H__

#include "arm_task.h"

/**
 * @brief 初始化机械臂电机
 */
extern void arm_motor_init(engineer_scara_arm_s *scara_arm);

/**
 * @brief 更新机械臂电机状态量并执行控制
 */
extern void arm_motor_update_and_execute(engineer_scara_arm_s *scara_arm);

/**
 * @brief 设置机械臂电机控制模式
 */
extern void arm_motor_set_mode(engineer_scara_arm_s *scara_arm, rfl_motor_control_mode_e mode);

/**
 * @brief 设置机械臂电机运动速度
 * @note 测试版 后续需要更新为动态速度规划
 */
extern void arm_motor_set_max_speed(engineer_scara_arm_s *scara_arm, float max_speed);

/**
 * @brief 设置机械臂电机可达角度
 */
extern void arm_motor_set_angle_limit(engineer_scara_arm_s *scara_arm, engineer_scara_arm_mode_e mode);

#endif /* _ARM_MOTOR_H__ */
