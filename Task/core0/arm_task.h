#ifndef _ARM_TASK_H__
#define _ARM_TASK_H__

#include "stdbool.h"
#include "stdint.h"

#include "board.h"

#include "dev_motor.h"

#include "algo_filter.h"
#include "algo_matrix.h"

#include "referee.h"
#include "remote_control.h"

#include "behavior_task.h"

#define USE_JOINTS_CONTROL 0

typedef enum EngineerScaraArmJoints
{
    JOINT_1 = 0,
    JOINT_2,
    JOINT_3,
    JOINT_4,
    JOINT_5,
    JOINT_6,
} engineer_scara_arm_joints_e;

typedef enum EngineerScaraArmStartUpStatus
{
    ARM_START_UP_OK = 0x00,

    ARM_START_UP_JOINT_1_FAILED = 0x01, // 归位失败
    ARM_START_UP_JOINT_2_FAILED = 0x02, // 归位失败
    ARM_START_UP_JOINT_3_FAILED = 0x04, // 归位失败
    ARM_START_UP_JOINT_4_FAILED = 0x08, // 归位失败
    ARM_START_UP_JOINT_5_FAILED = 0x10, // 归位失败
    ARM_START_UP_JOINT_6_FAILED = 0x20, // 归位失败

    ARM_START_UP_JOINT_4_STEP1_FAILED = 0x40, // 机械限位获取角度失败
    ARM_START_UP_JOINT_5_STEP1_FAILED = 0x80, // 机械限位获取角度失败

    ARM_NOT_START_UP = 0xff,
} engineer_scara_arm_start_up_status_e;

#define JOINT45_START_UP_STEP1_BIT_OFFSET 3

typedef enum EngineerScaraArmJointsMotorsIndex
{
    MOTOR_JOINT1_LEFT = 0,
    MOTOR_JOINT1_RIGHT,
    MOTOR_JOINT23_BACK,
    MOTOR_JOINT23_FRONT,
    MOTOR_JOINT4,
    MOTOR_JOINT56_LEFT,
    MOTOR_JOINT56_RIGHT,
} engineer_scara_arm_joints_motors_index_e;

typedef enum EngineerScaraArmSolution
{
    JOINT_3_ON_THE_LEFT = 0,
    JOINT_3_ON_THE_RIGHT,
} engineer_scara_arm_solution_e;

/**
 * @brief 清除机械臂启动状态
 * @param[out] arm_start_up_status 机械臂状态簇
 */
#define resetArmStartUpStatus(arm_start_up_status) (arm_start_up_status = ARM_NOT_START_UP)

/**
 * @brief 设定机械臂某一关节启动状态为OK
 * @param[in] joint_index 机械臂关节号
 * @param[out] arm_start_up_status 机械臂状态簇
 */
#define setJointStartUpStateOk(joint_index, arm_start_up_status)                                                       \
    (scara_arm->start_up_status &= (~((1 << (joint_index)) & 0xff)))

/**
 * @brief 获取机械臂某一关节的启动状态 若未成功启动则返回非零
 * @param[in] joint_index 机械臂关节号
 * @param[in] arm_start_up_status 机械臂状态簇
 */
#define checkIfJointNotStartUp(joint_index, arm_start_up_status) ((((arm_start_up_status) >> (joint_index)) & 1) != 0)

typedef struct EngineerScaraArm
{
    /*基础*/

    engineer_behavior_e behavior;
    engineer_behavior_e last_behavior;

    uint8_t start_up_status;
    bool reset_success;
    bool move_homing_success;
    bool operation_homing_success;
    uint8_t silver_mining_step;
    bool silver_mining_success;

    uint8_t joint_1_homing_timer;

    /*运动学模型*/

    rfl_matrix_instance j1_to_base_tmat; /*关节1坐标系到基坐标系变换矩阵*/
    float j1_to_base_tmat_data[16];
    rfl_matrix_instance tool_to_j6_tmat; /*工具坐标系到关节6坐标系变换矩阵*/
    float tool_to_j6_tmat_data[16];
    rfl_matrix_instance tool_to_base_tmat; /*工具坐标系到基坐标系变换矩阵*/
    float tool_to_base_tmat_data[16];
    /**
     * @brief   Scara构型机械臂DH参数
     * [0:5]    Joint 1-6
     * [x][0]   alpha
     * [x][1]   a
     * [x][2]   d
     * [x][3]   theta
     * [x][4]   theta_offset
     */
    float dh[6][5];

    /*状态量*/

    /**
     * @brief   工具坐标系在基坐标系六自由度位姿 距离单位 m 角度单位 rad
     * [0:5]    X Y Z YAW PITCH ROLL
     */
    float pose_6d[6];
    /**
     * @brief   关节变量 距离单位 m 角度单位 rad
     * [0:5]    J1-distance J2-angle J3-angle J4-angle J5-angle J6-angle
     */
    float joints_value[6];

    /*控制量*/

    engineer_scara_arm_solution_e solution; /*逆运动学多解选择*/
    /**
     * @brief   预期的工具坐标系在基坐标系六自由度位姿 距离单位 m 角度单位 rad
     * [0:5]    X Y Z YAW PITCH ROLL
     */
    float set_pose_6d[6];
    /**
     * @brief   逆运动学解算得到的期望关节变量 距离单位 m 角度单位 rad
     * [0:5]    J1-distance J2-angle J3-angle J4-angle J5-angle J6-angle
     */
    float set_joints_value[6];

    /*设备*/

    const remote_control_s *rc; // 遥控器数据

    const custom_robot_data_t *customer_controller; // 自定义控制器数据
    float cc_pose_6d[6];
    first_order_filter_type_t cc_pose_filter[6];
    float local_pos_memory[3]; // 机械臂位置记忆
    float cc_pos_memory[3];    // 自定义控制器位置记忆

    // uint16_t joint_6_encoder_value;
    // float joint_6_encoder_angle; // 磁编获取到的关节6的绝对角度，逆时针为正，单位为角度deg
    // sliding_window_filter_s_t joint_6_encoder_angle_filter;

    rfl_motor_s joints_motors[7];

    float printer[6]; // 只是方便把数发出来

} engineer_scara_arm_s;

extern void arm_task(void *pvParameters);
extern engineer_scara_arm_s *getArmDataPointer(void);

extern bool *getArmResetStatus(void);
extern bool *getArmMoveHomingStatue(void);
extern bool *getArmOperationHomingStatus(void);
extern bool *getSilverMiningStatus(void);

/* 机械臂结构参数 */

#define ENGINEER_ARM_1_LENGTH (0.276f)  /*第一节小臂臂长*/
#define ENGINEER_ARM_2_LENGTH (0.3015f) /*第二节小臂臂长*/
#define ENGINEER_ARM_3_LENGTH (0.07f)   /*第三节小臂臂长*/
#define ENGINEER_ARM_4_LENGTH (0.055f)  /*第四节小臂臂长*/

/**
 * @brief 抬升链轮半径 单位 mm
 */
#define LIFTER_SPROCKET_RADIUS (44.0f)
/**
 * @brief 抬升距离到电机转角转换系数
 * @note 算式 ：distance = degree * DEGREE_TO_RADIAN_FACTOR * LIFTER_SPROCKET_RADIUS
 */
#define LIFTER_DISTANCE_TO_DEGREE_FACTOR (1302.1768071155072926545035185024f)
/**
 * @brief 关节2 驱动源到执行器减速比
 */
#define JOINT2_REDUCTION (1.8f)
/**
 * @brief 末端传动齿轮减速比
 * @note 直接在电机初始化时将此减速比置入了 故电机角度并非出轴角度而是减速后的齿轮的角度
 */
#define END_TRANSMISSION_GEAR_REDUCTION (21.0f / 17.0f)
/**
 * @brief 末端锥型齿轮组减速比
 */
#define END_BEVEL_GEAR_SET_REDUCTION (2.0f)

/* 机械臂模型参数 */

// 位姿可达范围相关参数
#define ENGINEER_ARM_Z_MAX_DISTANCE (0.625f)                                           /*关节1的最大伸展距离*/
#define ENGINEER_ARM_Z_MIN_DISTANCE (0.0f)                                             /*关节1最小伸展距离*/
#define ENGINEER_ARM_XY24_MAX_DISTANCE (ENGINEER_ARM_1_LENGTH + ENGINEER_ARM_2_LENGTH) /*关节2到关节4的最大伸展距离*/
#define ENGINEER_ARM_YAW_MAX_ANGLE (180.0f)                                            /* 末端YAW最大角度 */
#define ENGINEER_ARM_YAW_MIN_ANGLE (-180.0f)                                           /* 末端YAW最小角度 */
#define ENGINEER_ARM_PITCH_MAX_ANGLE (48.0f)                                           /* 末端PITCH最大角度 */
#define ENGINEER_ARM_PITCH_MIN_ANGLE (-75.0f)                                          /* 末端PITCH最小角度 */
#define ENGINEER_ARM_ROLL_MAX_ANGLE (720.0f)                                           /* 末端ROLL最大角度 */
#define ENGINEER_ARM_ROLL_MIN_ANGLE (-720.0f)                                          /* 末端ROLL最小角度 */

// 关节运行基准速度
#define ENGINEER_ARM_MANUAL_OPERATION_BASE_SPEED (2.0f)
#define ENGINEER_ARM_AUTO_OPERATION_BASE_SPEED (1.0f)

// 正常运行时的关节可达范围
#define ENGINEER_ARM_JOINT_1_MAX_DISTANCE (ENGINEER_ARM_Z_MAX_DISTANCE)
#define ENGINEER_ARM_JOINT_1_MIN_DISTANCE (ENGINEER_ARM_Z_MIN_DISTANCE)
#define ENGINEER_ARM_JOINT_2_MAX_ANGLE (90.0f)
#define ENGINEER_ARM_JOINT_2_MIN_ANGLE (-90.0f)
#define ENGINEER_ARM_JOINT_3_MAX_ANGLE (160.0f)
#define ENGINEER_ARM_JOINT_3_MIN_ANGLE (-160.0f)
#define ENGINEER_ARM_JOINT_4_MAX_ANGLE (125.0f)
#define ENGINEER_ARM_JOINT_4_MIN_ANGLE (-125.0f)
#define ENGINEER_ARM_JOINT_5_MAX_ANGLE (ENGINEER_ARM_PITCH_MAX_ANGLE)
#define ENGINEER_ARM_JOINT_5_MIN_ANGLE (ENGINEER_ARM_PITCH_MIN_ANGLE)
#define ENGINEER_ARM_JOINT_6_MAX_ANGLE (ENGINEER_ARM_ROLL_MAX_ANGLE)
#define ENGINEER_ARM_JOINT_6_MIN_ANGLE (ENGINEER_ARM_ROLL_MIN_ANGLE)

// 关节初始化扩展可达范围 用于启动操作 有一定的危险性
#define ENGINEER_ARM_JOINT_1_INITIAL_MAX_DISTANCE (ENGINEER_ARM_JOINT_1_MAX_DISTANCE)
#define ENGINEER_ARM_JOINT_1_INITIAL_MIN_DISTANCE (-ENGINEER_ARM_JOINT_1_MAX_DISTANCE)
#define ENGINEER_ARM_JOINT_4_INITIAL_MAX_ANGLE (ENGINEER_ARM_JOINT_4_MAX_ANGLE * 2.0f)
#define ENGINEER_ARM_JOINT_4_INITIAL_MIN_ANGLE (ENGINEER_ARM_JOINT_4_MIN_ANGLE * 2.0f)
#define ENGINEER_ARM_JOINT_5_INITIAL_MAX_ANGLE (ENGINEER_ARM_JOINT_5_MAX_ANGLE - ENGINEER_ARM_JOINT_5_MIN_ANGLE)
#define ENGINEER_ARM_JOINT_5_INITIAL_MIN_ANGLE (ENGINEER_ARM_JOINT_5_MIN_ANGLE - ENGINEER_ARM_JOINT_5_MAX_ANGLE)

/* 机械臂设备参数 */

// 关节驱动电机运动范围
#define ENGINEER_ARM_MOTOR_JOINT_1_MAX_ANGLE (ENGINEER_ARM_JOINT_1_MAX_DISTANCE * LIFTER_DISTANCE_TO_DEGREE_FACTOR)
#define ENGINEER_ARM_MOTOR_JOINT_1_MIN_ANGLE (ENGINEER_ARM_JOINT_1_MIN_DISTANCE * LIFTER_DISTANCE_TO_DEGREE_FACTOR)
#define ENGINEER_ARM_MOTOR_JOINT_23_BACK_MAX_ANGLE (ENGINEER_ARM_JOINT_2_MAX_ANGLE * JOINT2_REDUCTION)
#define ENGINEER_ARM_MOTOR_JOINT_23_BACK_MIN_ANGLE (ENGINEER_ARM_JOINT_2_MIN_ANGLE * JOINT2_REDUCTION)
#define ENGINEER_ARM_MOTOR_JOINT_23_FRONT_MAX_ANGLE (173.0f)
#define ENGINEER_ARM_MOTOR_JOINT_23_FRONT_MIN_ANGLE (-173.0f)
#define ENGINEER_ARM_MOTOR_JOINT_4_MAX_ANGLE (ENGINEER_ARM_JOINT_4_MAX_ANGLE)
#define ENGINEER_ARM_MOTOR_JOINT_4_MIN_ANGLE (ENGINEER_ARM_JOINT_4_MIN_ANGLE)
#define ENGINEER_ARM_MOTOR_JOINT_56_MAX_ANGLE                                                                          \
    (ENGINEER_ARM_JOINT_5_MAX_ANGLE + ENGINEER_ARM_JOINT_6_MAX_ANGLE * END_BEVEL_GEAR_SET_REDUCTION)
#define ENGINEER_ARM_MOTOR_JOINT_56_MIN_ANGLE                                                                          \
    (ENGINEER_ARM_JOINT_5_MIN_ANGLE + ENGINEER_ARM_JOINT_6_MIN_ANGLE * END_BEVEL_GEAR_SET_REDUCTION)

// 关节驱动电机初始化扩展运动范围 用于启动操作 有一定的危险性
#define ENGINEER_ARM_JOINT_1_MOTOR_INITIAL_MAX_ANGLE                                                                   \
    (ENGINEER_ARM_JOINT_1_INITIAL_MAX_DISTANCE * LIFTER_DISTANCE_TO_DEGREE_FACTOR)
#define ENGINEER_ARM_JOINT_1_MOTOR_INITIAL_MIN_ANGLE                                                                   \
    (ENGINEER_ARM_JOINT_1_INITIAL_MIN_DISTANCE * LIFTER_DISTANCE_TO_DEGREE_FACTOR)
#define ENGINEER_ARM_JOINT_4_MOTOR_INITIAL_MAX_ANGLE (ENGINEER_ARM_JOINT_4_INITIAL_MAX_ANGLE)
#define ENGINEER_ARM_JOINT_4_MOTOR_INITIAL_MIN_ANGLE (ENGINEER_ARM_JOINT_4_INITIAL_MIN_ANGLE)
#define ENGINEER_ARM_JOINT_56_MOTOR_INITIAL_MAX_ANGLE (ENGINEER_ARM_JOINT_5_INITIAL_MAX_ANGLE * 2.0f)
#define ENGINEER_ARM_JOINT_56_MOTOR_INITIAL_MIN_ANGLE (ENGINEER_ARM_JOINT_5_INITIAL_MIN_ANGLE * 2.0f)

// 电机控制器参数
#define ENGINEER_ARM_JOINT_1_RM_M3508_ANGLE_PID_KP (1.5f)
#define ENGINEER_ARM_JOINT_1_RM_M3508_ANGLE_PID_KI (0.0f)
#define ENGINEER_ARM_JOINT_1_RM_M3508_ANGLE_PID_KD (0.1f)
#define ENGINEER_ARM_JOINT_1_RM_M3508_ANGLE_PID_MAX_IOUT (0.0f)
#define ENGINEER_ARM_JOINT_1_RM_M3508_ANGLE_PID_MAX_OUT (16.0f)
#define ENGINEER_ARM_JOINT_1_RM_M3508_SPEED_PID_KP (1000.0f)
#define ENGINEER_ARM_JOINT_1_RM_M3508_SPEED_PID_KI (8.0f)
#define ENGINEER_ARM_JOINT_1_RM_M3508_SPEED_PID_KD (0.0f)
#define ENGINEER_ARM_JOINT_1_RM_M3508_SPEED_PID_MAX_IOUT (8000.0f)
#define ENGINEER_ARM_JOINT_1_RM_M3508_SPEED_PID_MAX_OUT (16000.0f)

#define ENGINEER_ARM_JOINT_4_RM_M3508_ANGLE_PID_KP (0.8f)
#define ENGINEER_ARM_JOINT_4_RM_M3508_ANGLE_PID_KI (0.0f)
#define ENGINEER_ARM_JOINT_4_RM_M3508_ANGLE_PID_KD (0.08f)
#define ENGINEER_ARM_JOINT_4_RM_M3508_ANGLE_PID_MAX_IOUT (0.0f)
#define ENGINEER_ARM_JOINT_4_RM_M3508_ANGLE_PID_MAX_OUT (16.0f)
#define ENGINEER_ARM_JOINT_4_RM_M3508_SPEED_PID_KP (800.0f)
#define ENGINEER_ARM_JOINT_4_RM_M3508_SPEED_PID_KI (1.0f)
#define ENGINEER_ARM_JOINT_4_RM_M3508_SPEED_PID_KD (0.0f)
#define ENGINEER_ARM_JOINT_4_RM_M3508_SPEED_PID_MAX_IOUT (4000.0f)
#define ENGINEER_ARM_JOINT_4_RM_M3508_SPEED_PID_MAX_OUT (16000.0f)

#define ENGINEER_ARM_JOINT_56_RM_M2006_ANGLE_PID_KP (0.8f)
#define ENGINEER_ARM_JOINT_56_RM_M2006_ANGLE_PID_KI (0.0f)
#define ENGINEER_ARM_JOINT_56_RM_M2006_ANGLE_PID_KD (0.006f)
#define ENGINEER_ARM_JOINT_56_RM_M2006_ANGLE_PID_MAX_IOUT (0.0f)
#define ENGINEER_ARM_JOINT_56_RM_M2006_ANGLE_PID_MAX_OUT (16.0f)
#define ENGINEER_ARM_JOINT_56_RM_M2006_SPEED_PID_KP (800.0f)
#define ENGINEER_ARM_JOINT_56_RM_M2006_SPEED_PID_KI (1.0f)
#define ENGINEER_ARM_JOINT_56_RM_M2006_SPEED_PID_KD (0.0f)
#define ENGINEER_ARM_JOINT_56_RM_M2006_SPEED_PID_MAX_IOUT (2400.0f)
#define ENGINEER_ARM_JOINT_56_RM_M2006_SPEED_PID_MAX_OUT (10000.0f)

// CAN通信
#define ENGINEER_ARM_JOINTS_123_MOTORS_CAN_ORDINAL (1)
#define ENGINEER_ARM_JOINTS_123_RM_MOTORS_CAN_SLAVE_ID (0x200)
#define ENGINEER_ARM_JOINTS_456_MOTORS_CAN_ORDINAL (2)
#define ENGINEER_ARM_JOINTS_456_RM_MOTORS_CAN_SLAVE_ID (0x200)

// 磁编码器安装偏差
#define ENGINEER_ARM_JOINT_6_ENCODER_ANGLE_OFFSET (0.0f)

#endif /* _ARM_TASK_H__ */
