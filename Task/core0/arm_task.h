#ifndef _ARM_TASK_H__
#define _ARM_TASK_H__

#include "stdint.h"

#include "board.h"

#include "dev_motor.h"

#include "algo_filter.h"
#include "algo_matrix.h"

#include "referee.h"
#include "remote_control.h"

typedef enum EngineerScaraArmMode
{
    ARM_MODE_NO_FORCE, // 无力 跟没上电一样
    ARM_MODE_START_UP, // 启动 各个关节全部重新获取初始角度
    ARM_MODE_JOINTS,   // 关节控制 既挖掘机式控制
    ARM_MODE_POSE,     // 位姿控制
    ARM_MODE_CUSTOMER, // 自定义控制器控制
} engineer_scara_arm_mode_e;

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
    /*自身属性*/

    engineer_scara_arm_mode_e mode;
    engineer_scara_arm_mode_e last_mode;
    uint8_t start_up_status;

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

    bool use_normal_solution; /*使用默认解 默认解下关节3角度为正*/
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

    const RC_ctrl_t *dbus_rc;                  // dt7-dr16遥控器链路 数据
    const custom_robot_data_t *vt_customer_rc; // 图传链路 自定义控制器数据
    const remote_control_t *vt_mk;             // 图传链路 键鼠数据
    char last_mode_control_key_value;

    uint16_t joint_6_encoder_value;
    float joint_6_encoder_angle; // 磁编获取到的关节6的绝对角度，逆时针为正，单位为角度deg
    sliding_window_filter_s_t joint_6_encoder_angle_filter;

    rfl_motor_s joints_motors[7];

} engineer_scara_arm_s;

extern engineer_scara_arm_s scara_arm;

extern void arm_task(void *pvParameters);
extern engineer_scara_arm_s *getArmDataPointer(void);

/* 机械臂结构参数 */

#define ENGINEER_ARM_1_LENGTH (0.276f)  /*第一节小臂臂长*/
#define ENGINEER_ARM_2_LENGTH (0.3015f) /*第二节小臂臂长*/
#define ENGINEER_ARM_3_LENGTH (0.07f)   /*第三节小臂臂长*/
#define ENGINEER_ARM_4_LENGTH (0.058f)  /*第四节小臂臂长*/

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
 */
#define END_TRANSMISSION_GEAR_REDUCTION (1.2f)
/**
 * @brief 末端锥型齿轮组减速比
 */
#define END_BEVEL_GEAR_SET_REDUCTION (2.0f)

/* 机械臂模型参数 */

// 位姿可达范围
#define ENGINEER_ARM_XY24_MAX_DISTANCE (ENGINEER_ARM_1_LENGTH + ENGINEER_ARM_2_LENGTH) /*关节2到关节4的最大伸展距离*/
#define ENGINEER_ARM_XY24_MIN_DISTANCE (0.1f) /*关节2到关节4的最小伸展距离*/
#define ENGINEER_ARM_PITCH_MAX_ANGLE (60.0f)  /* 末端PITCH最大角度 */
#define ENGINEER_ARM_PITCH_MIN_ANGLE (-60.0f) /* 末端PITCH最小角度 */
#define ENGINEER_ARM_ROLL_MAX_ANGLE (180.0f)  /* 末端ROLL最大角度 */
#define ENGINEER_ARM_ROLL_MIN_ANGLE (-180.0f) /* 末端ROLL最小角度 */

// 正常运行时的关节可达范围
#define ENGINEER_ARM_JOINT_1_MAX_DISTANCE (0.44f)
#define ENGINEER_ARM_JOINT_1_MIN_DISTANCE (0.0f)
#define ENGINEER_ARM_JOINT_2_MAX_ANGLE (90.0f)
#define ENGINEER_ARM_JOINT_2_MIN_ANGLE (-90.0f)
#define ENGINEER_ARM_JOINT_3_MAX_ANGLE (160.0f)
#define ENGINEER_ARM_JOINT_3_MIN_ANGLE (-160.0f)
#define ENGINEER_ARM_JOINT_4_MAX_ANGLE (90.0f)
#define ENGINEER_ARM_JOINT_4_MIN_ANGLE (-90.0f)
#define ENGINEER_ARM_JOINT_5_MAX_ANGLE (ENGINEER_ARM_PITCH_MAX_ANGLE)
#define ENGINEER_ARM_JOINT_5_MIN_ANGLE (ENGINEER_ARM_PITCH_MIN_ANGLE)
#define ENGINEER_ARM_JOINT_6_MAX_ANGLE (ENGINEER_ARM_ROLL_MAX_ANGLE)
#define ENGINEER_ARM_JOINT_6_MIN_ANGLE (ENGINEER_ARM_ROLL_MIN_ANGLE)

// 关节初始化扩展可达范围 用于启动操作 有一定的危险性
#define ENGINEER_ARM_JOINT_1_INITIAL_MAX_DISTANCE (ENGINEER_ARM_JOINT_1_MAX_DISTANCE)
#define ENGINEER_ARM_JOINT_1_INITIAL_MIN_DISTANCE (-ENGINEER_ARM_JOINT_1_MAX_DISTANCE)

/* 机械臂设备参数 */

// CAN通信
#define ENGINEER_ARM_JOINTS_123_MOTORS_CAN_ORDINAL (1)
#define ENGINEER_ARM_JOINTS_123_RM_MOTORS_CAN_SLAVE_ID (0x200)
#define ENGINEER_ARM_JOINTS_456_MOTORS_CAN_ORDINAL (2)
#define ENGINEER_ARM_JOINTS_456_RM_MOTORS_CAN_SLAVE_ID (0x200)

// 关节驱动电机运动范围
#define ENGINEER_ARM_MOTOR_JOINT_1_MAX_ANGLE (ENGINEER_ARM_JOINT_1_MAX_DISTANCE * LIFTER_DISTANCE_TO_DEGREE_FACTOR)
#define ENGINEER_ARM_MOTOR_JOINT_1_MIN_ANGLE (ENGINEER_ARM_JOINT_1_MIN_DISTANCE * LIFTER_DISTANCE_TO_DEGREE_FACTOR)
#define ENGINEER_ARM_MOTOR_JOINT_23_BACK_MAX_ANGLE (ENGINEER_ARM_JOINT_2_MAX_ANGLE * JOINT2_REDUCTION)
#define ENGINEER_ARM_MOTOR_JOINT_23_BACK_MIN_ANGLE (ENGINEER_ARM_JOINT_2_MIN_ANGLE * JOINT2_REDUCTION)
#define ENGINEER_ARM_MOTOR_JOINT_23_FRONT_MAX_ANGLE                                                                    \
    (165.0f /* ENGINEER_ARM_JOINT_2_MAX_ANGLE + ENGINEER_ARM_JOINT_3_MAX_ANGLE */)
#define ENGINEER_ARM_MOTOR_JOINT_23_FRONT_MIN_ANGLE                                                                    \
    (-165.0f /* ENGINEER_ARM_JOINT_2_MIN_ANGLE + ENGINEER_ARM_JOINT_3_MIN_ANGLE */)
#define ENGINEER_ARM_MOTOR_JOINT_4_MAX_ANGLE (ENGINEER_ARM_JOINT_4_MAX_ANGLE)
#define ENGINEER_ARM_MOTOR_JOINT_4_MIN_ANGLE (ENGINEER_ARM_JOINT_4_MIN_ANGLE)
#define ENGINEER_ARM_MOTOR_JOINT_56_MAX_ANGLE                                                                          \
    (ENGINEER_ARM_JOINT_5_MAX_ANGLE * END_TRANSMISSION_GEAR_REDUCTION +                                                \
     ENGINEER_ARM_JOINT_6_MAX_ANGLE * END_BEVEL_GEAR_SET_REDUCTION * END_TRANSMISSION_GEAR_REDUCTION)
#define ENGINEER_ARM_MOTOR_JOINT_56_MIN_ANGLE                                                                          \
    (ENGINEER_ARM_JOINT_5_MIN_ANGLE * END_TRANSMISSION_GEAR_REDUCTION +                                                \
     ENGINEER_ARM_JOINT_6_MIN_ANGLE * END_BEVEL_GEAR_SET_REDUCTION * END_TRANSMISSION_GEAR_REDUCTION)

// 关节驱动电机初始化扩展运动范围 用于启动操作 有一定的危险性
#define ENGINEER_ARM_MOTOR_JOINT_1_INITIAL_MAX_ANGLE                                                                   \
    (ENGINEER_ARM_JOINT_1_INITIAL_MAX_DISTANCE * LIFTER_DISTANCE_TO_DEGREE_FACTOR)
#define ENGINEER_ARM_MOTOR_JOINT_1_INITIAL_MIN_ANGLE                                                                   \
    (ENGINEER_ARM_JOINT_1_INITIAL_MIN_DISTANCE * LIFTER_DISTANCE_TO_DEGREE_FACTOR)

// 电机控制器参数
#define ENGINEER_ARM_JOINT_1_RM_M3508_ANGLE_PID_KP (1.2f)
#define ENGINEER_ARM_JOINT_1_RM_M3508_ANGLE_PID_KI (0.0f)
#define ENGINEER_ARM_JOINT_1_RM_M3508_ANGLE_PID_KD (0.02f)
#define ENGINEER_ARM_JOINT_1_RM_M3508_ANGLE_PID_MAX_IOUT (0.0f)
#define ENGINEER_ARM_JOINT_1_RM_M3508_ANGLE_PID_MAX_OUT (16.0f)
#define ENGINEER_ARM_JOINT_1_RM_M3508_SPEED_PID_KP (1000.0f)
#define ENGINEER_ARM_JOINT_1_RM_M3508_SPEED_PID_KI (10.0f)
#define ENGINEER_ARM_JOINT_1_RM_M3508_SPEED_PID_KD (0.0f)
#define ENGINEER_ARM_JOINT_1_RM_M3508_SPEED_PID_MAX_IOUT (4000.0f)
#define ENGINEER_ARM_JOINT_1_RM_M3508_SPEED_PID_MAX_OUT (16000.0f)

#define ENGINEER_ARM_JOINT_4_RM_M3508_ANGLE_PID_KP (0.4f)
#define ENGINEER_ARM_JOINT_4_RM_M3508_ANGLE_PID_KI (0.0f)
#define ENGINEER_ARM_JOINT_4_RM_M3508_ANGLE_PID_KD (0.08f)
#define ENGINEER_ARM_JOINT_4_RM_M3508_ANGLE_PID_MAX_IOUT (0.0f)
#define ENGINEER_ARM_JOINT_4_RM_M3508_ANGLE_PID_MAX_OUT (16.0f)
#define ENGINEER_ARM_JOINT_4_RM_M3508_SPEED_PID_KP (1200.0f)
#define ENGINEER_ARM_JOINT_4_RM_M3508_SPEED_PID_KI (8.0f)
#define ENGINEER_ARM_JOINT_4_RM_M3508_SPEED_PID_KD (0.0f)
#define ENGINEER_ARM_JOINT_4_RM_M3508_SPEED_PID_MAX_IOUT (4000.0f)
#define ENGINEER_ARM_JOINT_4_RM_M3508_SPEED_PID_MAX_OUT (16000.0f)

#define ENGINEER_ARM_JOINT_56_RM_M2006_ANGLE_PID_KP (0.16f)
#define ENGINEER_ARM_JOINT_56_RM_M2006_ANGLE_PID_KI (0.0f)
#define ENGINEER_ARM_JOINT_56_RM_M2006_ANGLE_PID_KD (0.006f)
#define ENGINEER_ARM_JOINT_56_RM_M2006_ANGLE_PID_MAX_IOUT (0.0f)
#define ENGINEER_ARM_JOINT_56_RM_M2006_ANGLE_PID_MAX_OUT (16.0f)
#define ENGINEER_ARM_JOINT_56_RM_M2006_SPEED_PID_KP (800.0f)
#define ENGINEER_ARM_JOINT_56_RM_M2006_SPEED_PID_KI (0.8f)
#define ENGINEER_ARM_JOINT_56_RM_M2006_SPEED_PID_KD (0.0f)
#define ENGINEER_ARM_JOINT_56_RM_M2006_SPEED_PID_MAX_IOUT (2400.0f)
#define ENGINEER_ARM_JOINT_56_RM_M2006_SPEED_PID_MAX_OUT (10000.0f)

// 磁编码器安装偏差
#define ENGINEER_ARM_JOINT_6_ENCODER_ANGLE_OFFSET (0.0f)

#endif /* _ARM_TASK_H__ */
