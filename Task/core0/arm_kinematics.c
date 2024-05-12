#include "math.h"
#include "string.h"

#include "arm_kinematics.h"

#include "algo_data_limiting.h"

#include "behavior_task.h"

static void pose_6d_to_transform_matrix(rfl_matrix_instance *trans_mat, const float pose_6d[6]);
static void calc_joint_dh_to_transform_matrix(rfl_matrix_instance *trans_mat, const float joint_dh[5]);
static void calc_tool_to_base_transform_matrix(engineer_scara_arm_s *scara_arm);
static void solve_forward_kinematics(engineer_scara_arm_s *scara_arm);
static void set_pose_limiting(engineer_scara_arm_s *scara_arm);
static void solve_inverse_kinematics(engineer_scara_arm_s *scara_arm);

/**
 * @brief 初始化机械臂模型
 */
void arm_model_init(engineer_scara_arm_s *scara_arm)
{
    memset(&scara_arm->j1_to_base_tmat_data, 0, 16 * sizeof(float));
    rflMatrixInit(&scara_arm->j1_to_base_tmat, 4, 4, scara_arm->j1_to_base_tmat_data);
    const float j1_to_base_pose_6d[6] = {ARM_JI_TO_BASE_X,       ARM_JI_TO_BASE_Y,       ARM_JI_TO_BASE_Z,
                                         ARM_JI_TO_BASE_ANGLE_Z, ARM_JI_TO_BASE_ANGLE_Y, ARM_JI_TO_BASE_ANGLE_X};
    pose_6d_to_transform_matrix(&scara_arm->j1_to_base_tmat, j1_to_base_pose_6d);

    memset(&scara_arm->tool_to_j6_tmat_data, 0, 16 * sizeof(float));
    rflMatrixInit(&scara_arm->tool_to_j6_tmat, 4, 4, scara_arm->tool_to_j6_tmat_data);
    const float tool_to_j6_pose_6d[6] = {ARM_TOOL_TO_J6_X,       ARM_TOOL_TO_J6_Y,       ARM_TOOL_TO_J6_Z,
                                         ARM_TOOL_TO_J6_ANGLE_Z, ARM_TOOL_TO_J6_ANGLE_Y, ARM_TOOL_TO_J6_ANGLE_X};
    pose_6d_to_transform_matrix(&scara_arm->tool_to_j6_tmat, tool_to_j6_pose_6d);

    scara_arm->dh[0][0] = ARM_JOINT_1_DH_ALPHA;
    scara_arm->dh[0][1] = ARM_JOINT_1_DH_A;
    scara_arm->dh[0][2] = ARM_JOINT_1_DH_D;
    scara_arm->dh[0][3] = ARM_JOINT_1_DH_THETA;
    scara_arm->dh[0][4] = ARM_JOINT_1_DH_THETA_OFFSET;

    scara_arm->dh[1][0] = ARM_JOINT_2_DH_ALPHA;
    scara_arm->dh[1][1] = ARM_JOINT_2_DH_A;
    scara_arm->dh[1][2] = ARM_JOINT_2_DH_D;
    scara_arm->dh[1][3] = ARM_JOINT_2_DH_THETA;
    scara_arm->dh[1][4] = ARM_JOINT_2_DH_THETA_OFFSET;

    scara_arm->dh[2][0] = ARM_JOINT_3_DH_ALPHA;
    scara_arm->dh[2][1] = ARM_JOINT_3_DH_A;
    scara_arm->dh[2][2] = ARM_JOINT_3_DH_D;
    scara_arm->dh[2][3] = ARM_JOINT_3_DH_THETA;
    scara_arm->dh[2][4] = ARM_JOINT_3_DH_THETA_OFFSET;

    scara_arm->dh[3][0] = ARM_JOINT_4_DH_ALPHA;
    scara_arm->dh[3][1] = ARM_JOINT_4_DH_A;
    scara_arm->dh[3][2] = ARM_JOINT_4_DH_D;
    scara_arm->dh[3][3] = ARM_JOINT_4_DH_THETA;
    scara_arm->dh[3][4] = ARM_JOINT_4_DH_THETA_OFFSET;

    scara_arm->dh[4][0] = ARM_JOINT_5_DH_ALPHA;
    scara_arm->dh[4][1] = ARM_JOINT_5_DH_A;
    scara_arm->dh[4][2] = ARM_JOINT_5_DH_D;
    scara_arm->dh[4][3] = ARM_JOINT_5_DH_THETA;
    scara_arm->dh[4][4] = ARM_JOINT_5_DH_THETA_OFFSET;

    scara_arm->dh[5][0] = ARM_JOINT_6_DH_ALPHA;
    scara_arm->dh[5][1] = ARM_JOINT_6_DH_A;
    scara_arm->dh[5][2] = ARM_JOINT_6_DH_D;
    scara_arm->dh[5][3] = ARM_JOINT_6_DH_THETA;
    scara_arm->dh[5][4] = ARM_JOINT_6_DH_THETA_OFFSET;

    memset(&scara_arm->tool_to_base_tmat_data, 0, 16 * sizeof(float));
    rflMatrixInit(&scara_arm->tool_to_base_tmat, 4, 4, scara_arm->tool_to_base_tmat_data);

    scara_arm->solution = JOINT_3_ON_THE_LEFT;
}

/**
 * @brief 更新机械臂模型状态量 正运动学分析
 */
void arm_model_update_status(engineer_scara_arm_s *scara_arm)
{
    scara_arm->joints_value[JOINT_1] = (scara_arm->joints_motors[MOTOR_JOINT1_LEFT].angle_.deg +
                                        scara_arm->joints_motors[MOTOR_JOINT1_RIGHT].angle_.deg) *
                                       0.5f / LIFTER_DISTANCE_TO_DEGREE_FACTOR;
    scara_arm->joints_value[JOINT_2] = scara_arm->joints_motors[MOTOR_JOINT23_BACK].angle_.rad / JOINT2_REDUCTION;
    scara_arm->joints_value[JOINT_3] =
        (scara_arm->joints_motors[MOTOR_JOINT23_FRONT].angle_.rad + scara_arm->joint_23_front_motor_angle_offset) -
        scara_arm->joints_value[JOINT_2];
    scara_arm->joints_value[JOINT_4] = scara_arm->joints_motors[MOTOR_JOINT4].angle_.rad;
    scara_arm->joints_value[JOINT_5] = (scara_arm->joints_motors[MOTOR_JOINT56_LEFT].angle_.rad +
                                        scara_arm->joints_motors[MOTOR_JOINT56_RIGHT].angle_.rad) /
                                       2.0f;
    scara_arm->joints_value[JOINT_6] = (scara_arm->joints_motors[MOTOR_JOINT56_LEFT].angle_.rad -
                                        scara_arm->joints_motors[MOTOR_JOINT56_RIGHT].angle_.rad) /
                                       2.0f / END_BEVEL_GEAR_SET_REDUCTION;

    scara_arm->dh[0][2] = scara_arm->joints_value[JOINT_1]; // Joint 1 distance
    scara_arm->dh[1][3] = scara_arm->joints_value[JOINT_2]; // Joint 2 angle
    scara_arm->dh[2][3] = scara_arm->joints_value[JOINT_3]; // Joint 3 angle
    scara_arm->dh[3][3] = scara_arm->joints_value[JOINT_4]; // Joint 4 angle
    scara_arm->dh[4][3] = scara_arm->joints_value[JOINT_5]; // Joint 5 angle
    scara_arm->dh[5][3] = scara_arm->joints_value[JOINT_6]; // Joint 6 angle

    for (uint8_t i = 0; i < 6; i++)
    {
        scara_arm->last_pose_6d[i] = scara_arm->pose_6d[i];
    }

    solve_forward_kinematics(scara_arm);
}

/**
 * @brief 更新机械臂模型控制量 逆运动学解算
 */
void arm_model_update_control(engineer_scara_arm_s *scara_arm)
{
    if (scara_arm->behavior != ENGINEER_BEHAVIOR_RESET && scara_arm->behavior != ENGINEER_BEHAVIOR_AUTO_GOLD_MINING)
    {
#if !USE_JOINTS_CONTROL
        solve_inverse_kinematics(scara_arm);
#endif
    }
    else
    {
        for (uint8_t i = 0; i < 6; i++)
            scara_arm->set_pose_6d[i] = scara_arm->pose_6d[i];
    }

    if (scara_arm->behavior != ENGINEER_BEHAVIOR_RESET)
    {

        scara_arm->set_joints_value[JOINT_1] = rflFloatConstrain(
            scara_arm->set_joints_value[JOINT_1], ENGINEER_ARM_JOINT_1_MIN_DISTANCE, ENGINEER_ARM_JOINT_1_MAX_DISTANCE);
        scara_arm->set_joints_value[JOINT_2] = rflFloatConstrain(
            scara_arm->set_joints_value[JOINT_2], ENGINEER_ARM_JOINT_2_MIN_ANGLE * DEGREE_TO_RADIAN_FACTOR,
            ENGINEER_ARM_JOINT_2_MAX_ANGLE * DEGREE_TO_RADIAN_FACTOR);
        scara_arm->set_joints_value[JOINT_3] =
            rflFloatConstrain(scara_arm->set_joints_value[JOINT_3],
                              (ENGINEER_ARM_MOTOR_JOINT_23_FRONT_MIN_ANGLE -
                               scara_arm->set_joints_value[JOINT_2] * RADIAN_TO_DEGREE_FACTOR) *
                                  DEGREE_TO_RADIAN_FACTOR,
                              (ENGINEER_ARM_MOTOR_JOINT_23_FRONT_MAX_ANGLE -
                               scara_arm->set_joints_value[JOINT_2] * RADIAN_TO_DEGREE_FACTOR) *
                                  DEGREE_TO_RADIAN_FACTOR);
        scara_arm->set_joints_value[JOINT_4] = rflFloatConstrain(
            scara_arm->set_joints_value[JOINT_4], ENGINEER_ARM_JOINT_4_MIN_ANGLE * DEGREE_TO_RADIAN_FACTOR,
            ENGINEER_ARM_JOINT_4_MAX_ANGLE * DEGREE_TO_RADIAN_FACTOR);
        scara_arm->set_joints_value[JOINT_5] = rflFloatConstrain(
            scara_arm->set_joints_value[JOINT_5], ENGINEER_ARM_JOINT_5_MIN_ANGLE * DEGREE_TO_RADIAN_FACTOR,
            ENGINEER_ARM_JOINT_5_MAX_ANGLE * DEGREE_TO_RADIAN_FACTOR);
        scara_arm->set_joints_value[JOINT_6] = rflFloatConstrain(
            scara_arm->set_joints_value[JOINT_6], ENGINEER_ARM_JOINT_6_MIN_ANGLE * DEGREE_TO_RADIAN_FACTOR,
            ENGINEER_ARM_JOINT_6_MAX_ANGLE * DEGREE_TO_RADIAN_FACTOR);
    }
}

static void pose_6d_to_transform_matrix(rfl_matrix_instance *trans_mat, const float pose_6d[6])
{
    if (trans_mat->numCols != 4 || trans_mat->numRows != 4)
        return;

    float sin_angle_z = sinf(pose_6d[3]);
    float cos_angle_z = cosf(pose_6d[3]);
    float sin_angle_y = sinf(pose_6d[4]);
    float cos_angle_y = cosf(pose_6d[4]);
    float sin_angle_x = sinf(pose_6d[5]);
    float cos_angle_x = cosf(pose_6d[5]);

    trans_mat->pData[0] = cos_angle_z * cos_angle_y;
    trans_mat->pData[1] = cos_angle_z * sin_angle_y * sin_angle_x - sin_angle_z * cos_angle_x;
    trans_mat->pData[2] = cos_angle_z * sin_angle_y * cos_angle_x + sin_angle_z * sin_angle_x;
    trans_mat->pData[3] = pose_6d[0];

    trans_mat->pData[4] = sin_angle_z * cos_angle_y;
    trans_mat->pData[5] = sin_angle_z * sin_angle_y * sin_angle_x + cos_angle_z * cos_angle_x;
    trans_mat->pData[6] = sin_angle_z * sin_angle_y * cos_angle_x - cos_angle_z * sin_angle_x;
    trans_mat->pData[7] = pose_6d[1];

    trans_mat->pData[8] = -sin_angle_y;
    trans_mat->pData[9] = cos_angle_y * sin_angle_x;
    trans_mat->pData[10] = cos_angle_y * cos_angle_x;
    trans_mat->pData[11] = pose_6d[2];

    trans_mat->pData[12] = 0.0f;
    trans_mat->pData[13] = 0.0f;
    trans_mat->pData[14] = 0.0f;
    trans_mat->pData[15] = 1.0f;
}

static void solve_forward_kinematics(engineer_scara_arm_s *scara_arm)
{
    calc_tool_to_base_transform_matrix(scara_arm);

    scara_arm->pose_6d[0] = scara_arm->tool_to_base_tmat.pData[3];
    scara_arm->pose_6d[1] = scara_arm->tool_to_base_tmat.pData[7];
    scara_arm->pose_6d[2] = scara_arm->tool_to_base_tmat.pData[11];
    scara_arm->pose_6d[3] = scara_arm->joints_value[1] + scara_arm->joints_value[2] + scara_arm->joints_value[3];
    scara_arm->pose_6d[4] = scara_arm->joints_value[4];
    scara_arm->pose_6d[5] = scara_arm->joints_value[5];
}

static void calc_tool_to_base_transform_matrix(engineer_scara_arm_s *scara_arm)
{
    // 缓存
    rfl_matrix_instance calc_temp_mat_1;
    float calc_temp_mat_1_data[16] = {0.0f};
    rflMatrixInit(&calc_temp_mat_1, 4, 4, calc_temp_mat_1_data);
    rfl_matrix_instance calc_temp_mat_2;
    float calc_temp_mat_2_data[16] = {0.0f};
    rflMatrixInit(&calc_temp_mat_2, 4, 4, calc_temp_mat_2_data);
    rfl_matrix_instance calc_temp_mat_3;
    float calc_temp_mat_3_data[16] = {0.0f};
    rflMatrixInit(&calc_temp_mat_3, 4, 4, calc_temp_mat_3_data);
    // 计算
    memcpy(calc_temp_mat_3_data, scara_arm->j1_to_base_tmat.pData, 16 * sizeof(float));          // j0 to base
    calc_joint_dh_to_transform_matrix(&calc_temp_mat_1, scara_arm->dh[0]);                       // dh 1
    rflMatrixMult(&calc_temp_mat_3, &calc_temp_mat_1, &calc_temp_mat_2);                         // j1 to base
    calc_joint_dh_to_transform_matrix(&calc_temp_mat_1, scara_arm->dh[1]);                       // dh 2
    rflMatrixMult(&calc_temp_mat_2, &calc_temp_mat_1, &calc_temp_mat_3);                         // j2 to base
    calc_joint_dh_to_transform_matrix(&calc_temp_mat_1, scara_arm->dh[2]);                       // dh 3
    rflMatrixMult(&calc_temp_mat_3, &calc_temp_mat_1, &calc_temp_mat_2);                         // j3 to base
    calc_joint_dh_to_transform_matrix(&calc_temp_mat_1, scara_arm->dh[3]);                       // dh 4
    rflMatrixMult(&calc_temp_mat_2, &calc_temp_mat_1, &calc_temp_mat_3);                         // j4 to base
    calc_joint_dh_to_transform_matrix(&calc_temp_mat_1, scara_arm->dh[4]);                       // dh 5
    rflMatrixMult(&calc_temp_mat_3, &calc_temp_mat_1, &calc_temp_mat_2);                         // j5 to base
    calc_joint_dh_to_transform_matrix(&calc_temp_mat_1, scara_arm->dh[5]);                       // dh 6
    rflMatrixMult(&calc_temp_mat_2, &calc_temp_mat_1, &calc_temp_mat_3);                         // j6 to base
    rflMatrixMult(&calc_temp_mat_3, &scara_arm->tool_to_j6_tmat, &scara_arm->tool_to_base_tmat); // tool to base
}

static void calc_joint_dh_to_transform_matrix(rfl_matrix_instance *trans_mat, const float joint_dh[5])
{
    if (trans_mat->numCols != 4 || trans_mat->numRows != 4)
        return;

    float sin_alpha = sinf(joint_dh[0]);
    float cos_alpha = cosf(joint_dh[0]);
    float sin_theta = sinf(joint_dh[3] - joint_dh[4]);
    float cos_theta = cosf(joint_dh[3] - joint_dh[4]);

    trans_mat->pData[0] = cos_theta;
    trans_mat->pData[1] = -sin_theta;
    trans_mat->pData[2] = 0;
    trans_mat->pData[3] = joint_dh[1];

    trans_mat->pData[4] = sin_theta * cos_alpha;
    trans_mat->pData[5] = cos_theta * cos_alpha;
    trans_mat->pData[6] = -sin_alpha;
    trans_mat->pData[7] = -sin_alpha * joint_dh[2];

    trans_mat->pData[8] = sin_theta * sin_alpha;
    trans_mat->pData[9] = cos_theta * sin_alpha;
    trans_mat->pData[10] = cos_alpha;
    trans_mat->pData[11] = cos_alpha * joint_dh[2];

    trans_mat->pData[12] = 0.0f;
    trans_mat->pData[13] = 0.0f;
    trans_mat->pData[14] = 0.0f;
    trans_mat->pData[15] = 1.0f;
}

#define X scara_arm->set_pose_6d[0]
#define Y scara_arm->set_pose_6d[1]
#define Z scara_arm->set_pose_6d[2]
#define Yaw scara_arm->set_pose_6d[3]
#define Pitch scara_arm->set_pose_6d[4]
#define Roll scara_arm->set_pose_6d[5]

#define L1 ENGINEER_ARM_1_LENGTH
#define L2 ENGINEER_ARM_2_LENGTH
#define L3 ENGINEER_ARM_3_LENGTH
#define L4 ENGINEER_ARM_4_LENGTH

// 手动作业限幅参数

#define X_L (-0.3f)

#define X_B (-0.3f)
#define Y_B (0.044f)
#define Y_B_ (-0.044f)

#define X_GH (0.115f)

#define K_GK (-0.14666667f)
#define B_GK (0.24686667f)
#define K_HM (0.14666667f)
#define B_HM (-0.24686667f)

#define Y_C (ENGINEER_ARM_1_LENGTH)
#define Y_C_ (-ENGINEER_ARM_1_LENGTH)

// 自动作业限幅参数

#define X_M (-0.305f)
#define Y_INNER_BORDER (0.26f)

/**
 * @brief 期望位姿限幅
 * @note 难以用良好的程序呈现出逻辑，请阅读README文档来理解
 *
 * @param scara_arm
 */
static void set_pose_limiting(engineer_scara_arm_s *scara_arm)
{
    // 检查末端YAW角度是否超限

    Yaw = rflFloatConstrain(Yaw, ENGINEER_ARM_YAW_MIN_ANGLE * DEGREE_TO_RADIAN_FACTOR,
                            ENGINEER_ARM_YAW_MAX_ANGLE * DEGREE_TO_RADIAN_FACTOR);

    // 检查末端PITCH角度是否超限

    Pitch = rflFloatConstrain(Pitch, ENGINEER_ARM_PITCH_MIN_ANGLE * DEGREE_TO_RADIAN_FACTOR,
                              ENGINEER_ARM_PITCH_MAX_ANGLE * DEGREE_TO_RADIAN_FACTOR);

    // 检查末端ROLL角度是否超限

    Roll = rflFloatConstrain(Roll, ENGINEER_ARM_ROLL_MIN_ANGLE * DEGREE_TO_RADIAN_FACTOR,
                             ENGINEER_ARM_ROLL_MAX_ANGLE * DEGREE_TO_RADIAN_FACTOR);

    // 检查关节4在XY平面上的位置是否超限

    float xy_56 = L4 * cosf(Pitch);
    float x_56 = xy_56 * cosf(Yaw);
    float y_56 = xy_56 * sinf(Yaw);
    float x_45 = L3 * cosf(Yaw);
    float y_45 = L3 * sinf(Yaw);
    float x_24 = X - x_56 - x_45;
    float y_24 = Y - y_56 - y_45;
    float xy_24 = sqrtf(x_24 * x_24 + y_24 * y_24);

    if (scara_arm->behavior == ENGINEER_BEHAVIOR_MANUAL_OPERATION)
    {
        if (x_24 < X_L)
            x_24 = X_L;

        if (y_24 >= 0.0f)
        {
            float x_B_4 = x_24 - X_B;
            float y_B_4 = y_24 - Y_B;
            float xy_B_4 = sqrtf(x_B_4 * x_B_4 + y_B_4 * y_B_4);
            if (xy_B_4 < L1)
            {
                float t_x_B_4 = atan2f(y_B_4, x_B_4);
                x_24 = cosf(t_x_B_4) * L1 + X_B;
                y_24 = sinf(t_x_B_4) * L1 + Y_B;
            }

            float GK_y_4 = K_GK * x_24 + B_GK; // 经过关节4且平行与Y轴的直线与直线GK的交点Y坐标
            if (x_24 < X_GH && y_24 < GK_y_4)
            {
                float xy_4_GK = fabsf(K_GK * x_24 - y_24 + B_GK) / sqrtf(1 + K_GK * K_GK);
                float x_4_GH = X_GH - x_24;
                if (xy_4_GK < x_4_GH)
                {
                    y_24 = GK_y_4;
                    // float temp_x_24 = x_24;
                    // float temp_y_24 = y_24;
                    // x_24 = (temp_x_24 - K_GK * B_GK + K_GK * temp_y_24) / (1 + K_GK * K_GK);
                    // y_24 = (K_GK * temp_x_24 + K_GK * K_GK * temp_y_24 + B_GK) / (1 + K_GK * K_GK);
                }
                else
                {
                    x_24 = X_GH;
                }
            }

            else if (x_24 < 0.0f)
            {
                float y_C_4 = y_24 - Y_C;
                float xy_C_4 = sqrtf(x_24 * x_24 + y_C_4 * y_C_4);
                if (xy_C_4 > L2)
                {
                    float t_x_C_4 = atan2f(y_C_4, x_24);
                    x_24 = cosf(t_x_C_4) * L2;
                    y_24 = sinf(t_x_C_4) * L2 + Y_C;
                }
            }
        }
        else
        {
            float x_B_4 = x_24 - X_B;
            float y_B_4 = y_24 - Y_B_;
            float xy_B_4 = sqrtf(x_B_4 * x_B_4 + y_B_4 * y_B_4);
            if (xy_B_4 < L1)
            {
                float t_x_B_4 = atan2f(y_B_4, x_B_4);
                x_24 = cosf(t_x_B_4) * L1 + X_B;
                y_24 = sinf(t_x_B_4) * L1 + Y_B_;
            }

            float HM_y_4 = K_HM * x_24 + B_HM; // 经过关节4且平行与Y轴的直线与直线HM的交点Y坐标
            if (x_24 < X_GH && y_24 > HM_y_4)
            {
                float xy_4_HM = fabsf(K_HM * x_24 - y_24 + B_HM) / sqrtf(1 + K_HM * K_HM);
                float x_4_GH = X_GH - x_24;
                if (xy_4_HM < x_4_GH)
                {
                    y_24 = HM_y_4;
                    // float temp_x_24 = x_24;
                    // float temp_y_24 = y_24;
                    // x_24 = (temp_x_24 - K_HM * B_HM + K_HM * temp_y_24) / (1 + K_HM * K_HM);
                    // y_24 = (K_HM * temp_x_24 + K_HM * K_HM * temp_y_24 + B_GK) / (1 + K_HM * K_HM);
                }
                else
                {
                    x_24 = X_GH;
                }
            }

            else if (x_24 < 0.0f)
            {
                float y_C_4 = y_24 - Y_C_;
                float xy_C_4 = sqrtf(x_24 * x_24 + y_C_4 * y_C_4);
                if (xy_C_4 > L2)
                {
                    float t_x_C_4 = atan2f(y_C_4, x_24);
                    x_24 = cosf(t_x_C_4) * L2;
                    y_24 = sinf(t_x_C_4) * L2 + Y_C_;
                }
            }
        }
    }
    else
    {
        if (x_24 < X_M)
            x_24 = X_M;

        if (y_24 > 0.0f)
        {
            if (x_24 < X_GH && y_24 < Y_INNER_BORDER)
            {
                if (fabsf(X_GH - x_24) < fabsf(Y_INNER_BORDER - y_24))
                    x_24 = X_GH;
                else
                    y_24 = Y_INNER_BORDER;
            }

            else if (x_24 < 0.0f)
            {
                float y_C_4 = y_24 - Y_C;
                float xy_C_4 = sqrtf(x_24 * x_24 + y_C_4 * y_C_4);
                if (xy_C_4 > L2)
                {
                    float t_x_C_4 = atan2f(y_C_4, x_24);
                    x_24 = cosf(t_x_C_4) * L2;
                    y_24 = sinf(t_x_C_4) * L2 + Y_C;
                }
            }
        }
        else
        {
            if (x_24 < X_GH && y_24 > -Y_INNER_BORDER)
            {
                if (fabsf(X_GH - x_24) < fabsf(-Y_INNER_BORDER - y_24))
                    x_24 = X_GH;
                else
                    y_24 = -Y_INNER_BORDER;
            }

            else if (x_24 < 0.0f)
            {
                float y_C_4 = y_24 - Y_C_;
                float xy_C_4 = sqrtf(x_24 * x_24 + y_C_4 * y_C_4);
                if (xy_C_4 > L2)
                {
                    float t_x_C_4 = atan2f(y_C_4, x_24);
                    x_24 = cosf(t_x_C_4) * L2;
                    y_24 = sinf(t_x_C_4) * L2 + Y_C_;
                }
            }
        }
    }

    if (x_24 >= 0.0f && xy_24 > (ENGINEER_ARM_XY24_MAX_DISTANCE - PREVENT_DISTANCE))
    {
        float t_x24 = atan2f(y_24, x_24);
        x_24 = cosf(t_x24) * (ENGINEER_ARM_XY24_MAX_DISTANCE - PREVENT_DISTANCE);
        y_24 = sinf(t_x24) * (ENGINEER_ARM_XY24_MAX_DISTANCE - PREVENT_DISTANCE);
    }

    X = x_24 + x_45 + x_56;
    Y = y_24 + y_45 + y_56;

    // 检查末端在Z轴上是否超限

    float z_56 = L4 * sinf(Pitch);
    if (Z > (ENGINEER_ARM_JOINT_1_MAX_DISTANCE + z_56))
        Z = ENGINEER_ARM_JOINT_1_MAX_DISTANCE + z_56;
    else if (Z < (ENGINEER_ARM_JOINT_1_MIN_DISTANCE + z_56))
        Z = ENGINEER_ARM_JOINT_1_MIN_DISTANCE + z_56;
}

/**
 * @brief 位姿逆解算到关节空间
 * @note 难以用良好的程序呈现出逻辑，请阅读README文档来理解
 *
 * @param scara_arm
 */
static void solve_inverse_kinematics(engineer_scara_arm_s *scara_arm)
{
    set_pose_limiting(scara_arm);

    float xy56 = 0.0f, x56 = 0.0f, y56 = 0.0f, x45 = 0.0f, y45 = 0.0f, z56 = 0.0f, x24 = 0.0f, y24 = 0.0f, xy24 = 0.0f,
          angle_x24 = 0.0f, angle_324 = 0.0f;

    float d1 = 0.0f, t2 = 0.0f, t3 = 0.0f, t4 = 0.0f, t5 = 0.0f, t6 = 0.0f;

    z56 = L4 * sinf(Pitch);
    d1 = Z - z56;

    xy56 = L4 * cosf(Pitch);
    x56 = xy56 * cosf(Yaw);
    y56 = xy56 * sinf(Yaw);
    x45 = L3 * cosf(Yaw);
    y45 = L3 * sinf(Yaw);
    x24 = X - x56 - x45;
    y24 = Y - y56 - y45;
    xy24 = sqrtf(x24 * x24 + y24 * y24);
    angle_x24 = atan2f(y24, x24);
    angle_324 = acosf((L1 * L1 + xy24 * xy24 - L2 * L2) / (2 * L1 * xy24));

    scara_arm->printer[0] = x24;
    scara_arm->printer[1] = y24;
    scara_arm->printer[2] = xy24;

    if (scara_arm->behavior == ENGINEER_BEHAVIOR_MANUAL_OPERATION)
    {
        if (checkIfArmNeedSwitchSolution())
        {
            scara_arm->solution =
                (scara_arm->solution == JOINT_3_ON_THE_LEFT) ? JOINT_3_ON_THE_RIGHT : JOINT_3_ON_THE_LEFT;
        }
    }
    else
        scara_arm->solution = JOINT_3_ON_THE_LEFT;

    t3 = (scara_arm->solution == JOINT_3_ON_THE_LEFT ? -1.0f : 1.0f) *
         (RAD_PI - acosf((L1 * L1 + L2 * L2 - xy24 * xy24) / (2 * L1 * L2)));
    t2 = angle_x24 - (scara_arm->solution == JOINT_3_ON_THE_LEFT ? -1.0f : 1.0f) * angle_324;
    t4 = Yaw - t2 - t3;
    t5 = Pitch;
    t6 = Roll;

    if (isnormal(d1) || d1 == 0)
        scara_arm->set_joints_value[JOINT_1] = d1;
    if (isnormal(t2) || t2 == 0)
        scara_arm->set_joints_value[JOINT_2] = t2;
    if (isnormal(t3) || t3 == 0)
        scara_arm->set_joints_value[JOINT_3] = t3;
    if (isnormal(t4) || t4 == 0)
        scara_arm->set_joints_value[JOINT_4] = t4;
    if (isnormal(t5) || t5 == 0)
        scara_arm->set_joints_value[JOINT_5] = t5;
    if (isnormal(t6) || t6 == 0)
        scara_arm->set_joints_value[JOINT_6] = t6;
}
