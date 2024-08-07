#include "ui_element_builder.h"

#include "client_ui_plot.h"

#include "algo_data_limiting.h"

#include "remote_control.h"

#include "Detect_task.h"
#include "arm_task.h"
#include "behavior_task.h"
#include "gimbal_task.h"
#include "storage_task.h"

static void line_animation_generator(int16_t point_pos_param[4], int16_t start_point_pos_param[4],
                                     int16_t set_point_pos_param[4], TickType_t start_time, TickType_t now_time,
                                     TickType_t total_time);
static void eular_to_rotmat(float roll, float pitch, float yaw, rfl_matrix_instance *mat);

#define UI_DEFAULT_LAYER (5)

#define DEFAULT_MAGIC_STICK_LAYER (UI_DEFAULT_LAYER)
#define DEFAULT_MAGIC_STICK_WIDTH (4)

#define DISABLE_MAGIC_STICK_COLOR (FIGURE_PINK)
#define RESET_MAGIC_STICK_COLOR (FIGURE_YELLOW)
#define AUTO_HOMING_MAGIC_STICK_COLOR (FIGURE_ORANGE)

#define MAGIC_STICK_0_COLOR (FIGURE_PINK)
#define MAGIC_STICK_1_COLOR (FIGURE_GREEN)
#define MAGIC_STICK_2_COLOR (FIGURE_CYAN)

#define OPERATION_MAGIC_STICK_LENGTH (60)
#define OPERATION_MAGIC_STICK_X (960)
#define OPERATION_MAGIC_STICK_Y (540)

/* 六边形内矩形 半长 26 半高15 */
#define HALF_X (26)
#define HALF_Y (15)

/* 左下右上方向 */
#define DEFAULT_MAGIC_STICK_0_START_X (960 - HALF_X)
#define DEFAULT_MAGIC_STICK_0_START_Y (540 - HALF_Y)
#define DEFAULT_MAGIC_STICK_0_END_X (960 + HALF_X)
#define DEFAULT_MAGIC_STICK_0_END_Y (540 + HALF_Y)
#define MOVE_MAGIC_STICK_0_START_X (1152)
#define MOVE_MAGIC_STICK_0_START_Y (0)
#define MOVE_MAGIC_STICK_0_END_X (1057)
#define MOVE_MAGIC_STICK_0_END_Y (232)

/* 左上右下方向 */
#define DEFAULT_MAGIC_STICK_1_START_X (960 + HALF_X)
#define DEFAULT_MAGIC_STICK_1_START_Y (540 - HALF_Y)
#define DEFAULT_MAGIC_STICK_1_END_X (960 - HALF_X)
#define DEFAULT_MAGIC_STICK_1_END_Y (540 + HALF_Y)
#define MOVE_MAGIC_STICK_1_START_X (777)
#define MOVE_MAGIC_STICK_1_START_Y (0)
#define MOVE_MAGIC_STICK_1_END_X (888)
#define MOVE_MAGIC_STICK_1_END_Y (232)

/* 上下方向 */
#define DEFAULT_MAGIC_STICK_2_START_X (960)
#define DEFAULT_MAGIC_STICK_2_START_Y (540 - OPERATION_MAGIC_STICK_LENGTH / 2)
#define DEFAULT_MAGIC_STICK_2_END_X (960)
#define DEFAULT_MAGIC_STICK_2_END_Y (540 + OPERATION_MAGIC_STICK_LENGTH / 2)
#define MOVE_MAGIC_STICK_2_START_X (1288)
#define MOVE_MAGIC_STICK_2_START_Y (0)
#define MOVE_MAGIC_STICK_2_END_X (1147)
#define MOVE_MAGIC_STICK_2_END_Y (232)

ui_magic_sticks_s ui_magic_sticks = {0};

void uiMagicStick0(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    static engineer_behavior_e last_behavior = ENGINEER_BEHAVIOR_DISABLE;
    static bool inited = false;
    if (!inited)
    {
        memset(&ui_magic_sticks.origin_x_axis_vector, 0, sizeof(rfl_matrix_instance));
        memset(&ui_magic_sticks.origin_y_axis_vector, 0, sizeof(rfl_matrix_instance));
        memset(&ui_magic_sticks.origin_z_axis_vector, 0, sizeof(rfl_matrix_instance));
        memset(&ui_magic_sticks.rotation_mat_data, 0, sizeof(rfl_matrix_instance));
        memset(&ui_magic_sticks.x_axis_vector, 0, sizeof(rfl_matrix_instance));
        memset(&ui_magic_sticks.y_axis_vector, 0, sizeof(rfl_matrix_instance));
        memset(&ui_magic_sticks.z_axis_vector, 0, sizeof(rfl_matrix_instance));

        memset(&ui_magic_sticks.origin_x_axis_vector_data, 0, 3 * sizeof(float));
        memset(&ui_magic_sticks.origin_y_axis_vector_data, 0, 3 * sizeof(float));
        memset(&ui_magic_sticks.origin_z_axis_vector_data, 0, 3 * sizeof(float));
        memset(&ui_magic_sticks.rotation_mat_data, 0, 9 * sizeof(float));
        memset(&ui_magic_sticks.x_axis_vector_data, 0, 3 * sizeof(float));
        memset(&ui_magic_sticks.y_axis_vector_data, 0, 3 * sizeof(float));
        memset(&ui_magic_sticks.z_axis_vector_data, 0, 3 * sizeof(float));

        rflMatrixInit(&ui_magic_sticks.origin_x_axis_vector, 3, 1, ui_magic_sticks.origin_x_axis_vector_data);
        rflMatrixInit(&ui_magic_sticks.origin_y_axis_vector, 3, 1, ui_magic_sticks.origin_y_axis_vector_data);
        rflMatrixInit(&ui_magic_sticks.origin_z_axis_vector, 3, 1, ui_magic_sticks.origin_z_axis_vector_data);

        rflMatrixInit(&ui_magic_sticks.rotation_mat, 3, 3, ui_magic_sticks.rotation_mat_data);

        rflMatrixInit(&ui_magic_sticks.x_axis_vector, 3, 1, ui_magic_sticks.x_axis_vector_data);
        rflMatrixInit(&ui_magic_sticks.y_axis_vector, 3, 1, ui_magic_sticks.y_axis_vector_data);
        rflMatrixInit(&ui_magic_sticks.z_axis_vector, 3, 1, ui_magic_sticks.z_axis_vector_data);

        inited = true;
    }

    figure_color_type_e color = MAGIC_STICK_0_COLOR;

    engineer_behavior_e behavior = getEngineerCurrentBehavior();
    if (behavior != last_behavior)
    {
        ui_magic_sticks.ms0_animation_start_time = xTaskGetTickCount();
        for (uint8_t i = 0; i < 4; i++)
        {
            ui_magic_sticks.ms0_start_point_pos[i] = ui_magic_sticks.ms0_point_pos[i];
        }
    }
    last_behavior = behavior;

    switch (behavior)
    {
    case ENGINEER_BEHAVIOR_DISABLE:
        color = DISABLE_MAGIC_STICK_COLOR;
        ui_magic_sticks.ms0_point_pos[0] = DEFAULT_MAGIC_STICK_0_START_X;
        ui_magic_sticks.ms0_point_pos[1] = DEFAULT_MAGIC_STICK_0_START_Y;
        ui_magic_sticks.ms0_point_pos[2] = DEFAULT_MAGIC_STICK_0_END_X;
        ui_magic_sticks.ms0_point_pos[3] = DEFAULT_MAGIC_STICK_0_END_Y;
        break;
    case ENGINEER_BEHAVIOR_RESET:
        color = RESET_MAGIC_STICK_COLOR;
        ui_magic_sticks.ms0_point_pos[0] = DEFAULT_MAGIC_STICK_0_START_X;
        ui_magic_sticks.ms0_point_pos[1] = DEFAULT_MAGIC_STICK_0_START_Y;
        ui_magic_sticks.ms0_point_pos[2] = DEFAULT_MAGIC_STICK_0_END_X;
        ui_magic_sticks.ms0_point_pos[3] = DEFAULT_MAGIC_STICK_0_END_Y;
        break;
    case ENGINEER_BEHAVIOR_AUTO_MOVE_HOMING:
        color = AUTO_HOMING_MAGIC_STICK_COLOR;
        ui_magic_sticks.ms0_set_point_pos[0] = MOVE_MAGIC_STICK_0_START_X;
        ui_magic_sticks.ms0_set_point_pos[1] = MOVE_MAGIC_STICK_0_START_Y;
        ui_magic_sticks.ms0_set_point_pos[2] = MOVE_MAGIC_STICK_0_END_X;
        ui_magic_sticks.ms0_set_point_pos[3] = MOVE_MAGIC_STICK_0_END_Y;
        line_animation_generator(ui_magic_sticks.ms0_point_pos, ui_magic_sticks.ms0_start_point_pos,
                                 ui_magic_sticks.ms0_set_point_pos, ui_magic_sticks.ms0_animation_start_time,
                                 xTaskGetTickCount(), 500);
        break;
    case ENGINEER_BEHAVIOR_MOVE:
        ui_magic_sticks.ms0_point_pos[0] = MOVE_MAGIC_STICK_0_START_X;
        ui_magic_sticks.ms0_point_pos[1] = MOVE_MAGIC_STICK_0_START_Y;
        ui_magic_sticks.ms0_point_pos[2] = MOVE_MAGIC_STICK_0_END_X;
        ui_magic_sticks.ms0_point_pos[3] = MOVE_MAGIC_STICK_0_END_Y;
        break;
    case ENGINEER_BEHAVIOR_AUTO_OPERATION_HOMING:
        color = AUTO_HOMING_MAGIC_STICK_COLOR;
        ui_magic_sticks.ms0_set_point_pos[0] = OPERATION_MAGIC_STICK_X;
        ui_magic_sticks.ms0_set_point_pos[1] = OPERATION_MAGIC_STICK_Y;
        ui_magic_sticks.ms0_set_point_pos[2] = OPERATION_MAGIC_STICK_X;
        ui_magic_sticks.ms0_set_point_pos[3] = OPERATION_MAGIC_STICK_Y;
        line_animation_generator(ui_magic_sticks.ms0_point_pos, ui_magic_sticks.ms0_start_point_pos,
                                 ui_magic_sticks.ms0_set_point_pos, ui_magic_sticks.ms0_animation_start_time,
                                 xTaskGetTickCount(), 500);
        break;
    // case ENGINEER_BEHAVIOR_MANUAL_OPERATION:
    default:
        eular_to_rotmat(getArmPose(POSE_YAW), -getArmPose(POSE_PITCH), getArmPose(POSE_ROLL),
                        &ui_magic_sticks.rotation_mat);
        ui_magic_sticks.origin_x_axis_vector_data[0] = OPERATION_MAGIC_STICK_LENGTH;
        ui_magic_sticks.origin_x_axis_vector_data[1] = 0.0f;
        ui_magic_sticks.origin_x_axis_vector_data[2] = 0.0f;
        rflMatrixMult(&ui_magic_sticks.rotation_mat, &ui_magic_sticks.origin_x_axis_vector,
                      &ui_magic_sticks.x_axis_vector);

        ui_magic_sticks.ms0_point_pos[0] = OPERATION_MAGIC_STICK_X;
        ui_magic_sticks.ms0_point_pos[1] = OPERATION_MAGIC_STICK_Y;
        ui_magic_sticks.ms0_point_pos[2] = OPERATION_MAGIC_STICK_X - (int16_t)(ui_magic_sticks.x_axis_vector.pData[1]);
        ui_magic_sticks.ms0_point_pos[3] = OPERATION_MAGIC_STICK_Y + (int16_t)(ui_magic_sticks.x_axis_vector.pData[2]);
        break;
    }

    color = checkIfNeedRebootCore() ? FIGURE_BLACK : color;

    uiPlotLine(figure, "ms0", figure_operation_type, DEFAULT_MAGIC_STICK_LAYER, color, DEFAULT_MAGIC_STICK_WIDTH,
               (uint32_t)ui_magic_sticks.ms0_point_pos[0], (uint32_t)ui_magic_sticks.ms0_point_pos[2],
               (uint32_t)ui_magic_sticks.ms0_point_pos[1], (uint32_t)ui_magic_sticks.ms0_point_pos[3]);
}

void uiMagicStick1(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    static engineer_behavior_e last_behavior = ENGINEER_BEHAVIOR_DISABLE;

    figure_color_type_e color = MAGIC_STICK_1_COLOR;

    engineer_behavior_e behavior = getEngineerCurrentBehavior();
    if (behavior != last_behavior)
    {
        ui_magic_sticks.ms1_animation_start_time = xTaskGetTickCount();
        for (uint8_t i = 0; i < 4; i++)
        {
            ui_magic_sticks.ms1_start_point_pos[i] = ui_magic_sticks.ms1_point_pos[i];
        }
    }
    last_behavior = behavior;

    switch (behavior)
    {
    case ENGINEER_BEHAVIOR_DISABLE:
        color = DISABLE_MAGIC_STICK_COLOR;
        ui_magic_sticks.ms1_point_pos[0] = DEFAULT_MAGIC_STICK_1_START_X;
        ui_magic_sticks.ms1_point_pos[1] = DEFAULT_MAGIC_STICK_1_START_Y;
        ui_magic_sticks.ms1_point_pos[2] = DEFAULT_MAGIC_STICK_1_END_X;
        ui_magic_sticks.ms1_point_pos[3] = DEFAULT_MAGIC_STICK_1_END_Y;
        break;
    case ENGINEER_BEHAVIOR_RESET:
        color = RESET_MAGIC_STICK_COLOR;
        ui_magic_sticks.ms1_point_pos[0] = DEFAULT_MAGIC_STICK_1_START_X;
        ui_magic_sticks.ms1_point_pos[1] = DEFAULT_MAGIC_STICK_1_START_Y;
        ui_magic_sticks.ms1_point_pos[2] = DEFAULT_MAGIC_STICK_1_END_X;
        ui_magic_sticks.ms1_point_pos[3] = DEFAULT_MAGIC_STICK_1_END_Y;
        break;
    case ENGINEER_BEHAVIOR_AUTO_MOVE_HOMING:
        color = AUTO_HOMING_MAGIC_STICK_COLOR;
        ui_magic_sticks.ms1_set_point_pos[0] = MOVE_MAGIC_STICK_1_START_X;
        ui_magic_sticks.ms1_set_point_pos[1] = MOVE_MAGIC_STICK_1_START_Y;
        ui_magic_sticks.ms1_set_point_pos[2] = MOVE_MAGIC_STICK_1_END_X;
        ui_magic_sticks.ms1_set_point_pos[3] = MOVE_MAGIC_STICK_1_END_Y;
        line_animation_generator(ui_magic_sticks.ms1_point_pos, ui_magic_sticks.ms1_start_point_pos,
                                 ui_magic_sticks.ms1_set_point_pos, ui_magic_sticks.ms1_animation_start_time,
                                 xTaskGetTickCount(), 500);
        break;
    case ENGINEER_BEHAVIOR_MOVE:
        ui_magic_sticks.ms1_point_pos[0] = MOVE_MAGIC_STICK_1_START_X;
        ui_magic_sticks.ms1_point_pos[1] = MOVE_MAGIC_STICK_1_START_Y;
        ui_magic_sticks.ms1_point_pos[2] = MOVE_MAGIC_STICK_1_END_X;
        ui_magic_sticks.ms1_point_pos[3] = MOVE_MAGIC_STICK_1_END_Y;
        break;
    case ENGINEER_BEHAVIOR_AUTO_OPERATION_HOMING:
        color = AUTO_HOMING_MAGIC_STICK_COLOR;
        ui_magic_sticks.ms1_set_point_pos[0] = OPERATION_MAGIC_STICK_X;
        ui_magic_sticks.ms1_set_point_pos[1] = OPERATION_MAGIC_STICK_Y;
        ui_magic_sticks.ms1_set_point_pos[2] = OPERATION_MAGIC_STICK_X - OPERATION_MAGIC_STICK_LENGTH;
        ui_magic_sticks.ms1_set_point_pos[3] = OPERATION_MAGIC_STICK_Y;
        line_animation_generator(ui_magic_sticks.ms1_point_pos, ui_magic_sticks.ms1_start_point_pos,
                                 ui_magic_sticks.ms1_set_point_pos, ui_magic_sticks.ms1_animation_start_time,
                                 xTaskGetTickCount(), 500);
        break;
    // case ENGINEER_BEHAVIOR_MANUAL_OPERATION:
    default:
        eular_to_rotmat(getArmPose(POSE_YAW), -getArmPose(POSE_PITCH), getArmPose(POSE_ROLL),
                        &ui_magic_sticks.rotation_mat);
        ui_magic_sticks.origin_y_axis_vector_data[0] = 0.0f;
        ui_magic_sticks.origin_y_axis_vector_data[1] = OPERATION_MAGIC_STICK_LENGTH;
        ui_magic_sticks.origin_y_axis_vector_data[2] = 0.0f;
        rflMatrixMult(&ui_magic_sticks.rotation_mat, &ui_magic_sticks.origin_y_axis_vector,
                      &ui_magic_sticks.y_axis_vector);

        ui_magic_sticks.ms1_point_pos[0] = OPERATION_MAGIC_STICK_X;
        ui_magic_sticks.ms1_point_pos[1] = OPERATION_MAGIC_STICK_Y;
        ui_magic_sticks.ms1_point_pos[2] = OPERATION_MAGIC_STICK_X - (int16_t)(ui_magic_sticks.y_axis_vector.pData[1]);
        ui_magic_sticks.ms1_point_pos[3] = OPERATION_MAGIC_STICK_Y + (int16_t)(ui_magic_sticks.y_axis_vector.pData[2]);
        break;
    }

    color = checkIfNeedRebootCore() ? FIGURE_BLACK : color;

    uiPlotLine(figure, "ms1", figure_operation_type, DEFAULT_MAGIC_STICK_LAYER, color, DEFAULT_MAGIC_STICK_WIDTH,
               (uint32_t)ui_magic_sticks.ms1_point_pos[0], (uint32_t)ui_magic_sticks.ms1_point_pos[2],
               (uint32_t)ui_magic_sticks.ms1_point_pos[1], (uint32_t)ui_magic_sticks.ms1_point_pos[3]);
}

void uiMagicStick2(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    static engineer_behavior_e last_behavior = ENGINEER_BEHAVIOR_DISABLE;

    figure_color_type_e color = MAGIC_STICK_2_COLOR;

    engineer_behavior_e behavior = getEngineerCurrentBehavior();
    if (behavior != last_behavior)
    {
        ui_magic_sticks.ms2_animation_start_time = xTaskGetTickCount();
        for (uint8_t i = 0; i < 4; i++)
        {
            ui_magic_sticks.ms2_start_point_pos[i] = ui_magic_sticks.ms2_point_pos[i];
        }
    }
    last_behavior = behavior;

    switch (behavior)
    {
    case ENGINEER_BEHAVIOR_DISABLE:
        color = DISABLE_MAGIC_STICK_COLOR;
        ui_magic_sticks.ms2_point_pos[0] = DEFAULT_MAGIC_STICK_2_START_X;
        ui_magic_sticks.ms2_point_pos[1] = DEFAULT_MAGIC_STICK_2_START_Y;
        ui_magic_sticks.ms2_point_pos[2] = DEFAULT_MAGIC_STICK_2_END_X;
        ui_magic_sticks.ms2_point_pos[3] = DEFAULT_MAGIC_STICK_2_END_Y;
        break;
    case ENGINEER_BEHAVIOR_RESET:
        color = RESET_MAGIC_STICK_COLOR;
        ui_magic_sticks.ms2_point_pos[0] = DEFAULT_MAGIC_STICK_2_START_X;
        ui_magic_sticks.ms2_point_pos[1] = DEFAULT_MAGIC_STICK_2_START_Y;
        ui_magic_sticks.ms2_point_pos[2] = DEFAULT_MAGIC_STICK_2_END_X;
        ui_magic_sticks.ms2_point_pos[3] = DEFAULT_MAGIC_STICK_2_END_Y;
        break;
    case ENGINEER_BEHAVIOR_AUTO_MOVE_HOMING:
        color = AUTO_HOMING_MAGIC_STICK_COLOR;
        ui_magic_sticks.ms2_set_point_pos[0] = MOVE_MAGIC_STICK_2_START_X;
        ui_magic_sticks.ms2_set_point_pos[1] = MOVE_MAGIC_STICK_2_START_Y;
        ui_magic_sticks.ms2_set_point_pos[2] = MOVE_MAGIC_STICK_2_END_X;
        ui_magic_sticks.ms2_set_point_pos[3] = MOVE_MAGIC_STICK_2_END_Y;
        line_animation_generator(ui_magic_sticks.ms2_point_pos, ui_magic_sticks.ms2_start_point_pos,
                                 ui_magic_sticks.ms2_set_point_pos, ui_magic_sticks.ms2_animation_start_time,
                                 xTaskGetTickCount(), 500);
        break;
    case ENGINEER_BEHAVIOR_MOVE:
        ui_magic_sticks.ms2_point_pos[0] = MOVE_MAGIC_STICK_2_START_X;
        ui_magic_sticks.ms2_point_pos[1] = MOVE_MAGIC_STICK_2_START_Y;
        ui_magic_sticks.ms2_point_pos[2] = MOVE_MAGIC_STICK_2_END_X;
        ui_magic_sticks.ms2_point_pos[3] = MOVE_MAGIC_STICK_2_END_Y;
        break;
    case ENGINEER_BEHAVIOR_AUTO_OPERATION_HOMING:
        color = AUTO_HOMING_MAGIC_STICK_COLOR;
        ui_magic_sticks.ms2_set_point_pos[0] = OPERATION_MAGIC_STICK_X;
        ui_magic_sticks.ms2_set_point_pos[1] = OPERATION_MAGIC_STICK_Y;
        ui_magic_sticks.ms2_set_point_pos[2] = OPERATION_MAGIC_STICK_X;
        ui_magic_sticks.ms2_set_point_pos[3] = OPERATION_MAGIC_STICK_Y + OPERATION_MAGIC_STICK_LENGTH;
        line_animation_generator(ui_magic_sticks.ms2_point_pos, ui_magic_sticks.ms2_start_point_pos,
                                 ui_magic_sticks.ms2_set_point_pos, ui_magic_sticks.ms2_animation_start_time,
                                 xTaskGetTickCount(), 500);
        break;
    // case ENGINEER_BEHAVIOR_MANUAL_OPERATION:
    default:
        eular_to_rotmat(getArmPose(POSE_YAW), -getArmPose(POSE_PITCH), getArmPose(POSE_ROLL),
                        &ui_magic_sticks.rotation_mat);
        ui_magic_sticks.origin_z_axis_vector_data[0] = 0.0f;
        ui_magic_sticks.origin_z_axis_vector_data[1] = 0.0f;
        ui_magic_sticks.origin_z_axis_vector_data[2] = OPERATION_MAGIC_STICK_LENGTH;
        rflMatrixMult(&ui_magic_sticks.rotation_mat, &ui_magic_sticks.origin_z_axis_vector,
                      &ui_magic_sticks.z_axis_vector);

        ui_magic_sticks.ms2_point_pos[0] = OPERATION_MAGIC_STICK_X;
        ui_magic_sticks.ms2_point_pos[1] = OPERATION_MAGIC_STICK_Y;
        ui_magic_sticks.ms2_point_pos[2] = OPERATION_MAGIC_STICK_X - (int16_t)(ui_magic_sticks.z_axis_vector.pData[1]);
        ui_magic_sticks.ms2_point_pos[3] = OPERATION_MAGIC_STICK_Y + (int16_t)(ui_magic_sticks.z_axis_vector.pData[2]);
        break;
    }

    color = checkIfNeedRebootCore() ? FIGURE_BLACK : color;

    uiPlotLine(figure, "ms2", figure_operation_type, DEFAULT_MAGIC_STICK_LAYER, color, DEFAULT_MAGIC_STICK_WIDTH,
               (uint32_t)ui_magic_sticks.ms2_point_pos[0], (uint32_t)ui_magic_sticks.ms2_point_pos[2],
               (uint32_t)ui_magic_sticks.ms2_point_pos[1], (uint32_t)ui_magic_sticks.ms2_point_pos[3]);
}

static void line_animation_generator(int16_t point_pos_param[4], int16_t start_point_pos_param[4],
                                     int16_t set_point_pos_param[4], TickType_t start_time, TickType_t now_time,
                                     TickType_t total_time)
{
    int16_t delta[4];
    float delta_ratio[4] = {0.0f};
    for (uint8_t i = 0; i < 4; i++)
    {
        delta_ratio[i] = (float)(now_time - start_time) / (float)total_time;
        if (delta_ratio[i] > 1.0f)
            delta_ratio[i] = 1.0f;
        delta[i] = set_point_pos_param[i] - start_point_pos_param[i];
        point_pos_param[i] = start_point_pos_param[i] + (int16_t)(delta_ratio[i] * (float)delta[i]);
    }
}

static void eular_to_rotmat(float roll, float pitch, float yaw, rfl_matrix_instance *mat)
{
    if (mat->numCols != 3 || mat->numRows != 3)
        return;

    float cr, cp, cy, sr, sp, sy;

    cr = cosf(roll);
    cp = cosf(pitch);
    cy = cosf(yaw);
    sr = sinf(roll);
    sp = sinf(pitch);
    sy = sinf(yaw);

    mat->pData[0] = cp * cr;
    mat->pData[1] = sy * sp * cr - cy * sr;
    mat->pData[2] = cy * sp * cr + sy * sr;
    mat->pData[3] = cp * sr;
    mat->pData[4] = sy * sp * sr + cy * cr;
    mat->pData[5] = cy * sp * sr - sy * cr;
    mat->pData[6] = -sp;
    mat->pData[7] = sy * cp;
    mat->pData[8] = cy * cp;
}

#define BODY_X (1661)
#define BODY_Y (541)
#define BODY_INDICATOR_WIDTH (12)
#define BODY_INDICATOR_RADIUS (60)
#define STORAGE_INDICATOR_WIDTH (10)
#define STORAGE_INDICATOR_RADIUS (24)
#define STORAGE_INDICATOR_DISTANCE (BODY_INDICATOR_WIDTH / 2 + STORAGE_INDICATOR_WIDTH / 2 + STORAGE_INDICATOR_RADIUS)
#define STORAGE_INDICATOR_ANGLE_HALF_DIFF (30.0f)

void uiBodyIndicator(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    figure_color_type_e color;
    if (getEngineerCurrentBehavior() == ENGINEER_BEHAVIOR_MOVE ||
        getEngineerCurrentBehavior() == ENGINEER_BEHAVIOR_MANUAL_OPERATION)
        color = FIGURE_CYAN;
    else if (getEngineerCurrentBehavior() == ENGINEER_BEHAVIOR_DISABLE)
        color = FIGURE_PINK;
    else if (getEngineerCurrentBehavior() == ENGINEER_BEHAVIOR_RESET)
        color = FIGURE_YELLOW;
    else
        color = FIGURE_ORANGE;

    color = checkIfNeedRebootCore() ? FIGURE_BLACK : color;

    uiPlotCircle(figure, "byd", figure_operation_type, UI_DEFAULT_LAYER, color, BODY_INDICATOR_WIDTH, BODY_X, BODY_Y,
                 BODY_INDICATOR_RADIUS);
}

void uiArmGrabberIndicator(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    figure_color_type_e color = checkIfArmGrabbed()
                                    ? (getLatestNuggetTypeToGrab() == GOLD_NUGGET ? FIGURE_YELLOW : FIGURE_WHITE)
                                    : (getArmGrabMode() ? FIGURE_GREEN : FIGURE_CYAN);
    color = getEngineerCurrentBehavior() == ENGINEER_BEHAVIOR_DISABLE ? FIGURE_PINK : color;
    color = checkIfNeedRebootCore() ? FIGURE_BLACK : color;

    float follow_mid_angle = rflFloatLoopConstrain(getArmTargetDirection() * RADIAN_TO_DEGREE_FACTOR -
                                                       getGimbalYawAngle(RFL_ANGLE_FORMAT_DEGREE) + 90.0f,
                                                   -DEG_PI, DEG_PI) *
                             DEGREE_TO_RADIAN_FACTOR;
    float follow_mid_radius = BODY_INDICATOR_RADIUS + STORAGE_INDICATOR_DISTANCE;
    float follow_mid_x = BODY_X + cosf(follow_mid_angle) * follow_mid_radius;
    float follow_mid_y = BODY_Y + sinf(follow_mid_angle) * follow_mid_radius;

    uiPlotCircle(figure, "agi", figure_operation_type, UI_DEFAULT_LAYER, color, STORAGE_INDICATOR_WIDTH,
                 (uint32_t)follow_mid_x, (uint32_t)follow_mid_y, STORAGE_INDICATOR_RADIUS);
}

void uiFrontStorageIndicator(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    figure_color_type_e color =
        getStorageSlotStatus(STORAGE_FRONT) == STORAGE_SLOT_USED
            ? (getStorageSlotNuggetType(STORAGE_FRONT) == GOLD_NUGGET ? FIGURE_YELLOW : FIGURE_WHITE)
            : (checkIfStorageSlotPowered(STORAGE_FRONT) ? FIGURE_GREEN : FIGURE_CYAN);
    color = getEngineerCurrentBehavior() == ENGINEER_BEHAVIOR_DISABLE ? FIGURE_PINK : color;
    color = checkIfNeedRebootCore() ? FIGURE_BLACK : color;

    float follow_mid_angle =
        rflFloatLoopConstrain(-getGimbalYawAngle(RFL_ANGLE_FORMAT_DEGREE) + STORAGE_INDICATOR_ANGLE_HALF_DIFF, -DEG_PI,
                              DEG_PI) *
        DEGREE_TO_RADIAN_FACTOR;
    float follow_mid_radius = BODY_INDICATOR_RADIUS + STORAGE_INDICATOR_DISTANCE;
    float follow_mid_x = BODY_X + cosf(follow_mid_angle) * follow_mid_radius;
    float follow_mid_y = BODY_Y + sinf(follow_mid_angle) * follow_mid_radius;

    uiPlotCircle(figure, "fsi", figure_operation_type, UI_DEFAULT_LAYER, color, STORAGE_INDICATOR_WIDTH,
                 (uint32_t)follow_mid_x, (uint32_t)follow_mid_y, STORAGE_INDICATOR_RADIUS);
}

void uiBackStorageIndicator(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    figure_color_type_e color =
        getStorageSlotStatus(STORAGE_BACK) == STORAGE_SLOT_USED
            ? (getStorageSlotNuggetType(STORAGE_BACK) == GOLD_NUGGET ? FIGURE_YELLOW : FIGURE_WHITE)
            : (checkIfStorageSlotPowered(STORAGE_BACK) ? FIGURE_GREEN : FIGURE_CYAN);
    color = getEngineerCurrentBehavior() == ENGINEER_BEHAVIOR_DISABLE ? FIGURE_PINK : color;
    color = checkIfNeedRebootCore() ? FIGURE_BLACK : color;

    float follow_mid_angle =
        rflFloatLoopConstrain(-getGimbalYawAngle(RFL_ANGLE_FORMAT_DEGREE) - STORAGE_INDICATOR_ANGLE_HALF_DIFF, -DEG_PI,
                              DEG_PI) *
        DEGREE_TO_RADIAN_FACTOR;
    float follow_mid_radius = BODY_INDICATOR_RADIUS + STORAGE_INDICATOR_DISTANCE;
    float follow_mid_x = BODY_X + cosf(follow_mid_angle) * follow_mid_radius;
    float follow_mid_y = BODY_Y + sinf(follow_mid_angle) * follow_mid_radius;

    uiPlotCircle(figure, "bsi", figure_operation_type, UI_DEFAULT_LAYER, color, STORAGE_INDICATOR_WIDTH,
                 (uint32_t)follow_mid_x, (uint32_t)follow_mid_y, STORAGE_INDICATOR_RADIUS);
}

#define REMOTE_LINK_INDICATOR_WIDTH (10)
#define REMOTE_LINK_INDICATOR_RADIUS (23)
#define REMOTE_LINK_INDICATOR_SPLITER_WIDTH (16)

void uiDt7Dr16linkIndicator(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    figure_color_type_e color = detect_error(DBUS_DH) ? FIGURE_PINK : FIGURE_GREEN;
    color = checkIfNeedRebootCore() ? FIGURE_BLACK : color;
    uiPlotArc(figure, "dli", figure_operation_type, UI_DEFAULT_LAYER, color, REMOTE_LINK_INDICATOR_WIDTH, BODY_X,
              BODY_Y, 270 + 6, 90 - 6, REMOTE_LINK_INDICATOR_RADIUS, REMOTE_LINK_INDICATOR_RADIUS);
}

void uiVtlinkIndicator(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    figure_color_type_e color = detect_error(VT_REFEREE_DH) ? FIGURE_PINK : FIGURE_GREEN;
    color = checkIfNeedRebootCore() ? FIGURE_BLACK : color;
    uiPlotArc(figure, "vli", figure_operation_type, UI_DEFAULT_LAYER, color, REMOTE_LINK_INDICATOR_WIDTH, BODY_X,
              BODY_Y, 90 + 6, 270 - 6, REMOTE_LINK_INDICATOR_RADIUS, REMOTE_LINK_INDICATOR_RADIUS);
}

void uiRemoteIndicatorLeftSplitLine(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotArc(figure, "lil", figure_operation_type, UI_DEFAULT_LAYER, FIGURE_WHITE, REMOTE_LINK_INDICATOR_SPLITER_WIDTH,
              BODY_X, BODY_Y, 270 - 2, 270 + 2, REMOTE_LINK_INDICATOR_RADIUS, REMOTE_LINK_INDICATOR_RADIUS);
}

void uiRemoteIndicatorRightSplitLine(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotArc(figure, "ril", figure_operation_type, UI_DEFAULT_LAYER, FIGURE_WHITE, REMOTE_LINK_INDICATOR_SPLITER_WIDTH,
              BODY_X, BODY_Y, 90 - 2, 90 + 2, REMOTE_LINK_INDICATOR_RADIUS, REMOTE_LINK_INDICATOR_RADIUS);
}

void uiGimbalCaliLine(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotLine(figure, "gcl", figure_operation_type, 9, FIGURE_WHITE, 4, 622, 622, 501, 607);
}

#define LIFTER_MOTORS_SAFE_TEMPERATURE (50.0f)
#define LIFTER_MOTORS_DANGER_TEMPERATURE (75.0f)
#define LIFTER_MOTORS_MAX_TEMPERATURE (125.0f)

void uiLifterLeftMotorOverheatWarningBuilder(interaction_figure_t *figure,
                                             figure_operation_type_e figure_operation_type)
{
    uiPlotArc(
        figure, "loh", figure_operation_type, 9,
        (getArmMotorTemperature(MOTOR_JOINT1_LEFT) > LIFTER_MOTORS_DANGER_TEMPERATURE)
            ? FIGURE_MAGENTA
            : ((getArmMotorTemperature(MOTOR_JOINT1_LEFT) > LIFTER_MOTORS_SAFE_TEMPERATURE) ? FIGURE_ORANGE
                                                                                            : FIGURE_GREEN),
        7, 960 + 230, 140, 180,
        (uint32_t)rflFloatLoopConstrain(
            getArmMotorTemperature(MOTOR_JOINT1_LEFT) / LIFTER_MOTORS_MAX_TEMPERATURE * 360.0f + 180.0f, 0.0f, 360.0f),
        24, 24);
}
void uiLifterRightMotorOverheatWarningBuilder(interaction_figure_t *figure,
                                              figure_operation_type_e figure_operation_type)
{
    uiPlotArc(
        figure, "roh", figure_operation_type, 9,
        (getArmMotorTemperature(MOTOR_JOINT1_RIGHT) > LIFTER_MOTORS_DANGER_TEMPERATURE)
            ? FIGURE_MAGENTA
            : ((getArmMotorTemperature(MOTOR_JOINT1_RIGHT) > LIFTER_MOTORS_SAFE_TEMPERATURE) ? FIGURE_ORANGE
                                                                                             : FIGURE_GREEN),
        7, 960 - 230, 140, 180,
        (uint32_t)rflFloatLoopConstrain(
            getArmMotorTemperature(MOTOR_JOINT1_RIGHT) / LIFTER_MOTORS_MAX_TEMPERATURE * 360.0f + 180.0f, 0.0f, 360.0f),
        24, 24);
}

void uiErrorIndicator(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    figure_color_type_e color =
        (detect_error(PM_REFEREE_DH) || detect_error(ARM_JOINT_1_L_DH) || detect_error(ARM_JOINT_1_R_DH) ||
         detect_error(ARM_JOINT_4_DH) || detect_error(ARM_JOINT_56_L_DH) || detect_error(ARM_JOINT_56_R_DH) ||
         detect_error(CHASSIS_MOTOR_0_DH) || detect_error(CHASSIS_MOTOR_1_DH) || detect_error(CHASSIS_MOTOR_2_DH) ||
         detect_error(CHASSIS_MOTOR_3_DH))
            ? FIGURE_ORANGE
            : FIGURE_GREEN;
    color = checkIfNeedRebootCore() ? FIGURE_BLACK : color;
    uiPlotCircle(figure, "eri", figure_operation_type, 9, color, 8, BODY_X, BODY_Y, 5);
}
