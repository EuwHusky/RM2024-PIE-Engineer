#ifndef _UI_ELEMENT_BUILDER__
#define _UI_ELEMENT_BUILDER__

#include "referee_robot_interaction_manager.h"

#include "FreeRTOS.h"
#include "task.h"

#include "algo_matrix.h"

typedef struct UiMagicSticks
{
    // 0-start_x 1-start_y 2-end_x 3-end_y
    int16_t ms0_point_pos[4];
    int16_t ms0_start_point_pos[4];
    int16_t ms0_set_point_pos[4];
    TickType_t ms0_animation_start_time;
    int16_t ms1_point_pos[4];
    int16_t ms1_start_point_pos[4];
    int16_t ms1_set_point_pos[4];
    TickType_t ms1_animation_start_time;
    int16_t ms2_point_pos[4];
    int16_t ms2_start_point_pos[4];
    int16_t ms2_set_point_pos[4];
    TickType_t ms2_animation_start_time;

    rfl_matrix_instance origin_x_axis_vector;
    rfl_matrix_instance origin_y_axis_vector;
    rfl_matrix_instance origin_z_axis_vector;
    float origin_x_axis_vector_data[3];
    float origin_y_axis_vector_data[3];
    float origin_z_axis_vector_data[3];
    rfl_matrix_instance rotation_mat;
    float rotation_mat_data[9];
    rfl_matrix_instance x_axis_vector;
    rfl_matrix_instance y_axis_vector;
    rfl_matrix_instance z_axis_vector;
    float x_axis_vector_data[3];
    float y_axis_vector_data[3];
    float z_axis_vector_data[3];

} ui_magic_sticks_s;

extern void uiMagicStick0(interaction_figure_t *figure, figure_operation_type_e figure_operation_type);
extern void uiMagicStick1(interaction_figure_t *figure, figure_operation_type_e figure_operation_type);
extern void uiMagicStick2(interaction_figure_t *figure, figure_operation_type_e figure_operation_type);

extern void uiBodyIndicator(interaction_figure_t *figure, figure_operation_type_e figure_operation_type);
extern void uiArmGrabberIndicator(interaction_figure_t *figure, figure_operation_type_e figure_operation_type);
extern void uiFrontStorageIndicator(interaction_figure_t *figure, figure_operation_type_e figure_operation_type);
extern void uiBackStorageIndicator(interaction_figure_t *figure, figure_operation_type_e figure_operation_type);

extern void uiDt7Dr16linkIndicator(interaction_figure_t *figure, figure_operation_type_e figure_operation_type);
extern void uiVtlinkIndicator(interaction_figure_t *figure, figure_operation_type_e figure_operation_type);
extern void uiRemoteIndicatorLeftSplitLine(interaction_figure_t *figure, figure_operation_type_e figure_operation_type);
extern void uiRemoteIndicatorRightSplitLine(interaction_figure_t *figure,
                                            figure_operation_type_e figure_operation_type);

extern void uiGimbalCaliLine(interaction_figure_t *figure, figure_operation_type_e figure_operation_type);

extern void uiLifterLeftMotorOverheatWarningBuilder(interaction_figure_t *figure,
                                                    figure_operation_type_e figure_operation_type);
extern void uiLifterRightMotorOverheatWarningBuilder(interaction_figure_t *figure,
                                                     figure_operation_type_e figure_operation_type);

#endif /* _UI_ELEMENT_BUILDER__ */
