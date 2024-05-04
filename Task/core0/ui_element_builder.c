#include "ui_element_builder.h"

#include "algo_data_limiting.h"

#include "client_ui_plot.h"

#include "arm_task.h"
#include "behavior_task.h"
#include "storage_task.h"

/**
 * X,Y
 * 屏幕左下角坐标为 0,0
 * 屏幕右上角坐标为 1920,1080
 * 屏幕中心点坐标为 960,540
 */

void uiModeIndicatorBuilder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    figure_color_type_e color;
    if (getEngineerCurrentBehavior() == ENGINEER_BEHAVIOR_MOVE ||
        getEngineerCurrentBehavior() == ENGINEER_BEHAVIOR_MANUAL_OPERATION)
        color = FIGURE_CYAN;
    else if (getEngineerCurrentBehavior() == ENGINEER_BEHAVIOR_DISABLE)
        color = FIGURE_MAGENTA;
    else if (getEngineerCurrentBehavior() == ENGINEER_BEHAVIOR_RESET)
        color = FIGURE_YELLOW;
    else
        color = FIGURE_ORANGE;

    if (checkIfNeedRebootCore())
        color = FIGURE_BLACK;

    uiPlotRectangle(figure, "mod", figure_operation_type, 9, color, 9, 1840, 1870, 804, 834);
}

void uiSplitLine0Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotLine(figure, "sp0", figure_operation_type, 9, FIGURE_MAJOR_COLOR, 7, 1830, 1880, 784, 784);
}

void uiGrabberPoweredBuilder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotRectangle(figure, "agp", figure_operation_type, 9, getArmGrabMode() ? FIGURE_GREEN : FIGURE_MAGENTA, 9, 1840,
                    1870, 734, 764);
}

void uiGrabbedBuilder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotRectangle(figure, "agd", figure_operation_type, 9, checkIfArmGrabbed() ? FIGURE_GREEN : FIGURE_MAGENTA, 9,
                    1840, 1870, 694, 724);
}

void uiSplitLine1Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotLine(figure, "sp1", figure_operation_type, 9, FIGURE_MAJOR_COLOR, 7, 1830, 1880, 674, 674);
}

void uiStorageFrontUsedBuilder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotRectangle(figure, "sfu", figure_operation_type, 9,
                    getStorageSlotStatus(STORAGE_FRONT) == STORAGE_SLOT_USED ? FIGURE_GREEN : FIGURE_MAGENTA, 9, 1840,
                    1870, 624, 654);
}

void uiStorageBackUsedBuilder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotRectangle(figure, "sbu", figure_operation_type, 9,
                    getStorageSlotStatus(STORAGE_BACK) == STORAGE_SLOT_USED ? FIGURE_GREEN : FIGURE_MAGENTA, 9, 1840,
                    1870, 584, 614);
}

#define GOLD_PRE_AIM_BOX_X_0 (800)
#define GOLD_PRE_AIM_BOX_X_1 (900)
#define GOLD_PRE_AIM_BOX_Y_0 (440)
#define GOLD_PRE_AIM_BOX_Y_1 (640)

#define SILVER_PRE_AIM_BOX_X_0 (800)
#define SILVER_PRE_AIM_BOX_X_1 (900)
#define SILVER_PRE_AIM_BOX_Y_0 (540)
#define SILVER_PRE_AIM_BOX_Y_1 (640)

#define AUTO_SILVER_MINING_AID_BOX_X_0 (900)
#define AUTO_SILVER_MINING_AID_BOX_X_1 (1080)
#define AUTO_SILVER_MINING_AID_BOX_Y_0 (630)
#define AUTO_SILVER_MINING_AID_BOX_Y_1 (740)

void uiVauAid0Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    if (getVisualAidUi() == VAU_GOLD_PRE)
    {
        uiPlotLine(figure, "va0", figure_operation_type, 9, FIGURE_CYAN, 3, GOLD_PRE_AIM_BOX_X_1, GOLD_PRE_AIM_BOX_X_1,
                   GOLD_PRE_AIM_BOX_Y_0, GOLD_PRE_AIM_BOX_Y_1);
    }
    else if (getVisualAidUi() == VAU_SILVER_PRE)
    {
        uiPlotLine(figure, "va0", figure_operation_type, 9, FIGURE_CYAN, 3, SILVER_PRE_AIM_BOX_X_1,
                   SILVER_PRE_AIM_BOX_X_1, SILVER_PRE_AIM_BOX_Y_0, SILVER_PRE_AIM_BOX_Y_1);
    }
    else
    {
        uiPlotLine(figure, "va0", figure_operation_type, 9, FIGURE_CYAN, 3, AUTO_SILVER_MINING_AID_BOX_X_0,
                   AUTO_SILVER_MINING_AID_BOX_X_1, AUTO_SILVER_MINING_AID_BOX_Y_1, AUTO_SILVER_MINING_AID_BOX_Y_1);
    }
}

void uiVauAid1Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    if (getVisualAidUi() == VAU_GOLD_PRE)
    {
        uiPlotLine(figure, "va1", figure_operation_type, 9, FIGURE_CYAN, 3, GOLD_PRE_AIM_BOX_X_0, GOLD_PRE_AIM_BOX_X_1,
                   GOLD_PRE_AIM_BOX_Y_0, GOLD_PRE_AIM_BOX_Y_0);
    }
    else if (getVisualAidUi() == VAU_SILVER_PRE)
    {
        uiPlotLine(figure, "va1", figure_operation_type, 9, FIGURE_CYAN, 3, SILVER_PRE_AIM_BOX_X_0,
                   SILVER_PRE_AIM_BOX_X_1, SILVER_PRE_AIM_BOX_Y_0, SILVER_PRE_AIM_BOX_Y_0);
    }
    else
    {
        uiPlotLine(figure, "va1", figure_operation_type, 9, FIGURE_CYAN, 3, AUTO_SILVER_MINING_AID_BOX_X_0,
                   AUTO_SILVER_MINING_AID_BOX_X_0, AUTO_SILVER_MINING_AID_BOX_Y_0, AUTO_SILVER_MINING_AID_BOX_Y_1);
    }
}

void uiVauAid2Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    if (getVisualAidUi() == VAU_GOLD_PRE)
    {
        uiPlotLine(figure, "va2", figure_operation_type, 9, FIGURE_CYAN, 3, GOLD_PRE_AIM_BOX_X_0, GOLD_PRE_AIM_BOX_X_1,
                   GOLD_PRE_AIM_BOX_Y_1, GOLD_PRE_AIM_BOX_Y_1);
    }
    else if (getVisualAidUi() == VAU_SILVER_PRE)
    {
        uiPlotLine(figure, "va2", figure_operation_type, 9, FIGURE_CYAN, 3, SILVER_PRE_AIM_BOX_X_0,
                   SILVER_PRE_AIM_BOX_X_1, SILVER_PRE_AIM_BOX_Y_1, SILVER_PRE_AIM_BOX_Y_1);
    }
    else
    {
        uiPlotLine(figure, "va2", figure_operation_type, 9, FIGURE_CYAN, 3, AUTO_SILVER_MINING_AID_BOX_X_1,
                   AUTO_SILVER_MINING_AID_BOX_X_1, AUTO_SILVER_MINING_AID_BOX_Y_0, AUTO_SILVER_MINING_AID_BOX_Y_1);
    }
}

#define AIM_SIGHT_X (987)
#define AIM_SIGHT_Y (484)
#define AIM_SIGHT_LENGTH (24)
#define AIM_SIGHT_WIDTH (6)
#define AIM_LINE_K (-7.2894736842105f)
#define AIM_LINE_B (7678.460526315789f)

void uiAimSight0Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotLine(figure, "cs0", figure_operation_type, 9, FIGURE_CYAN, 2, AIM_SIGHT_X - AIM_SIGHT_LENGTH,
               AIM_SIGHT_X + AIM_SIGHT_LENGTH, AIM_SIGHT_Y, AIM_SIGHT_Y);
}

void uiAimSight1Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotLine(figure, "cs1", figure_operation_type, 9, FIGURE_CYAN, 3, AIM_SIGHT_X, AIM_SIGHT_X + AIM_SIGHT_WIDTH,
               AIM_SIGHT_Y, (uint32_t)(AIM_LINE_K * (AIM_SIGHT_X + AIM_SIGHT_WIDTH) + AIM_LINE_B));
}

#define SAFE_RIGHT_BARRIER_WARNING_LINE_X_0 (1147)
#define SAFE_RIGHT_BARRIER_WARNING_LINE_X_1 (1288)
#define SAFE_RIGHT_BARRIER_WARNING_LINE_Y_0 (226)
#define SAFE_RIGHT_BARRIER_WARNING_LINE_Y_1 (0)
#define DANGER_RIGHT_BARRIER_WARNING_LINE_X_0 (1057)
#define DANGER_RIGHT_BARRIER_WARNING_LINE_X_1 (1152)
#define DANGER_RIGHT_BARRIER_WARNING_LINE_Y_0 (232)
#define DANGER_RIGHT_BARRIER_WARNING_LINE_Y_1 (0)

void uiSafeRightBarrierWarningLineBuilder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotLine(figure, "sbw", figure_operation_type, 9, FIGURE_GREEN, 5, SAFE_RIGHT_BARRIER_WARNING_LINE_X_0,
               SAFE_RIGHT_BARRIER_WARNING_LINE_X_1, SAFE_RIGHT_BARRIER_WARNING_LINE_Y_0,
               SAFE_RIGHT_BARRIER_WARNING_LINE_Y_1);
}
void uiDangerRightBarrierWarningLineBuilder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotLine(figure, "dbw", figure_operation_type, 9, FIGURE_ORANGE, 5, DANGER_RIGHT_BARRIER_WARNING_LINE_X_0,
               DANGER_RIGHT_BARRIER_WARNING_LINE_X_1, DANGER_RIGHT_BARRIER_WARNING_LINE_Y_0,
               DANGER_RIGHT_BARRIER_WARNING_LINE_Y_1);
}

void uiLifterLeftMotorOverheatWarningBuilder(interaction_figure_t *figure,
                                             figure_operation_type_e figure_operation_type)
{
    uiPlotArc(figure, "loh", figure_operation_type, 9,
              (getArmMotorTemperature(MOTOR_JOINT1_RIGHT) > 75.0f)
                  ? FIGURE_MAGENTA
                  : ((getArmMotorTemperature(MOTOR_JOINT1_RIGHT) > 50.0f) ? FIGURE_ORANGE : FIGURE_GREEN),
              7, 960 - 230, 140, 180,
              (uint32_t)rflFloatLoopConstrain(getArmMotorTemperature(MOTOR_JOINT1_LEFT) / 125.0f * 360.0f + 180.0f,
                                              0.0f, 360.0f),
              24, 24);
}
void uiLifterRightMotorOverheatWarningBuilder(interaction_figure_t *figure,
                                              figure_operation_type_e figure_operation_type)
{
    uiPlotArc(figure, "roh", figure_operation_type, 9,
              (getArmMotorTemperature(MOTOR_JOINT1_RIGHT) > 75.0f)
                  ? FIGURE_MAGENTA
                  : ((getArmMotorTemperature(MOTOR_JOINT1_RIGHT) > 50.0f) ? FIGURE_ORANGE : FIGURE_GREEN),
              7, 960 + 230, 140, 180,
              (uint32_t)rflFloatLoopConstrain(getArmMotorTemperature(MOTOR_JOINT1_RIGHT) / 125.0f * 360.0f + 180.0f,
                                              0.0f, 360.0f),
              24, 24);
}
