#include "ui_element_builder.h"

#include "algo_data_limiting.h"

#include "client_ui_plot.h"

#include "remote_control.h"

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

/* ================================================================================================================== */

void uiSplitLine0Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotLine(figure, "sp0", figure_operation_type, 9, FIGURE_MAJOR_COLOR, 7, 1830, 1880, 784, 784);
}

/* ================================================================================================================== */

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

/* ================================================================================================================== */

void uiSplitLine1Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotLine(figure, "sp1", figure_operation_type, 9, FIGURE_MAJOR_COLOR, 7, 1830, 1880, 674, 674);
}

/* ================================================================================================================== */

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

/* ================================================================================================================== */

void uiSplitLine2Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotLine(figure, "sp2", figure_operation_type, 9, FIGURE_MAJOR_COLOR, 7, 1830, 1880, 564, 564);
}
void uiSplitLine3Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotLine(figure, "sp3", figure_operation_type, 9, FIGURE_MAJOR_COLOR, 7, 1830, 1880, 554, 554);
}

/* ================================================================================================================== */

void uiDt7Dr16linkIndicatorBuilder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotRectangle(figure, "dti", figure_operation_type, 9,
                    getRemoteControlStatus() == RC_USE_DT7 ? FIGURE_GREEN : FIGURE_MAGENTA, 9, 1840, 1870, 504, 534);
}

void uiVtlinkIndicatorBuilder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotRectangle(figure, "vti", figure_operation_type, 9, getVtLinkStatus() ? FIGURE_GREEN : FIGURE_MAGENTA, 9, 1840,
                    1870, 464, 494);
}

/* ================================================================================================================== */

void uiSplitLine4Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotLine(figure, "sp4", figure_operation_type, 9, FIGURE_MAJOR_COLOR, 7, 1840, 1870, 444, 444);
}

/* ================================================================================================================== */

void uiMotorStatusIndicatorBuilder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotRectangle(figure, "msi", figure_operation_type, 9,
                    checkIfMotorFailureDetected() ? FIGURE_ORANGE : FIGURE_GREEN, 9, 1840, 1870, 394, 424);
}

/* ================================================================================================================== */

void uiSplitLine5Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotLine(figure, "sp5", figure_operation_type, 9, FIGURE_MAJOR_COLOR, 7, 1890, 1890, 384, 844);
}

/* ================================================================================================================== */

#define AUTO_SILVER_MINING_AID_BOX_X_0 (722)
#define AUTO_SILVER_MINING_AID_BOX_X_1 (1012)
#define AUTO_SILVER_MINING_AID_BOX_Y_0 (565)
#define AUTO_SILVER_MINING_AID_BOX_Y_1 (718)

void uiVauAid0Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotLine(figure, "va0", figure_operation_type, 9, FIGURE_CYAN, 2, AUTO_SILVER_MINING_AID_BOX_X_0,
               AUTO_SILVER_MINING_AID_BOX_X_1, AUTO_SILVER_MINING_AID_BOX_Y_1, AUTO_SILVER_MINING_AID_BOX_Y_1);
}

void uiVauAid1Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotLine(figure, "va1", figure_operation_type, 9, FIGURE_CYAN, 2, AUTO_SILVER_MINING_AID_BOX_X_0,
               AUTO_SILVER_MINING_AID_BOX_X_0, AUTO_SILVER_MINING_AID_BOX_Y_0, AUTO_SILVER_MINING_AID_BOX_Y_1);
}

void uiVauAid2Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotLine(figure, "va2", figure_operation_type, 9, FIGURE_CYAN, 2, AUTO_SILVER_MINING_AID_BOX_X_1,
               AUTO_SILVER_MINING_AID_BOX_X_1, AUTO_SILVER_MINING_AID_BOX_Y_0, AUTO_SILVER_MINING_AID_BOX_Y_1);
}

/* ================================================================================================================== */

#define SAFE_RIGHT_BARRIER_WARNING_LINE_X_0 (1147)
#define SAFE_RIGHT_BARRIER_WARNING_LINE_X_1 (1288)
#define SAFE_RIGHT_BARRIER_WARNING_LINE_Y_0 (226)
#define SAFE_RIGHT_BARRIER_WARNING_LINE_Y_1 (0)
#define DANGER_RIGHT_BARRIER_WARNING_LINE_X_0 (1057)
#define DANGER_RIGHT_BARRIER_WARNING_LINE_X_1 (1152)
#define DANGER_RIGHT_BARRIER_WARNING_LINE_Y_0 (232)
#define DANGER_RIGHT_BARRIER_WARNING_LINE_Y_1 (0)

#define GRABBED_SAFE_RIGHT_BARRIER_WARNING_LINE_X_0 (1208)
#define GRABBED_SAFE_RIGHT_BARRIER_WARNING_LINE_X_1 (1526)
#define GRABBED_SAFE_RIGHT_BARRIER_WARNING_LINE_Y_0 (282)
#define GRABBED_SAFE_RIGHT_BARRIER_WARNING_LINE_Y_1 (0)
#define GRABBED_DANGER_RIGHT_BARRIER_WARNING_LINE_X_0 (1189)
#define GRABBED_DANGER_RIGHT_BARRIER_WARNING_LINE_X_1 (1436)
#define GRABBED_DANGER_RIGHT_BARRIER_WARNING_LINE_Y_0 (267)
#define GRABBED_DANGER_RIGHT_BARRIER_WARNING_LINE_Y_1 (0)

void uiSafeRightBarrierWarningLineBuilder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    if (checkIfArmGrabbed())
    {
        uiPlotLine(figure, "sbw", figure_operation_type, 9, FIGURE_GREEN, 5,
                   GRABBED_SAFE_RIGHT_BARRIER_WARNING_LINE_X_0, GRABBED_SAFE_RIGHT_BARRIER_WARNING_LINE_X_1,
                   GRABBED_SAFE_RIGHT_BARRIER_WARNING_LINE_Y_0, GRABBED_SAFE_RIGHT_BARRIER_WARNING_LINE_Y_1);
    }
    else
    {
        uiPlotLine(figure, "sbw", figure_operation_type, 9, FIGURE_GREEN, 5, SAFE_RIGHT_BARRIER_WARNING_LINE_X_0,
                   SAFE_RIGHT_BARRIER_WARNING_LINE_X_1, SAFE_RIGHT_BARRIER_WARNING_LINE_Y_0,
                   SAFE_RIGHT_BARRIER_WARNING_LINE_Y_1);
    }
}
void uiDangerRightBarrierWarningLineBuilder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    if (checkIfArmGrabbed())
    {
        uiPlotLine(figure, "dbw", figure_operation_type, 9, FIGURE_ORANGE, 5,
                   GRABBED_DANGER_RIGHT_BARRIER_WARNING_LINE_X_0, GRABBED_DANGER_RIGHT_BARRIER_WARNING_LINE_X_1,
                   GRABBED_DANGER_RIGHT_BARRIER_WARNING_LINE_Y_0, GRABBED_DANGER_RIGHT_BARRIER_WARNING_LINE_Y_1);
    }
    else
    {
        uiPlotLine(figure, "dbw", figure_operation_type, 9, FIGURE_ORANGE, 5, DANGER_RIGHT_BARRIER_WARNING_LINE_X_0,
                   DANGER_RIGHT_BARRIER_WARNING_LINE_X_1, DANGER_RIGHT_BARRIER_WARNING_LINE_Y_0,
                   DANGER_RIGHT_BARRIER_WARNING_LINE_Y_1);
    }
}

/* ================================================================================================================== */

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
