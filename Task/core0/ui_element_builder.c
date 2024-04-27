#include "ui_element_builder.h"

#include "client_ui_plot.h"

#include "arm_task.h"
#include "behavior_task.h"
#include "storage_task.h"

// 屏幕左下角坐标为 0,0
// 屏幕右上角坐标为 1920,1080
// 屏幕中心点坐标为 960,540

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

    uiPlotRectangle(figure, "mod", figure_operation_type, 9, color, 9, 1790, 1820, 804, 834);
}

void uiSplitLine0Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotLine(figure, "sp0", figure_operation_type, 9, FIGURE_MAJOR_COLOR, 7, 1780, 1830, 784, 784);
}

void uiGrabberPoweredBuilder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotRectangle(figure, "agp", figure_operation_type, 9, getArmGrabMode() ? FIGURE_GREEN : FIGURE_MAGENTA, 9, 1790,
                    1820, 734, 764);
}

void uiGrabbedBuilder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotRectangle(figure, "agd", figure_operation_type, 9, checkIfArmGrabbed() ? FIGURE_GREEN : FIGURE_MAGENTA, 9,
                    1790, 1820, 694, 724);
}

void uiSplitLine1Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotLine(figure, "sp1", figure_operation_type, 9, FIGURE_MAJOR_COLOR, 7, 1780, 1830, 674, 674);
}

void uiStorageFrontUsedBuilder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotRectangle(figure, "sfu", figure_operation_type, 9,
                    getStorageSlotStatus(STORAGE_FRONT) == STORAGE_SLOT_USED ? FIGURE_GREEN : FIGURE_MAGENTA, 9, 1790,
                    1820, 624, 654);
}

void uiStorageBackUsedBuilder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotRectangle(figure, "sbu", figure_operation_type, 9,
                    getStorageSlotStatus(STORAGE_BACK) == STORAGE_SLOT_USED ? FIGURE_GREEN : FIGURE_MAGENTA, 9, 1790,
                    1820, 584, 614);
}

void uiAutoGrabCalibrationLine0Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotLine(figure, "ag0", figure_operation_type, 9, FIGURE_CYAN, 3, 910, 1040, 680, 680);
}

void uiAutoGrabCalibrationLine1Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotRectangle(figure, "ag1", figure_operation_type, 9, FIGURE_CYAN, 3, 910, 910, 600, 680);
}

void uiAutoGrabCalibrationLine2Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotRectangle(figure, "ag2", figure_operation_type, 9, FIGURE_CYAN, 3, 1040, 1040, 600, 680);
}
