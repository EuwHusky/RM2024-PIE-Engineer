#include "ui_element_builder.h"

#include "client_ui_plot.h"

#include "behavior_task.h"

// 屏幕左下角坐标为 0,0
// 屏幕右上角坐标为 1920,1080
// 屏幕中心点坐标为 960,540

void uiModeIndicatorBuilder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotRectangle(figure, "mod", figure_operation_type, 9,
                    (getEngineerCurrentBehavior() == ENGINEER_BEHAVIOR_MOVE ||
                     getEngineerCurrentBehavior() == ENGINEER_BEHAVIOR_MANUAL_OPERATION)
                        ? FIGURE_CYAN
                        : FIGURE_ORANGE,
                    32, 1664, 1694, 784, 814);
}

void uiPumpIndicatorBuilder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotRectangle(figure, "pum", figure_operation_type, 9, getArmGrabMode() ? FIGURE_GREEN : FIGURE_MAGENTA, 32, 1664,
                    1694, 734, 764);
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
