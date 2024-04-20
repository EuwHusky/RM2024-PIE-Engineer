#include "ui_element_builder.h"

#include "client_ui_plot.h"

#include "behavior_task.h"

// 屏幕左下角坐标为 0,0
// 屏幕右上角坐标为 1920,1080
// 屏幕中心点坐标为 960,540

void uiPumpIndicatorBuilder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotRectangle(figure, "pum", figure_operation_type, 9, getArmGrabMode() ? FIGURE_GREEN : FIGURE_MAGENTA, 32, 1664,
                    1694, 784, 814);
}

void uiAutoGrabCalibrationLine0Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotLine(figure, "ag0", figure_operation_type, 9, FIGURE_CYAN, 3, 620, 1300, 832, 832);
}

void uiAutoGrabCalibrationLine1Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotRectangle(figure, "ag1", figure_operation_type, 9, FIGURE_CYAN, 3, 620, 620, 432, 832);
}

void uiAutoGrabCalibrationLine2Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotRectangle(figure, "ag2", figure_operation_type, 9, FIGURE_CYAN, 3, 1300, 1300, 432, 832);
}
