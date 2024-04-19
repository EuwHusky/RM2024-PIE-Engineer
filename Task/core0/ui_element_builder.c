#include "ui_element_builder.h"

#include "client_ui_plot.h"

#include "behavior_task.h"

void uiPumpIndicatorBuilder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotRectangle(figure, "pum", figure_operation_type, 9, getArmGrabMode() ? FIGURE_GREEN : FIGURE_MAGENTA, 9, 768,
                    968, 532, 732);
}

void ui0Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotRectangle(figure, "000", figure_operation_type, 9, getArmGrabMode() ? FIGURE_MAGENTA : FIGURE_GREEN, 5, 768,
                    968, 632, 832);
}

void ui1Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotRectangle(figure, "001", figure_operation_type, 9, getArmGrabMode() ? FIGURE_MAGENTA : FIGURE_GREEN, 5, 868,
                    1068, 532, 732);
}

void ui2Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotRectangle(figure, "002", figure_operation_type, 9, getArmGrabMode() ? FIGURE_MAGENTA : FIGURE_GREEN, 5, 500,
                    530, 320, 330);
}

void ui3Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotRectangle(figure, "003", figure_operation_type, 9, getArmGrabMode() ? FIGURE_MAGENTA : FIGURE_GREEN, 5, 600,
                    640, 320, 330);
}

void ui4Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotRectangle(figure, "004", figure_operation_type, 9, getArmGrabMode() ? FIGURE_MAGENTA : FIGURE_GREEN, 5, 700,
                    750, 320, 330);
}

void ui5Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotRectangle(figure, "005", figure_operation_type, 9, getArmGrabMode() ? FIGURE_MAGENTA : FIGURE_GREEN, 5, 800,
                    860, 320, 330);
}

void ui6Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotRectangle(figure, "006", figure_operation_type, 9, getArmGrabMode() ? FIGURE_MAGENTA : FIGURE_GREEN, 5, 900,
                    970, 320, 330);
}
