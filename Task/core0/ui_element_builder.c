#include "ui_element_builder.h"

#include "client_ui_plot.h"

void uiPumpIndicatorBuilder(interaction_figure_t *figure)
{
    // static uint8_t is_init_plot_flag = 1;
    // if (is_init_plot_flag)
    // {
    uiPlotRectangle(figure, "pum", FIGURE_ADD, 9, FIGURE_GREEN, 5, 768, 968, 432, 632);
    //     is_init_plot_flag = 0;
    // }
    // else
    //     uiPlotRectangle(figure, "pum", FIGURE_MODIFY, 9, FIGURE_GREEN, 5, 768, 968, 432, 632);
}
