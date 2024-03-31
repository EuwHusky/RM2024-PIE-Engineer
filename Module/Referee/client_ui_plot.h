#ifndef _CLIENT_UI_PLOT_H__
#define _CLIENT_UI_PLOT_H__

#include "referee_protocol.h"

extern void uiPlotRectangle(interaction_figure_t *figure, char figure_name[3], figure_operation_type_e operate_tpye,
                            uint32_t layer, figure_color_type_e color, uint32_t width, uint32_t start_x, uint32_t end_x,
                            uint32_t start_y, uint32_t end_y);

#endif /* _CLIENT_UI_PLOT_H__ */
