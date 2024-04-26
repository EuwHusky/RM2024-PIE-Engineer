#ifndef _UI_ELEMENT_BUILDER__
#define _UI_ELEMENT_BUILDER__

#include "referee_robot_interaction_manager.h"

extern void uiModeIndicatorBuilder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type);

extern void uiPumpIndicatorBuilder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type);

extern void uiAutoGrabCalibrationLine0Builder(interaction_figure_t *figure,
                                              figure_operation_type_e figure_operation_type);
extern void uiAutoGrabCalibrationLine1Builder(interaction_figure_t *figure,
                                              figure_operation_type_e figure_operation_type);
extern void uiAutoGrabCalibrationLine2Builder(interaction_figure_t *figure,
                                              figure_operation_type_e figure_operation_type);

#endif /* _UI_ELEMENT_BUILDER__ */
