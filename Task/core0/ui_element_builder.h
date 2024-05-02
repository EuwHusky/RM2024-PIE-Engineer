#ifndef _UI_ELEMENT_BUILDER__
#define _UI_ELEMENT_BUILDER__

#include "referee_robot_interaction_manager.h"

extern void uiModeIndicatorBuilder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type);

extern void uiSplitLine0Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type);

extern void uiGrabberPoweredBuilder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type);
extern void uiGrabbedBuilder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type);

extern void uiSplitLine1Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type);

extern void uiStorageFrontUsedBuilder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type);
extern void uiStorageBackUsedBuilder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type);

extern void uiAutoSilverMiningAid0Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type);
extern void uiAutoSilverMiningAid1Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type);
extern void uiAutoSilverMiningAid2Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type);

extern void uiAutoGoldMiningAidLeftIndicatorBuilder(interaction_figure_t *figure,
                                                    figure_operation_type_e figure_operation_type);
extern void uiAutoGoldMiningAidMidIndicatorBuilder(interaction_figure_t *figure,
                                                   figure_operation_type_e figure_operation_type);
extern void uiAutoGoldMiningAidRightIndicatorBuilder(interaction_figure_t *figure,
                                                     figure_operation_type_e figure_operation_type);

extern void uiSafeRightBarrierWarningLineBuilder(interaction_figure_t *figure,
                                                 figure_operation_type_e figure_operation_type);
extern void uiDangerRightBarrierWarningLineBuilder(interaction_figure_t *figure,
                                                   figure_operation_type_e figure_operation_type);

#endif /* _UI_ELEMENT_BUILDER__ */
