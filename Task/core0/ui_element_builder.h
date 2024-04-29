#ifndef _UI_ELEMENT_BUILDER__
#define _UI_ELEMENT_BUILDER__

#include "referee_robot_interaction_manager.h"

#define AUTO_GRAB_CALIBRAION_BOX_X_0 (870)
#define AUTO_GRAB_CALIBRAION_BOX_X_1 (1040)
#define AUTO_GRAB_CALIBRAION_BOX_Y_0 (580)
#define AUTO_GRAB_CALIBRAION_BOX_Y_1 (680)

#define AUTO_GOLD_MINING_AID_LEFT_INDICATOR_X_0 (360)
#define AUTO_GOLD_MINING_AID_LEFT_INDICATOR_X_1 (560)
#define AUTO_GOLD_MINING_AID_MID_INDICATOR_X_0 (860)
#define AUTO_GOLD_MINING_AID_MID_INDICATOR_X_1 (1060)
#define AUTO_GOLD_MINING_AID_RIGHT_INDICATOR_X_0 (1360)
#define AUTO_GOLD_MINING_AID_RIGHT_INDICATOR_X_1 (1560)
#define AUTO_GOLD_MINING_AID_INDICATOR_Y_0 (814)
#define AUTO_GOLD_MINING_AID_INDICATOR_Y_1 (824)

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

#endif /* _UI_ELEMENT_BUILDER__ */
