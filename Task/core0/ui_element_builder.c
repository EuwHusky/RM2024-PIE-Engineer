#include "ui_element_builder.h"

#include "client_ui_plot.h"

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

    uiPlotRectangle(figure, "mod", figure_operation_type, 9, color, 9, 1840, 1870, 804, 834);
}

void uiSplitLine0Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotLine(figure, "sp0", figure_operation_type, 9, FIGURE_MAJOR_COLOR, 7, 1830, 1880, 784, 784);
}

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

void uiSplitLine1Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotLine(figure, "sp1", figure_operation_type, 9, FIGURE_MAJOR_COLOR, 7, 1830, 1880, 674, 674);
}

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

void uiAutoSilverMiningAid0Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotLine(figure, "sa0", figure_operation_type, 9, FIGURE_CYAN, 3, AUTO_GRAB_CALIBRAION_BOX_X_0,
               AUTO_GRAB_CALIBRAION_BOX_X_1, AUTO_GRAB_CALIBRAION_BOX_Y_1, AUTO_GRAB_CALIBRAION_BOX_Y_1);
}

void uiAutoSilverMiningAid1Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotLine(figure, "sa1", figure_operation_type, 9, FIGURE_CYAN, 3, AUTO_GRAB_CALIBRAION_BOX_X_0,
               AUTO_GRAB_CALIBRAION_BOX_X_0, AUTO_GRAB_CALIBRAION_BOX_Y_0, AUTO_GRAB_CALIBRAION_BOX_Y_1);
}

void uiAutoSilverMiningAid2Builder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotLine(figure, "sa2", figure_operation_type, 9, FIGURE_CYAN, 3, AUTO_GRAB_CALIBRAION_BOX_X_1,
               AUTO_GRAB_CALIBRAION_BOX_X_1, AUTO_GRAB_CALIBRAION_BOX_Y_0, AUTO_GRAB_CALIBRAION_BOX_Y_1);
}

void uiAutoGoldMiningAidLeftIndicatorBuilder(interaction_figure_t *figure,
                                             figure_operation_type_e figure_operation_type)
{
    uiPlotRectangle(figure, "gil", figure_operation_type, 9,
                    getVisualAidUi() == VAU_LEFT_GOLD ? FIGURE_CYAN : FIGURE_BLACK, 5,
                    AUTO_GOLD_MINING_AID_LEFT_INDICATOR_X_0, AUTO_GOLD_MINING_AID_LEFT_INDICATOR_X_1,
                    AUTO_GOLD_MINING_AID_INDICATOR_Y_0, AUTO_GOLD_MINING_AID_INDICATOR_Y_1);
}

void uiAutoGoldMiningAidMidIndicatorBuilder(interaction_figure_t *figure, figure_operation_type_e figure_operation_type)
{
    uiPlotRectangle(figure, "gim", figure_operation_type, 9,
                    getVisualAidUi() == VAU_MID_GOLD ? FIGURE_CYAN : FIGURE_BLACK, 5,
                    AUTO_GOLD_MINING_AID_MID_INDICATOR_X_0, AUTO_GOLD_MINING_AID_MID_INDICATOR_X_1,
                    AUTO_GOLD_MINING_AID_INDICATOR_Y_0, AUTO_GOLD_MINING_AID_INDICATOR_Y_1);
}

void uiAutoGoldMiningAidRightIndicatorBuilder(interaction_figure_t *figure,
                                              figure_operation_type_e figure_operation_type)
{
    uiPlotRectangle(figure, "gir", figure_operation_type, 9,
                    getVisualAidUi() == VAU_RIGHT_GOLD ? FIGURE_CYAN : FIGURE_BLACK, 5,
                    AUTO_GOLD_MINING_AID_RIGHT_INDICATOR_X_0, AUTO_GOLD_MINING_AID_RIGHT_INDICATOR_X_1,
                    AUTO_GOLD_MINING_AID_INDICATOR_Y_0, AUTO_GOLD_MINING_AID_INDICATOR_Y_1);
}
