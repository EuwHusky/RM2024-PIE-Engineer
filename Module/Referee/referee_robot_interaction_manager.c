#include "stdlib.h"
#include "string.h"

#include "referee_robot_interaction_manager.h"

#include "referee_frame_process.h"

static robot_interaction_manager_t manager;

static uint8_t *encode_client_ui_delete_data(layer_delete_type_e delete_type, uint8_t layer);
static uint8_t *manage_and_encode_client_ui_plot_data(void);

static robot_interaction_data_header_t *modify_interaction_data_header(uint16_t sub_cmd_id, uint16_t target_id);
static uint8_t get_sub_data_len_by_sub_cmd_id(uint16_t sub_cmd_id);

void refereeInitRobotInteractionManager(uint32_t *step_clock_source, uint16_t step_time, uint8_t figure_num,
                                        uint8_t character_num)
{
    if (step_clock_source == NULL)
        return;
    if (step_time < 1)
        return;

    memset(&manager, 0, sizeof(robot_interaction_manager_t));

    manager.interaction_data_sent = malloc(REF_PROTOCOL_FRAME_DATA_MAX_SIZE);
    memset(manager.interaction_data_sent, 0, REF_PROTOCOL_FRAME_DATA_MAX_SIZE);

    manager.step_clock = step_clock_source;
    manager.step_time = step_time;

    manager.figure_num = figure_num;
    if (manager.figure_num > 0)
    {
        manager.figures = malloc(manager.figure_num * sizeof(interaction_figure_factory_t));
        memset(manager.figures, 0, manager.figure_num * sizeof(interaction_figure_factory_t));
    }

    manager.character_num = character_num;
    if (manager.character_num > 0)
    {
        manager.characters = malloc(manager.character_num * sizeof(interaction_character_factory_t));
        memset(manager.characters, 0, manager.character_num * sizeof(interaction_character_factory_t));
    }
}

/**
 * @brief 编码机器人交互数据
 *
 * @param robot_interaction_type 机器人交互类型
 * @return uint8_t* 一整包可直接发给裁判系统的数据流
 * @note 使用的时候务必检查返回值是否为NULL，仅当不为NULL时用户才可发送
 */
uint8_t *refereeEncodeRobotInteractionData(robot_interaction_type_e robot_interaction_type)
{
    uint8_t *sub_data = NULL;

    if ((float)(*manager.step_clock - manager.last_successful_send_time) >
        (ROBOT_INTERACTION_COMM_TIME_MS / (float)manager.step_time))
    {
        switch (robot_interaction_type)
        {
        // case ROBOT_TO_ROBOT_INTERACTION: {
        // }
        case CLIENT_UI_PLOT: {
            if (sub_data = manage_and_encode_client_ui_plot_data(), sub_data == NULL)
                return NULL;
            return referee_pack_data(ROBOT_INTERACTION_DATA_CMD_ID, sub_data,
                                     sizeof(robot_interaction_data_header_t) +
                                         get_sub_data_len_by_sub_cmd_id(manager.sub_cmd_id));
        }
        case SENTRY_CMD: {
        }
        case RADAR_CMD: {
        }

        default:
            break;
        }
    }

    return NULL;
}

void refereeRobotInteractionManagerSuccessfullySentHook(void)
{
    manager.last_successful_send_time = *manager.step_clock;
}

// void refereeSetRobotInteractionMessageBuilder(uint8_t index, void (*builder)(interaction_figure_t *))
// {
//     if (index >= manager.message_target_num)
//         return;
//     if (builder == NULL)
//         return;

//     manager.figures[index].builder = builder;
// }

void refereeSetRobotInteractionLayerDeleterBuilder(void (*builder)(interaction_layer_delete_t *))
{
    if (builder == NULL)
        return;

    manager.layer_deleter_builder = builder;
}

void refereeSetRobotInteractionFigureBuilder(uint8_t index,
                                             void (*builder)(interaction_figure_t *, figure_operation_type_e))
{
    if (index >= manager.figure_num)
        return;
    if (builder == NULL)
        return;

    manager.figures[index].builder = builder;
}

void refereeSetRobotInteractionCharacterBuilder(uint8_t index,
                                                void (*builder)(interaction_character_t *, figure_operation_type_e))
{
    if (index >= manager.character_num)
        return;
    if (builder == NULL)
        return;

    manager.characters[index].builder = builder;
}

void refereeClientUiOperate(client_ui_operation_type_e operation_type, uint8_t index)
{
    if (operation_type == UI_RESET_ALL)
    {
        for (uint8_t i = 0; i < manager.figure_num; i++)
            manager.figures[i].is_plotted = false;
        for (uint8_t i = 0; i < manager.character_num; i++)
            manager.characters[i].is_plotted = false;
    }
    else if (operation_type == UI_RESET_ALL_FIGURES)
    {
        for (uint8_t i = 0; i < manager.figure_num; i++)
            manager.figures[i].is_plotted = false;
    }
    else if (operation_type == UI_RESET_ALL_CHARACTERS)
    {
        for (uint8_t i = 0; i < manager.character_num; i++)
            manager.characters[i].is_plotted = false;
    }
    else if (operation_type == UI_RESET_FIGURE)
    {
        if (index >= manager.figure_num)
            return;
        manager.figures[index].is_plotted = false;
    }
    else if (operation_type == UI_RESET_CHARACTER)
    {
        if (index >= manager.character_num)
            return;
        manager.characters[index].is_plotted = false;
    }
    else if (operation_type == UI_HIDE_ALL)
    {
        for (uint8_t i = 0; i < manager.figure_num; i++)
            manager.figures[i].is_hidden = true;
        for (uint8_t i = 0; i < manager.character_num; i++)
            manager.characters[i].is_hidden = true;
    }
    else if (operation_type == UI_HIDE_ALL_FIGURES)
    {
        for (uint8_t i = 0; i < manager.figure_num; i++)
            manager.figures[i].is_hidden = true;
    }
    else if (operation_type == UI_HIDE_ALL_CHARACTERS)
    {
        for (uint8_t i = 0; i < manager.character_num; i++)
            manager.characters[i].is_hidden = true;
    }
    else if (operation_type == UI_HIDE_FIGURE)
    {
        if (index >= manager.figure_num)
            return;
        manager.figures[index].is_hidden = true;
    }
    else if (operation_type == UI_HIDE_CHARACTER)
    {
        if (index >= manager.character_num)
            return;
        manager.characters[index].is_hidden = true;
    }
    else if (operation_type == UI_DISPLAY_ALL)
    {
        for (uint8_t i = 0; i < manager.figure_num; i++)
            manager.figures[i].is_hidden = false;
        for (uint8_t i = 0; i < manager.character_num; i++)
            manager.characters[i].is_hidden = false;
    }
    else if (operation_type == UI_DISPLAY_ALL_FIGURES)
    {
        for (uint8_t i = 0; i < manager.figure_num; i++)
            manager.figures[i].is_hidden = false;
    }
    else if (operation_type == UI_DISPLAY_ALL_CHARACTERS)
    {
        for (uint8_t i = 0; i < manager.character_num; i++)
            manager.characters[i].is_hidden = false;
    }
    else if (operation_type == UI_DISPLAY_FIGURE)
    {
        if (index >= manager.figure_num)
            return;
        manager.figures[index].is_hidden = false;
    }
    else if (operation_type == UI_DISPLAY_CHARACTER)
    {
        if (index >= manager.character_num)
            return;
        manager.characters[index].is_hidden = false;
    }
}

// 机器人间通信还需要提供修改内容的API和获取编码后数据流的API

static uint8_t *encode_client_ui_delete_data(layer_delete_type_e delete_type, uint8_t layer)
{
    return NULL;
}

static uint8_t *manage_and_encode_client_ui_plot_data(void)
{
    // 优先打印静态内容 随后便不再打印 一次只能打印一个
    if (manager.plotted_character_num < manager.character_num)
    {
        manager.sub_cmd_id = PLOT_CHARACTER_FIGURE_SUBCMD_ID;

        // 构建图形
        if (manager.characters[manager.plotted_character_num].builder != NULL)
        {
            figure_operation_type_e figure_operation_type = FIGURE_NOP;
            if (manager.characters[manager.plotted_character_num].is_hidden)
            {
                if (manager.characters[manager.plotted_character_num].is_plotted)
                {
                    figure_operation_type = FIGURE_DELETE;
                    manager.characters[manager.plotted_character_num].is_plotted = false;
                }
                else
                {
                    figure_operation_type = FIGURE_NOP;
                }
            }
            else
            {
                if (manager.characters[manager.plotted_character_num].is_plotted)
                {
                    figure_operation_type = FIGURE_MODIFY;
                }
                else
                {
                    figure_operation_type = FIGURE_ADD;
                    manager.characters[manager.plotted_character_num].is_plotted = true;
                }
            }

            manager.characters[manager.plotted_character_num].builder(
                &manager.characters[manager.plotted_character_num].character, figure_operation_type);
        }

        // 数据头编码
        memcpy(manager.interaction_data_sent,
               modify_interaction_data_header(manager.sub_cmd_id, getRobotClientId(ROBOT_SELF_ID)),
               sizeof(robot_interaction_data_header_t));
        // 数据内容编码
        memcpy(manager.interaction_data_sent + sizeof(robot_interaction_data_header_t),
               &manager.characters[manager.plotted_character_num].character, sizeof(interaction_character_t));

        manager.plotted_character_num++;

        return manager.interaction_data_sent;
    }

    // 循环打印动态内容 一次可以打印至多七个
    else if (manager.figure_num > 0)
    {
        uint8_t to_be_plotted_figure_num_this_round = 0;
        uint8_t unplotted_figure_num = manager.figure_num - manager.plotted_figure_num;

        // 管理打印流
        if (unplotted_figure_num >= 7)
        {
            manager.sub_cmd_id = PLOT_SEVEN_FIGURE_SUBCMD_ID;
            to_be_plotted_figure_num_this_round = 7;
        }
        else if (unplotted_figure_num >= 5)
        {
            manager.sub_cmd_id = PLOT_FIVE_FIGURE_SUBCMD_ID;
            to_be_plotted_figure_num_this_round = 5;
        }
        else if (unplotted_figure_num >= 2)
        {
            manager.sub_cmd_id = PLOT_TWO_FIGURE_SUBCMD_ID;
            to_be_plotted_figure_num_this_round = 2;
        }
        else if (unplotted_figure_num == 1)
        {
            manager.sub_cmd_id = PLOT_ONE_FIGURE_SUBCMD_ID;
            to_be_plotted_figure_num_this_round = 1;
        }

        // 构建图形
        for (uint8_t i = 0; i < to_be_plotted_figure_num_this_round; i++)
        {
            if (manager.figures[manager.plotted_figure_num + i].builder != NULL)
            {
                figure_operation_type_e figure_operation_type = FIGURE_NOP;
                if (manager.figures[manager.plotted_figure_num + i].is_hidden)
                {
                    if (manager.figures[manager.plotted_figure_num + i].is_plotted)
                    {
                        figure_operation_type = FIGURE_DELETE;
                        manager.figures[manager.plotted_figure_num + i].is_plotted = false;
                    }
                    else
                    {
                        figure_operation_type = FIGURE_NOP;
                    }
                }
                else
                {
                    if (manager.figures[manager.plotted_figure_num + i].is_plotted)
                    {
                        figure_operation_type = FIGURE_MODIFY;
                    }
                    else
                    {
                        figure_operation_type = FIGURE_ADD;
                        manager.figures[manager.plotted_figure_num + i].is_plotted = true;
                    }
                }

                manager.figures[manager.plotted_figure_num + i].builder(
                    &manager.figures[manager.plotted_figure_num + i].figure, figure_operation_type);
            }
        }

        // 数据头编码
        memcpy(manager.interaction_data_sent,
               modify_interaction_data_header(manager.sub_cmd_id, getRobotClientId(ROBOT_SELF_ID)),
               sizeof(robot_interaction_data_header_t));
        // 数据内容编码
        for (uint8_t i = 0; i < to_be_plotted_figure_num_this_round; i++)
        {
            memcpy(manager.interaction_data_sent + sizeof(robot_interaction_data_header_t) +
                       i * sizeof(interaction_figure_t),
                   &manager.figures[manager.plotted_figure_num + i].figure, sizeof(interaction_figure_t));
        }

        // 结算打印流并循环
        manager.plotted_figure_num += to_be_plotted_figure_num_this_round;
        if (manager.plotted_figure_num >= manager.figure_num)
            manager.plotted_figure_num = 0;

        return manager.interaction_data_sent;
    }

    return NULL;
}

// 烧饼和雷达还需要提供修改内容的API和获取编码后数据流的API

static robot_interaction_data_header_t *modify_interaction_data_header(uint16_t sub_cmd_id, uint16_t target_id)
{
    manager.robot_interaction_data_header.data_cmd_id = sub_cmd_id;
    manager.robot_interaction_data_header.sender_id = ROBOT_SELF_ID;
    manager.robot_interaction_data_header.receiver_id = target_id;

    return &manager.robot_interaction_data_header;
}

static uint8_t get_sub_data_len_by_sub_cmd_id(uint16_t sub_cmd_id)
{
    if (sub_cmd_id == LAYER_DELETE_SUBCMD_ID)
        return ROBOT_INTERACTION_LAYER_DELETE_SUB_DATA_SIZE;
    else if (sub_cmd_id == PLOT_ONE_FIGURE_SUBCMD_ID)
        return ROBOT_INTERACTION_PLOT_ONE_FIGURE_SUB_DATA_SIZE;
    else if (sub_cmd_id == PLOT_TWO_FIGURE_SUBCMD_ID)
        return ROBOT_INTERACTION_PLOT_TWO_FIGURE_SUB_DATA_SIZE;
    else if (sub_cmd_id == PLOT_FIVE_FIGURE_SUBCMD_ID)
        return ROBOT_INTERACTION_PLOT_FIVE_FIGURE_SUB_DATA_SIZE;
    else if (sub_cmd_id == PLOT_SEVEN_FIGURE_SUBCMD_ID)
        return ROBOT_INTERACTION_PLOT_SEVEN_FIGURE_SUB_DATA_SIZE;
    else if (sub_cmd_id == PLOT_CHARACTER_FIGURE_SUBCMD_ID)
        return ROBOT_INTERACTION_PLOT_CHARACTER_FIGURE_SUB_DATA_SIZE;
    else if (sub_cmd_id == SENTRY_CMD_SUBCMD_ID)
        return ROBOT_INTERACTION_SENTRY_CMD_SUB_DATA_SIZE;
    else if (sub_cmd_id == RADAR_CMD_SUBCMD_ID)
        return ROBOT_INTERACTION_RADAR_CMD_SUB_DATA_SIZE;

    return 113;
}
