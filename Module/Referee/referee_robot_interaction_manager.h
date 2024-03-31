#ifndef _REFEREE_ROBOT_INTERACTION_MANAGER_H__
#define _REFEREE_ROBOT_INTERACTION_MANAGER_H__

#include "referee.h"
#include "referee_protocol.h"

#define AUTO_ID_OBTAIN 1 // 是否自动获取ID

#if AUTO_ID_OBTAIN
#define ROBOT_SELF_ID getRobotStatusId()
#else
#define ROBOT_SELF_ID ROBOT_ID_RED_HERO
#endif

#define REFEREE_SYSTEM_SERVER_ID 0x8080

typedef enum RobotInteractionType
{
    // ROBOT_TO_ROBOT_INTERACTION = 0,
    CLIENT_UI_PLOT,
    CLIENT_UI_CLEAN,
    SENTRY_CMD,
    RADAR_CMD,

} robot_interaction_type_e;

typedef struct InteractionMessageFactory
{
    uint8_t *message;
    void (*builder)(uint8_t target_id, uint8_t *);
} interaction_message_factory_t;

typedef struct InteractionFigureFactory
{
    interaction_figure_t figure;
    void (*builder)(interaction_figure_t *);
} interaction_figure_factory_t;

typedef struct InteractionCharacterFactory
{
    interaction_character_t character;
    void (*builder)(interaction_character_t *);
} interaction_character_factory_t;

typedef struct RobotInteractionManager
{
    uint32_t *step_clock;               // 管理器时钟
    uint16_t step_time;                 // 管理器时钟步长 单位 ms
    uint32_t last_successful_send_time; // 上次成功发送的时间

    robot_interaction_data_header_t robot_interaction_data_header; // 子内容数据头
    uint16_t sub_cmd_id;                                           // 子内容ID
    uint8_t *interaction_data_sent;                                // 子内容数据

    // // 机器人间通信
    // uint8_t message_target_num;
    // interaction_message_factory_t messages;

    // UI
    interaction_layer_delete_t layer_deleter;
    void (*layer_deleter_builder)(interaction_layer_delete_t *);
    uint8_t figure_num;
    uint8_t plotted_figure_num;
    interaction_figure_factory_t *figures;
    uint8_t character_num;
    uint8_t plotted_character_num;
    interaction_character_factory_t *characters;

    // 烧饼自主决策
    interaction_sentry_cmd_t sentry_cmd;
    void (*sentry_cmd_builder)(interaction_sentry_cmd_t *);

    // 雷达自主决策
    interaction_radar_cmd_t radar_cmd;
    void (*radar_cmd_builder)(interaction_radar_cmd_t *);

} robot_interaction_manager_t;

extern void refereeInitRobotInteractionManager(uint32_t *step_clock_source, uint16_t step_time, uint8_t figure_num,
                                               uint8_t character_num);
extern uint8_t *refereeEncodeRobotInteractionData(robot_interaction_type_e robot_interaction_type);
extern void refereeRobotInteractionManagerSuccessfullySentHook(void);

// extern void refereeSetRobotInteractionMessageBuilder(uint8_t index, void (*builder)(interaction_figure_t *));

extern void refereeSetRobotInteractionLayerDeleterBuilder(void (*builder)(interaction_layer_delete_t *));
extern void refereeSetRobotInteractionFigureBuilder(uint8_t index, void (*builder)(interaction_figure_t *));
extern void refereeSetRobotInteractionCharacterBuilder(uint8_t index, void (*builder)(interaction_character_t *));

#endif /* _REFEREE_ROBOT_INTERACTION_MANAGER_H__ */
