#ifndef _REFEREE_PROTOCOL_H__
#define _REFEREE_PROTOCOL_H__

#include "stdint.h"

#define HEADER_SOF 0xA5
#define REF_PROTOCOL_FRAME_MAX_SIZE 128
#define REF_PROTOCOL_FRAME_DATA_MAX_SIZE 119

#define REF_PROTOCOL_HEADER_SIZE 5
#define REF_PROTOCOL_CMD_SIZE 2
#define REF_PROTOCOL_CRC16_SIZE 2
#define REF_HEADER_CMDID_LEN (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CMD_SIZE)
#define REF_HEADER_CRC_LEN (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + REF_PROTOCOL_CMD_SIZE)

#define ROBOT_INTERACTION_COMM_FREQ 10
#define ROBOT_INTERACTION_COMM_TIME_MS (1000 / ROBOT_INTERACTION_COMM_FREQ)

#define ROBOT_INTERACTION_LAYER_DELETE_SUB_DATA_SIZE 2
#define ROBOT_INTERACTION_PLOT_ONE_FIGURE_SUB_DATA_SIZE 15
#define ROBOT_INTERACTION_PLOT_TWO_FIGURE_SUB_DATA_SIZE 30
#define ROBOT_INTERACTION_PLOT_FIVE_FIGURE_SUB_DATA_SIZE 75
#define ROBOT_INTERACTION_PLOT_SEVEN_FIGURE_SUB_DATA_SIZE 105
#define ROBOT_INTERACTION_PLOT_CHARACTER_FIGURE_SUB_DATA_SIZE 45
#define ROBOT_INTERACTION_SENTRY_CMD_SUB_DATA_SIZE 4
#define ROBOT_INTERACTION_RADAR_CMD_SUB_DATA_SIZE 1

typedef enum
{
    GAME_ROBOT_HP_CMD_ID = 0x0003,
    ROBOT_STATUS_CMD_ID = 0x0201,

    ROBOT_INTERACTION_DATA_CMD_ID = 0x0301,

    CUSTOM_ROBOT_DATA_CMD_ID = 0x0302,
    REMOTE_CONTROL_CMD_ID = 0x0304,
} referee_cmd_id_e;

typedef enum
{
    ROBOT_ID_RED_HERO = 1,
    ROBOT_ID_RED_ENGINEER = 2,
    ROBOT_ID_RED_STANDARD_1 = 3,
    ROBOT_ID_RED_STANDARD_2 = 4,
    ROBOT_ID_RED_STANDARD_3 = 5,
    ROBOT_ID_RED_AERIAL = 6,
    ROBOT_ID_RED_SENTRY = 7,
    ROBOT_ID_RED_DART = 8,
    ROBOT_ID_RED_RADAR = 9,

    ROBOT_ID_BLUE_HERO = 101,
    ROBOT_ID_BLUE_ENGINEER = 102,
    ROBOT_ID_BLUE_STANDARD_1 = 103,
    ROBOT_ID_BLUE_STANDARD_2 = 104,
    ROBOT_ID_BLUE_STANDARD_3 = 105,
    ROBOT_ID_BLUE_AERIAL = 106,
    ROBOT_ID_BLUE_SENTRY = 107,
    ROBOT_ID_BLUE_DART = 108,
    ROBOT_ID_BLUE_RADAR = 109,

} robot_id_e;

typedef enum
{
    CLIENT_ID_RED_HERO = 0x101,
    CLIENT_ID_RED_ENGINEER = 0x102,
    CLIENT_ID_RED_STANDARD_1 = 0x103,
    CLIENT_ID_RED_STANDARD_2 = 0x104,
    CLIENT_ID_RED_STANDARD_3 = 0x105,
    CLIENT_ID_RED_AERIAL = 0x106,

    CLIENT_ID_BLUE_HERO = 0x165,
    CLIENT_ID_BLUE_ENGINEER = 0x166,
    CLIENT_ID_BLUE_STANDARD_1 = 0x167,
    CLIENT_ID_BLUE_STANDARD_2 = 0x168,
    CLIENT_ID_BLUE_STANDARD_3 = 0x169,
    CLIENT_ID_BLUE_AERIAL = 0x16A,

} client_id_e;

#define getRobotClientId(robot_id) (robot_id + 256)

typedef enum
{
    LAYER_DELETE_SUBCMD_ID = 0x0100,
    PLOT_ONE_FIGURE_SUBCMD_ID = 0x0101,
    PLOT_TWO_FIGURE_SUBCMD_ID = 0x0102,
    PLOT_FIVE_FIGURE_SUBCMD_ID = 0x0103,
    PLOT_SEVEN_FIGURE_SUBCMD_ID = 0x0104,
    PLOT_CHARACTER_FIGURE_SUBCMD_ID = 0x0110,
    SENTRY_CMD_SUBCMD_ID = 0x0120,
    RADAR_CMD_SUBCMD_ID = 0x0121,
} referee_interaction_subcmd_id_e;

typedef enum
{
    LAYER_DELETE_NOP = 0,
    LAYER_DELETE_SELECTIVELY = 1,
    LAYER_DELETE_ALL = 2,
} layer_delete_type_e;

typedef enum
{
    FIGURE_NOP = 0,
    FIGURE_ADD = 1,
    FIGURE_MODIFY = 2,
    FIGURE_DELETE = 3,
} figure_operation_type_e;

typedef enum
{
    FIGURE_LINE = 0,
    FIGURE_RECTANGLE = 1,
    FIGURE_CIRCLE = 2,
    FIGURE_ELLIPSE = 3,
    FIGURE_ARC = 4,
    FIGURE_FLOAT_NUM = 5,
    FIGURE_INT_NUM = 6,
    FIGURE_CHARACTER = 7,
} figure_shape_type_e;

typedef enum
{
    FIGURE_MAJOR_COLOR = 0,
    FIGURE_YELLOW = 1,
    FIGURE_GREEN = 2,
    FIGURE_ORANGE = 3,
    FIGURE_MAGENTA = 4,
    FIGURE_PINK = 5,
    FIGURE_CYAN = 6,
    FIGURE_BLACK = 7,
    FIGURE_WHITE = 8,
} figure_color_type_e;

#pragma pack(push, 1)

typedef struct
{
    uint8_t SOF;
    uint8_t data_length[2];
    uint8_t seq;
    uint8_t CRC8;
} frame_header_struct_t;

/**
 * @命令码 0x0003
 * @数据段长度 32
 * @说明 机器人血量数据，固定以 3Hz 频率发送
 * @发送方/接收方 服务器→全体机器人
 * @所属数据链路 常规链路
 */
typedef struct
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} game_robot_HP_t;

/**
 * @命令码 0x0201
 * @数据段长度 13
 * @说明 机器人性能体系数据，固定以 10Hz 频率发送
 * @发送方/接收方 主控模块→对应机器人
 * @所属数据链路 常规链路
 */
typedef struct
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
} robot_status_t;

/**
 * @命令码 0x0301
 * @数据段长度 128
 * @说明 机器人交互数据，发送方触发发送，频率上限为 10Hz
 * @发送方/接收方 -
 * @所属数据链路 常规链路
 * @此条由统一的数据头+数据组成，此结构体仅是统一数据头
 */
typedef struct
{
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
} robot_interaction_data_header_t;

/**
 * @子内容ID 0x0100
 * @子内容长度 2
 * @功能说明 选手端删除图层
 */
typedef struct
{
    uint8_t delete_type;
    uint8_t layer;
} interaction_layer_delete_t;

/**
 * @子内容ID 0x0101
 * @子内容长度 15
 * @功能说明 选手端绘制一个图形
 */
typedef struct
{
    uint8_t figure_name[3];
    uint32_t operate_tpye : 3;
    uint32_t figure_tpye : 3;
    uint32_t layer : 4;
    uint32_t color : 4;
    uint32_t details_a : 9;
    uint32_t details_b : 9;
    uint32_t width : 10;
    uint32_t start_x : 11;
    uint32_t start_y : 11;
    uint32_t details_c : 10;
    uint32_t details_d : 11;
    uint32_t details_e : 11;
} interaction_figure_t;

/**
 * @子内容ID 0x0102
 * @子内容长度 30
 * @功能说明 选手端绘制两个图形
 */
typedef struct
{
    interaction_figure_t figure[2];
} interaction_figure_2_t;

/**
 * @子内容ID 0x0103
 * @子内容长度 75
 * @功能说明 选手端绘制五个图形
 */
typedef struct
{
    interaction_figure_t figure[5];
} interaction_figure_3_t;

/**
 * @子内容ID 0x0104
 * @子内容长度 105
 * @功能说明 选手端绘制七个图形
 */
typedef struct
{
    interaction_figure_t figure[7];
} interaction_figure_4_t;

/**
 * @子内容ID 0x0110
 * @子内容长度 45
 * @功能说明 选手端绘制字符图形
 */
typedef struct
{
    interaction_figure_t figure;
    uint8_t data[30];
} interaction_character_t;

/**
 * @子内容ID 0x0120
 * @子内容长度 4
 * @功能说明 哨兵自主决策指令
 */
typedef struct
{
    uint32_t cmd;
} interaction_sentry_cmd_t;

/**
 * @子内容ID 0x0121
 * @子内容长度 1
 * @功能说明 雷达自主决策指令
 */
typedef struct
{
    uint8_t cmd;
} interaction_radar_cmd_t;

/**
 * @命令码 0x0302
 * @数据段长度 30
 * @说明 自定义控制器与机器人交互数据，发送方触发发送，频率上限为 30Hz
 * @发送方/接收方 自定义控制器→选手端图传连接的机器人
 * @所属数据链路 图传链路
 */
typedef struct
{
    float pose[6]; // X Y Z YAW PITCH ROLL
    uint8_t key;
    uint8_t reserved[5];
} custom_robot_data_t;

/**
 * @命令码 0x0304
 * @数据段长度 30
 * @说明 自定义控制器与机器人交互数据，发送方触发发送，频率上限为 30Hz
 * @发送方/接收方 自定义控制器→选手端图传连接的机器人
 * @所属数据链路 图传链路
 */
typedef struct
{
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    int8_t left_button_down;
    int8_t right_button_down;
    uint16_t keyboard_value;
    uint16_t reserved;
} vt_link_remote_control_t;

#pragma pack(pop)

#endif /* _REFEREE_PROTOCOL_H__ */
