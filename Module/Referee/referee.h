#ifndef REFEREE_H
#define REFEREE_H

#include "protocol.h"

typedef enum {
  RED_HERO = 1,             // 红方英雄
  RED_ENGINEER = 2,         // 红方工程
  RED_STANDARD_1 = 3,       // 红方步兵1
  RED_STANDARD_2 = 4,       // 红方步兵2
  RED_STANDARD_3 = 5,       // 红方步兵3
  RED_AERIAL = 6,           // 红方无人机
  RED_SENTRY = 7,           // 红方哨兵
  RED_DART = 8,             // 红方飞镖
  RED_RADAR_STATION = 9,    // 红方雷达
  BLUE_HERO = 101,          // 蓝方英雄
  BLUE_ENGINEER = 102,      // 蓝方工程
  BLUE_STANDARD_1 = 103,    // 蓝方步兵1
  BLUE_STANDARD_2 = 104,    // 蓝方步兵2
  BLUE_STANDARD_3 = 105,    // 蓝方步兵3
  BLUE_AERIAL = 106,        // 蓝方无人机
  BLUE_SENTRY = 107,        // 蓝方哨兵
  BLUE_DART = 108,          // 蓝方飞镖
  BLUE_RADAR_STATION = 109, // 蓝方雷达
} robot_id_t;
typedef enum {
  PROGRESS_UNSTART = 0,     // 比赛未开始
  PROGRESS_PREPARE = 1,     // 准备阶段
  PROGRESS_SELFCHECK = 2,   // 自检阶段
  PROGRESS_5sCOUNTDOWN = 3, // 5s倒计时
  PROGRESS_BATTLE = 4,      // 对战中
  PROGRESS_CALCULATING = 5, // 比赛解算中
} game_progress_t;

typedef struct // 0x0001 比赛状态数据
{
  uint8_t game_type : 4;      // 比赛类型
  uint8_t game_progress : 4;  // 比赛阶段
  uint16_t stage_remain_time; // 当前阶段剩余时间
  uint64_t SyncTimeStamp;     // Unix时间
} __packed ext_game_state_t;

typedef struct // 0x0002 比赛结果数据
{
  uint8_t winner; // 比赛结果
} __packed ext_game_result_t;

typedef struct // 0x0003 机器人血量数据
{
  uint16_t red_1_robot_HP;  // 红1血量
  uint16_t red_2_robot_HP;  // 红2血量
  uint16_t red_3_robot_HP;  // 红3血量
  uint16_t red_4_robot_HP;  // 红4血量
  uint16_t red_5_robot_HP;  // 红5血量
  uint16_t red_7_robot_HP;  // 红7血量
  uint16_t red_outpost_HP;  // 红方前哨战血量
  uint16_t red_base_HP;     // 红方基地血量
  uint16_t blue_1_robot_HP; // 红1血量
  uint16_t blue_2_robot_HP; // 红2血量
  uint16_t blue_3_robot_HP; // 红3血量
  uint16_t blue_4_robot_HP; // 红4血量
  uint16_t blue_5_robot_HP; // 红5血量
  uint16_t blue_7_robot_HP; // 红7血量
  uint16_t blue_outpost_HP; // 红方前哨战血量
  uint16_t blue_base_HP;    // 红方基地血量
} __packed ext_game_robot_HP_t;

typedef struct // 人工智能挑战赛加成与惩罚区状态(未用)
{
  uint8_t F1_zone_status : 1;
  uint8_t F1_zone_buff_debuff_status : 3;
  uint8_t F2_zone_status : 1;
  uint8_t F2_zone_buff_debuff_status : 3;
  uint8_t F3_zone_status : 1;
  uint8_t F3_zone_buff_debuff_status : 3;
  uint8_t F4_zone_status : 1;
  uint8_t F4_zone_buff_debuff_status : 3;
  uint8_t F5_zone_status : 1;
  uint8_t F5_zone_buff_debuff_status : 3;
  uint8_t F6_zone_status : 1;
  uint8_t F6_zone_buff_debuff_status : 3;
  uint16_t red1_bullet_left;
  uint16_t red2_bullet_left;
  uint16_t blue1_bullet_left;
  uint16_t blue2_bullet_left;
} __packed ext_ICRA_buff_debuff_zone_status_t;

typedef struct // 0x0101 场地事件数据
{
  uint32_t event_type;
} __packed ext_event_data_t;

typedef struct // 0x0102 补给站动作标识
{
  uint8_t supply_projectile_id;   // 补给站口ID
  uint8_t supply_robot_id;        // 补弹机器人ID
  uint8_t supply_projectile_step; // 出弹口开闭状态
  uint8_t supply_projectile_num;  // 补弹数量
} __packed ext_supply_projectile_action_t;

typedef struct // 0x104 裁判警告信息
{
  uint8_t level;         // 警告等级
  uint8_t foul_robot_id; // 犯规机器人ID
} __packed ext_referee_warning_t;

typedef struct // 0x105 飞镖发射口倒计时
{
  uint8_t dart_remaining_time; // 15s倒计时
} __packed ext_dart_remaining_time_t;

typedef struct // 0x0201 比赛机器人状态
{
  uint8_t robot_id;    // 本机器人ID
  uint8_t robot_level; // 机器人等级
  uint16_t remain_HP;  // 机器人剩余血量
  uint16_t max_HP;     // 机器人上限血量

  uint16_t shooter_id1_17mm_cooling_rate;  // 机器人1号17mm枪口每秒冷却值
  uint16_t shooter_id1_17mm_cooling_limit; // 机器人1号17mm枪口热量上限
  uint16_t shooter_id1_17mm_speed_limit;   // 机器人1号17mm枪口上限速度

  uint16_t shooter_id2_17mm_cooling_rate;  // 机器人2号17mm枪口每秒冷却值
  uint16_t shooter_id2_17mm_cooling_limit; // 机器人2号17mm枪口热量上限
  uint16_t shooter_id2_17mm_speed_limit;   // 机器人2号17mm枪口上限速度

  uint16_t shooter_id1_42mm_cooling_rate;  // 机器人42mm枪口每秒冷却值
  uint16_t shooter_id1_42mm_cooling_limit; // 机器人42mm枪口热量上限
  uint16_t shooter_id1_42mm_speed_limit;   // 机器人42mm枪口上限速度

  uint16_t chassis_power_limit;           // 机器人底盘功率限制上限
  uint8_t mains_power_gimbal_output : 1;  // gimbal口输出
  uint8_t mains_power_chassis_output : 1; // chassis口输出
  uint8_t mains_power_shooter_output : 1; // shooter口输出
} __packed ext_game_robot_status_t;

typedef struct // 0x0202 实时功率热量数据
{
  uint16_t chassis_volt;                  // 底盘输出电压
  uint16_t chassis_current;               // 底盘输出电流
  float chassis_power;                    // 底盘输出功率
  uint16_t chassis_power_buffer;          // 底盘功率缓冲
  uint16_t shooter_id1_17mm_cooling_heat; // 1号17mm枪口热量
  uint16_t shooter_id2_17mm_cooling_heat; // 2号17mm枪口热量
  uint16_t shooter_id1_42mm_cooling_heat; // 42mm枪口热量
} __packed ext_power_heat_data_t;

typedef struct // 0x0203 机器人位置
{
  float x;   // 位置x坐标
  float y;   // 位置y坐标
  float z;   // 位置z坐标
  float yaw; // 位置枪口
} __packed ext_game_robot_pos_t;

typedef struct // 0x0204 机器人增益
{
  uint8_t power_rune_buff;
} __packed ext_buff_t;

typedef struct // 0x0205 空中机器人能量状态
{
  uint8_t attack_time; // 可攻击时间
} __packed aerial_robot_energy_t;

typedef struct // 0x0206 伤害状态
{
  uint8_t armor_type : 4;
  uint8_t hurt_type : 4;
} __packed ext_robot_hurt_t;

typedef struct // 0x0207 实时射击信息
{
  uint8_t bullet_type; // 子弹类型
  uint8_t shooter_id;  // 发射机构ID
  uint8_t bullet_freq; // 子弹射频
  float bullet_speed;  // 子弹射速
} __packed ext_shoot_data_t;

typedef struct // 0x0208 子弹剩余发射数
{
  uint16_t bullet_remaining_num_17mm; // 17mm子弹剩余发射数目
  uint16_t bullet_remaining_num_42mm; // 42mm子弹剩余发射数目
  uint16_t coin_remaining_num;        // 剩余金币数量
} __packed ext_bullet_remaining_t;

typedef struct // 0x0209 机器人RFID状态
{
  uint32_t rfid_status; // RFID状态
} __packed ext_rfid_status_t;

typedef struct // 0x020A 飞镖机器人客户端指令数据
{
  uint8_t dart_launch_opening_status; // 当前飞镖发射口状态
  uint8_t dart_attack_target;         // 飞镖的打击目标
  uint16_t target_change_time;        // 切换打击目标的比赛剩余时间
  uint16_t operate_launch_cmd_time;   // 最近一次操作手确定指令时的比赛剩余时间
} __packed ext_dart_client_cmd_t;

typedef struct // 0x0301 交互数据接收信息 详见裁判系统文档
{
  uint16_t data_cmd_id;
  uint16_t sender_ID;
  uint16_t receiver_ID;
} __packed ext_student_interactive_header_data_t;

typedef struct // 交互数据 0x0301
{
  uint8_t data[128]; // 机器人通信按最大带宽接收，后解包
} __packed robot_interactive_data_t;

typedef struct // 交互数据 0x0301
{
  uint8_t data[10]; // 机器人通信按最大带宽接收，后解包
} __packed Robot_Zero_Cmd_Date_t;

typedef struct // 客户端删除图形 0x0301
{
  uint8_t operate_tpye;
  uint8_t layer;
} __packed ext_client_custom_graphic_delete_t;

typedef struct // 图形数据 0x0301 详见裁判系统文档
{

  uint8_t graphic_name[3];
  uint32_t operate_tpye : 3;
  uint32_t graphic_tpye : 3;
  uint32_t layer : 4;
  uint32_t color : 4;
  uint32_t start_angle : 9;
  uint32_t end_angle : 9;
  uint32_t width : 10;
  uint32_t start_x : 11;
  uint32_t start_y : 11;
  uint32_t radius : 10;
  uint32_t end_x : 11;
  uint32_t end_y : 11;

} __packed graphic_data_struct_t;

typedef struct
{
  uint8_t graphic_name[3];
  uint32_t operate_tpye : 3;
  uint32_t graphic_tpye : 3;
  uint32_t layer : 4;
  uint32_t color : 4;
  uint32_t start_angle : 9;
  uint32_t end_angle : 9;
  uint32_t width : 10;
  uint32_t start_x : 11;
  uint32_t start_y : 11;
  float graph_Float; // 浮点数据
} __packed Float_Data;

typedef struct // 客户端绘制一个图形 0x0301
{
  graphic_data_struct_t grapic_data_struct;
} __packed ext_client_custom_graphic_single_t;

typedef struct // 客户端绘制两个图形 0x0301
{
  graphic_data_struct_t grapic_data_struct[2];
} __packed ext_client_custom_graphic_double_t;

typedef struct // 客户端绘制五个图形 0x0301
{
  graphic_data_struct_t grapic_data_struct[5];
} __packed ext_client_custom_graphic_five_t;

typedef struct // 客户端绘制七个图形 0x0301
{
  graphic_data_struct_t grapic_data_struct[7];
} __packed ext_client_custom_graphic_seven_t;

typedef struct // 客户端绘制字符 机器人间通信 0x0301
{
  graphic_data_struct_t grapic_data_struct;
  uint8_t data[30];
} __packed ext_client_custom_character_t;

typedef struct // 0x0303小地图下发信息标识
{
  float target_position_x;  // 目标x位置坐标
  float target_position_y;  // 目标y位置坐标
  float target_position_z;  // 目标z位置坐标
  uint8_t commd_keyboard;   // 键盘信息
  uint16_t target_robot_ID; // 目标机器人ID
} __packed ext_robot_command_t1;

typedef struct // 0x304 客户端下发信息
{
  int16_t mouse_x;
  int16_t mouse_y;
  int16_t mouse_z;
  int8_t left_button_down;
  int8_t right_button_down;
  uint16_t keyboard_value;
  uint16_t reserved;
} __packed ext_robot_command_t2;

typedef struct
{
  uint8_t SOF;          // 起始字节0xA5
  uint16_t Data_Length; // 数据长度
  uint8_t Seq;          // 包序号
  uint8_t CRC8;         // CRC8校验值
  uint16_t CMD_ID;      // 命令ID
} __packed UI_Packhead; // 帧头

extern void init_referee_struct_data(void);
extern void referee_data_solve(uint8_t *frame);

extern void get_chassis_power_and_buffer(float *power, float *buffer);

extern uint8_t get_robot_id(void);                  // 获取己方机器人id
extern uint8_t get_game_progress(void);             // 返回比赛阶段
extern uint16_t get_stage_remain_time(void);        // 获取比赛本阶段剩余时间
extern uint8_t get_robot_level(void);               // 获取己方机器人等级
extern uint16_t get_robot_HP(void);                 // 获取己方机器人剩余血量
extern uint16_t get_chassis_power_limit(void);      // 获取己方机器人底盘功率限制数据
extern float get_chassis_power(void);               // 获取己方机器人底盘输出功率
extern uint16_t get_chassis_power_buffer(void);     // 获取己方机器人底盘缓存功率
extern uint16_t get_shooter_cooling_heat(void);     // 获取己方机器人42mm枪口热量
extern uint16_t get_shoot_heat0_limit(void);        // 获取己方机器人42mm枪口热量上限
extern uint16_t get_shoot_heat0_cooling_rate(void); // 获取己方机器人42mm枪口热量每秒冷却值
extern uint16_t get_shoot_heat0_speed_limit(void);  // 获取己方机器人42mm枪口速度限制
extern float get_shoot_speed(void);                 // 获取己方机器人42mm枪口子弹速度

extern ext_game_robot_HP_t *get_all_member_HP(void); // 获取比赛全部成员血量

extern Robot_Zero_Cmd_Date_t *Uppack_date_Communicate(void);
#endif
