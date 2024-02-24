#include "referee.h"
#include "CRC8_CRC16.h"
#include "protocol.h"
#include "stdio.h"
#include "string.h"

frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

ext_game_state_t game_state;         // 0x0001
ext_game_result_t game_result;       // 0x0002
ext_game_robot_HP_t game_robot_HP_t; // 0x0003

ext_event_data_t field_event;                              // 0x0101
ext_supply_projectile_action_t supply_projectile_action_t; // 0x0102
ext_referee_warning_t referee_warning_t;                   // 0x0104
ext_dart_remaining_time_t dart_remaining_time_t;           // 0x0105

ext_game_robot_status_t game_robot_status_t; // 0x0201
ext_power_heat_data_t power_heat_data_t;     // 0x0202
ext_game_robot_pos_t game_robot_pos_t;       // 0x0203
ext_buff_t buff_t;                           // 0x0204
aerial_robot_energy_t Aerial_robot_energy_t; // 0x0205
ext_robot_hurt_t robot_hurt_t;               // 0x0206
ext_shoot_data_t shoot_data_t;               // 0x0207
ext_bullet_remaining_t bullet_remaining_t;   // 0x0208
ext_rfid_status_t rfid_status_t;             // 0x0209
ext_dart_client_cmd_t dart_client_cmd_t;     // 0x020A

robot_interactive_data_t Robot_interactive_data1_t; // 0x0301robot_interactive_data_t
Robot_Zero_Cmd_Date_t Date_Zero;

void init_referee_struct_data(void) {
  memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
  memset(&referee_send_header, 0, sizeof(frame_header_struct_t));

  memset(&game_state, 0, sizeof(ext_game_state_t));
  memset(&game_result, 0, sizeof(ext_game_result_t));
  memset(&game_robot_HP_t, 0, sizeof(ext_game_robot_HP_t));

  memset(&field_event, 0, sizeof(ext_event_data_t));
  memset(&supply_projectile_action_t, 0, sizeof(ext_supply_projectile_action_t));
  memset(&referee_warning_t, 0, sizeof(ext_referee_warning_t));
  memset(&dart_remaining_time_t, 0, sizeof(ext_dart_remaining_time_t));

  memset(&game_robot_status_t, 0, sizeof(ext_game_robot_status_t));
  memset(&power_heat_data_t, 0, sizeof(ext_power_heat_data_t));
  memset(&game_robot_pos_t, 0, sizeof(ext_game_robot_pos_t));
  memset(&buff_t, 0, sizeof(ext_buff_t));
  memset(&Aerial_robot_energy_t, 0, sizeof(aerial_robot_energy_t));
  memset(&robot_hurt_t, 0, sizeof(ext_robot_hurt_t));
  memset(&shoot_data_t, 0, sizeof(ext_shoot_data_t));
  memset(&bullet_remaining_t, 0, sizeof(ext_bullet_remaining_t));
  memset(&rfid_status_t, 0, sizeof(ext_rfid_status_t));
  memset(&dart_client_cmd_t, 0, sizeof(ext_dart_client_cmd_t));
}

void referee_data_solve(uint8_t *frame) {
  uint16_t cmd_id = 0;

  uint8_t index = 0;

  memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

  index += sizeof(frame_header_struct_t);

  memcpy(&cmd_id, frame + index, sizeof(uint16_t));
  index += sizeof(uint16_t);

  switch (cmd_id) {

  case GAME_STATE_CMD_ID: {
    memcpy(&game_state, frame + index, sizeof(ext_game_state_t));
  } break;
  case GAME_RESULT_CMD_ID: {
    memcpy(&game_result, frame + index, sizeof(ext_game_result_t));
  } break;
  case GAME_ROBOT_HP_CMD_ID: {
    memcpy(&game_robot_HP_t, frame + index, sizeof(ext_game_robot_HP_t));
  } break;
  case FIELD_EVENTS_CMD_ID: {
    memcpy(&field_event, frame + index, sizeof(ext_event_data_t));
  } break;
  case SUPPLY_PROJECTILE_ACTION_CMD_ID: {
    memcpy(&supply_projectile_action_t, frame + index, sizeof(ext_supply_projectile_action_t));
  } break;
  case REFEREE_WARNING_CMD_ID: {
    memcpy(&referee_warning_t, frame + index, sizeof(ext_referee_warning_t));
  } break;
  case DART_REMAINING_TIME_CMD_ID: {
    memcpy(&dart_remaining_time_t, frame + index, sizeof(ext_dart_remaining_time_t));
  } break;

  case ROBOT_STATE_CMD_ID: {
    memcpy(&game_robot_status_t, frame + index, sizeof(ext_game_robot_status_t));
  } break;
  case POWER_HEAT_DATA_CMD_ID: {
    memcpy(&power_heat_data_t, frame + index, sizeof(ext_power_heat_data_t));
  } break;
  case ROBOT_POS_CMD_ID: {
    memcpy(&game_robot_pos_t, frame + index, sizeof(ext_game_robot_pos_t));
  } break;
  case BUFF_CMD_ID: {
    memcpy(&buff_t, frame + index, sizeof(ext_buff_t));
  } break;
  case AERIAL_ROBOT_ENERGY_CMD_ID: {
    memcpy(&Aerial_robot_energy_t, frame + index, sizeof(aerial_robot_energy_t));
  } break;
  case ROBOT_HURT_CMD_ID: {
    memcpy(&robot_hurt_t, frame + index, sizeof(ext_robot_hurt_t));
  } break;
  case SHOOT_DATA_CMD_ID: {
    memcpy(&shoot_data_t, frame + index, sizeof(ext_shoot_data_t));
  } break;
  case BULLET_REMAINING_CMD_ID: {
    memcpy(&bullet_remaining_t, frame + index, sizeof(ext_bullet_remaining_t));
  } break;
  case RFID_STATUS_CMD_ID: {
    memcpy(&rfid_status_t, frame + index, sizeof(ext_rfid_status_t));
  } break;
  case STUDENT_INTERACTIVE_DATA_CMD_ID: {
    memcpy(&Date_Zero, frame + index, 25);
  } break;

  default: {
    break;
  }
  }
}
Robot_Zero_Cmd_Date_t *Uppack_date_Communicate(void) // 8.9位自定义数据id
{
  // int date_lenth;//定义数据长度
  uint16_t Date_Cmd_ID; // date_lenth = Robot_interactive_data1_t.data
  uint8_t *framepoint;  // memcpy连续拷贝指针1
  uint8_t *framepoint2; // memcpy连续拷贝指针2

  framepoint = (unsigned char *)&Robot_interactive_data1_t;
  memcpy(&Date_Cmd_ID, framepoint + 7, 2); // 填充自定义命令

  if (Date_Cmd_ID == 0x0200) {
    framepoint = (unsigned char *)&Date_Zero;
    framepoint2 = (unsigned char *)&Robot_interactive_data1_t;
    memcpy(framepoint, framepoint2 + 13, 2); // 填充自定义命令
  }
  memset(&Robot_interactive_data1_t, 0, sizeof(Robot_interactive_data1_t));

  return &Date_Zero;
}

uint8_t get_robot_id(void) // 获取己方机器人id
{
  return game_robot_status_t.robot_id;
}
uint8_t get_game_progress(void) // 返回比赛阶段
{
  return game_state.game_progress;
}
uint8_t get_robot_level(void) // 获取己方机器人等级
{
  return game_robot_status_t.robot_level;
}
uint16_t get_robot_HP(void) // 获取己方机器人剩余血量
{
  return game_robot_status_t.remain_HP;
}
uint16_t get_chassis_power_limit(void) // 获取己方机器人底盘功率限制数据
{
  return game_robot_status_t.chassis_power_limit;
}
float get_chassis_power(void) // 获取己方机器人底盘输出功率
{
  return power_heat_data_t.chassis_power;
}
uint16_t get_chassis_power_buffer(void) // 获取己方机器人底盘缓冲功率
{
  return power_heat_data_t.chassis_power_buffer;
}
uint16_t get_shooter_cooling_heat(void) // 获取己方机器人枪口热量
{
  return power_heat_data_t.shooter_id1_42mm_cooling_heat;
}
uint16_t get_shoot_heat0_limit(void) // 获取己方机器人枪口热量上限
{
  return game_robot_status_t.shooter_id1_42mm_cooling_limit;
}
uint16_t get_shoot_heat0_cooling_rate(void) // 获取己方机器人枪口热量每秒冷却值
{
  return game_robot_status_t.shooter_id1_42mm_cooling_rate;
}
uint16_t get_shoot_heat0_speed_limit(void) // 获取己方机器人枪口速度限制
{
  return game_robot_status_t.shooter_id1_42mm_speed_limit;
}
float get_shoot_speed(void) // 获取己方机器人枪口子弹速度
{
  return shoot_data_t.bullet_speed;
}
uint16_t get_stage_remain_time(void) // 返回比赛状态
{
  return game_state.stage_remain_time;
}
// 返回比赛所有成员血量指针
ext_game_robot_HP_t *get_all_member_HP(void) {
  return &game_robot_HP_t;
}
