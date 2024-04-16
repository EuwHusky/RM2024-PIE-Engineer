#include "stdio.h"
#include "string.h"

#include "referee.h"

static game_robot_HP_t game_robot_HP;
static robot_status_t robot_status;
static custom_robot_data_t customer_controller_data;
static vt_link_remote_control_t vt_link_remote_control;

void refereeInitData(void)
{
    memset(&game_robot_HP, 0, sizeof(game_robot_HP_t));
    memset(&robot_status, 0, sizeof(robot_status_t));

    memset(&customer_controller_data, 0, sizeof(custom_robot_data_t));
    memset(&vt_link_remote_control, 0, sizeof(vt_link_remote_control_t));
}

void referee_data_decode(uint8_t *frame, uint16_t cmd_id)
{
    uint8_t index = REF_HEADER_CMDID_LEN;

    switch (cmd_id)
    {
        /* 常规链路 */

    case GAME_ROBOT_HP_CMD_ID: {
        memcpy(&game_robot_HP, frame + index, sizeof(game_robot_HP_t));
        break;
    }
    case ROBOT_STATUS_CMD_ID: {
        memcpy(&robot_status, frame + index, sizeof(robot_status_t));
        break;
    }

        /* 图传链路 */

    case CUSTOM_ROBOT_DATA_CMD_ID: {
        uint32_t temp;
        float temp_out;
        temp = ((frame[10] << 24) | (frame[9] << 16) | (frame[8] << 8) | frame[7]);
        temp_out = *(float *)&temp;
        customer_controller_data.pose[0] = temp_out;
        temp = ((frame[14] << 24) | (frame[13] << 16) | (frame[12] << 8) | frame[11]);
        temp_out = *(float *)&temp;
        customer_controller_data.pose[1] = temp_out;
        temp = ((frame[18] << 24) | (frame[17] << 16) | (frame[16] << 8) | frame[15]);
        temp_out = *(float *)&temp;
        customer_controller_data.pose[2] = temp_out;
        temp = ((frame[22] << 24) | (frame[21] << 16) | (frame[20] << 8) | frame[19]);
        temp_out = *(float *)&temp;
        customer_controller_data.pose[3] = temp_out;
        temp = ((frame[26] << 24) | (frame[25] << 16) | (frame[24] << 8) | frame[23]);
        temp_out = *(float *)&temp;
        customer_controller_data.pose[4] = temp_out;
        temp = ((frame[30] << 24) | (frame[29] << 16) | (frame[28] << 8) | frame[27]);
        temp_out = *(float *)&temp;
        customer_controller_data.pose[5] = temp_out;
        customer_controller_data.key = frame[31];
        customer_controller_data.reserved[0] = frame[32];
        customer_controller_data.reserved[1] = frame[33];
        customer_controller_data.reserved[2] = frame[34];
        customer_controller_data.reserved[3] = frame[35];
        customer_controller_data.reserved[4] = frame[36];
        break;
    }
    case REMOTE_CONTROL_CMD_ID: {
        memcpy(&vt_link_remote_control, frame + index, sizeof(vt_link_remote_control_t));
        break;
    }

    default:
        break;
    }
}

const game_robot_HP_t *getRobotHp(void)
{
    return &game_robot_HP;
}

const robot_status_t *getRobotStatus(void)
{
    return &robot_status;
}

const uint8_t getRobotStatusId(void)
{
    return robot_status.robot_id;
}

const uint16_t getRobotCurrentHp(void)
{
    return robot_status.current_HP;
}

const custom_robot_data_t *getCustomerControllerData(void)
{
    return &customer_controller_data;
}

const vt_link_remote_control_t *getVtLinkRemoteControlData(void)
{
    return &vt_link_remote_control;
}
