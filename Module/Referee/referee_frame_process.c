#include "referee_frame_process.h"

#include "referee.h"

#include "crc8_crc16.h"

static fifo_s_t pm_fifo;                        // FIFO结构体
static uint8_t pm_fifo_buf[PM_FIFO_BUF_LENGTH]; // FIFO缓存数组

static fifo_s_t vt_fifo;                        // FIFO结构体
static uint8_t vt_fifo_buf[VT_FIFO_BUF_LENGTH]; // FIFO缓存数组

static referee_frame_processer_t referee_frame_processer; // 数据解包结构体

void refereeInitFrameProcesser(void)
{
    memset(&referee_frame_processer, 0, sizeof(referee_frame_processer_t));

    fifo_s_init(&pm_fifo, pm_fifo_buf, PM_FIFO_BUF_LENGTH);
    fifo_s_init(&vt_fifo, vt_fifo_buf, VT_FIFO_BUF_LENGTH);
}

uint8_t *referee_pack_data(uint16_t cmd_id, uint8_t *data, uint16_t data_len)
{
    referee_frame_processer.sent_data_len = REF_HEADER_CRC_CMDID_LEN + data_len;

    memset(referee_frame_processer.sent_package, 0, REF_PROTOCOL_FRAME_MAX_SIZE);

    referee_frame_processer.sent_frame_header.SOF = HEADER_SOF;

    referee_frame_processer.sent_frame_header.data_length[0] = data_len;
    referee_frame_processer.sent_frame_header.data_length[1] = 0x00;

    memcpy(referee_frame_processer.sent_package, &referee_frame_processer.sent_frame_header,
           REF_PROTOCOL_HEADER_SIZE - 1);

    referee_frame_processer.sent_frame_header.CRC8 =
        get_CRC8_check_sum(referee_frame_processer.sent_package, REF_PROTOCOL_HEADER_SIZE - 1, 0xff);

    memcpy(referee_frame_processer.sent_package, &referee_frame_processer.sent_frame_header, REF_PROTOCOL_HEADER_SIZE);

    referee_frame_processer.sent_frame_header.seq++;

    referee_frame_processer.sent_package[5] = cmd_id & 0xff;
    referee_frame_processer.sent_package[6] = (cmd_id >> 8) & 0xff;

    memcpy(referee_frame_processer.sent_package + REF_HEADER_CMDID_LEN, data, data_len);

    append_CRC16_check_sum(referee_frame_processer.sent_package, referee_frame_processer.sent_data_len);

    return referee_frame_processer.sent_package;
}

void refereeUnpackFifoData(referee_link_type_e referee_link_type)
{
    uint8_t byte = 0;
    uint8_t sof = HEADER_SOF; // 帧头
    fifo_s_t *p_fifo = NULL;

    if (referee_link_type == PM_REFEREE_LINK)
        p_fifo = &pm_fifo;
    else if (referee_link_type == VT_REFEREE_LINK)
        p_fifo = &vt_fifo;

    while (fifo_s_used(p_fifo))
    {
        byte = fifo_s_get(p_fifo);
        switch (referee_frame_processer.unpack_step)
        {
        case STEP_HEADER_SOF: {
            if (byte == sof)
            {
                referee_frame_processer.unpack_step = STEP_LENGTH_LOW;
                referee_frame_processer.received_package[referee_frame_processer.unpack_index++] = byte;
            }
            else
            {
                referee_frame_processer.unpack_index = 0;
            }
        }
        break;

        case STEP_LENGTH_LOW: {
            referee_frame_processer.received_data_len = byte;
            referee_frame_processer.received_package[referee_frame_processer.unpack_index++] = byte;
            referee_frame_processer.unpack_step = STEP_LENGTH_HIGH;
        }
        break;

        case STEP_LENGTH_HIGH: {
            referee_frame_processer.received_data_len |= (byte << 8);
            referee_frame_processer.received_package[referee_frame_processer.unpack_index++] = byte;

            if (referee_frame_processer.received_data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
            {
                referee_frame_processer.unpack_step = STEP_FRAME_SEQ;
            }
            else
            {
                referee_frame_processer.unpack_step = STEP_HEADER_SOF;
                referee_frame_processer.unpack_index = 0;
            }
        }
        break;
        case STEP_FRAME_SEQ: {
            referee_frame_processer.received_package[referee_frame_processer.unpack_index++] = byte;
            referee_frame_processer.unpack_step = STEP_HEADER_CRC8;
        }
        break;

        case STEP_HEADER_CRC8: {
            referee_frame_processer.received_package[referee_frame_processer.unpack_index++] = byte;

            if (referee_frame_processer.unpack_index == REF_PROTOCOL_HEADER_SIZE)
            {
                if (verify_CRC8_check_sum(referee_frame_processer.received_package, REF_PROTOCOL_HEADER_SIZE))
                {
                    referee_frame_processer.unpack_step = STEP_DATA_CRC16;
                }
                else
                {
                    referee_frame_processer.unpack_step = STEP_HEADER_SOF;
                    referee_frame_processer.unpack_index = 0;
                }
            }
        }
        break;

        case STEP_DATA_CRC16: {
            if (referee_frame_processer.unpack_index <
                (REF_HEADER_CRC_CMDID_LEN + referee_frame_processer.received_data_len))
            {
                referee_frame_processer.received_package[referee_frame_processer.unpack_index++] = byte;
            }
            if (referee_frame_processer.unpack_index >=
                (REF_HEADER_CRC_CMDID_LEN + referee_frame_processer.received_data_len))
            {
                referee_frame_processer.unpack_step = STEP_HEADER_SOF;
                referee_frame_processer.unpack_index = 0;

                if (verify_CRC16_check_sum(referee_frame_processer.received_package,
                                           REF_HEADER_CRC_CMDID_LEN + referee_frame_processer.received_data_len))
                {
                    uint16_t cmd_id = 0;
                    memcpy(&cmd_id, referee_frame_processer.received_package + REF_PROTOCOL_HEADER_SIZE,
                           sizeof(uint16_t));

                    referee_data_decode(referee_frame_processer.received_package, cmd_id);
                }
            }
        }
        break;

        default: {
            referee_frame_processer.unpack_step = STEP_HEADER_SOF;
            referee_frame_processer.unpack_index = 0;
        }
        break;
        }
    }
}

uint16_t getRefSentDataLen(void)
{
    return referee_frame_processer.sent_data_len;
}

fifo_s_t *get_pm_fifo(void)
{
    return &pm_fifo;
}

fifo_s_t *get_vt_fifo(void)
{
    return &vt_fifo;
}
