#ifndef _REFEREE_FRAME_PROCESS__
#define _REFEREE_FRAME_PROCESS__

#include "referee_protocol.h"

#include "fifo.h"

#define PM_FIFO_BUF_LENGTH 1024 // 电管链路裁判系统数据fifo数组大小
#define VT_FIFO_BUF_LENGTH 96   // 图传链路裁判系统数据fifo数组大小

typedef enum RefereeLinkType
{
    PM_REFEREE_LINK = 0,
    VT_REFEREE_LINK,
} referee_link_type_e;

typedef enum
{
    STEP_HEADER_SOF = 0,
    STEP_LENGTH_LOW = 1,
    STEP_LENGTH_HIGH = 2,
    STEP_FRAME_SEQ = 3,
    STEP_HEADER_CRC8 = 4,
    STEP_DATA_CRC16 = 5,
} unpack_step_e;

typedef struct
{
    frame_header_struct_t sent_frame_header;
    uint16_t sent_data_len;
    uint8_t sent_package[REF_PROTOCOL_FRAME_MAX_SIZE];

    uint16_t received_data_len;
    uint8_t received_package[REF_PROTOCOL_FRAME_MAX_SIZE];
    unpack_step_e unpack_step;
    uint16_t unpack_index;
} referee_frame_processer_t;

extern void refereeInitFrameProcesser(void);

extern uint8_t *referee_pack_data(uint16_t cmd_id, uint8_t data_len, uint8_t *data);
extern void refereeUnpackFifoData(referee_link_type_e referee_link_type);

extern uint16_t getRefSentDataLen(void);

extern fifo_s_t *get_pm_fifo(void);
extern fifo_s_t *get_vt_fifo(void);

#endif /* _REFEREE_FRAME_PROCESS__ */
