#include "fifo.h"

bool fifo_s_init(fifo_s_t *p_fifo, void *p_base_addr, int uint_cnt) {
  // 检查输入参数
  if (p_fifo == NULL || p_base_addr == NULL || uint_cnt <= 0)
    return false;
  // 初始化FIFO控制模块。
  p_fifo->p_start_addr = (char *)p_base_addr;
  p_fifo->p_end_addr = (char *)p_base_addr + uint_cnt - 1;
  p_fifo->free_num = uint_cnt;
  p_fifo->used_num = 0;
  p_fifo->p_read_addr = (char *)p_base_addr;
  p_fifo->p_write_addr = (char *)p_base_addr;

  return true;
}

int fifo_s_used(fifo_s_t *p_fifo) {
  if (p_fifo == NULL)
    return (-1);
  return p_fifo->used_num;
}

char fifo_s_get(fifo_s_t *p_fifo) {
  if (p_fifo == NULL)
    return (-1);
  char retval = 0;
  if (p_fifo->p_read_addr > p_fifo->p_end_addr)
    p_fifo->p_read_addr = p_fifo->p_start_addr;
  retval = *p_fifo->p_read_addr;
  p_fifo->p_read_addr++;
  p_fifo->free_num++;
  p_fifo->used_num--;
  return (retval);
}

int fifo_s_puts(fifo_s_t *p_fifo, char *p_source, int len) {
  if (p_fifo == NULL || p_source == NULL || p_fifo->free_num == 0)
    return (-1);

  int retval, len_to_end, len_from_start;

  if (p_fifo->p_write_addr > p_fifo->p_end_addr)
    p_fifo->p_write_addr = p_fifo->p_start_addr;

  len = (len < p_fifo->free_num) ? len : p_fifo->free_num;
  len_to_end = p_fifo->p_end_addr - p_fifo->p_write_addr + 1;

  if (len_to_end >= len) // no rollback
  {
    len_to_end = len;
    memcpy(p_fifo->p_write_addr, p_source, len_to_end);
    p_fifo->p_write_addr += len_to_end;
  } else // rollback
  {
    len_from_start = len - len_to_end;
    memcpy(p_fifo->p_write_addr, p_source, len_to_end);
    memcpy(p_fifo->p_start_addr, p_source + len_to_end, len_from_start);
    p_fifo->p_write_addr = p_fifo->p_start_addr + len_from_start;
  }

  p_fifo->free_num -= len;
  p_fifo->used_num += len;
  retval = len;

  return retval;
}
