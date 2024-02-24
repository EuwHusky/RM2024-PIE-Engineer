
#ifndef __FIFO_H__
#define __FIFO_H__

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

typedef struct
{
  char *p_start_addr; // FIFO起始地址
  char *p_end_addr;   // FIFO终止地址
  int free_num;       // FIFO的剩余容量
  int used_num;       // FIFO中的元素数量
  char *p_read_addr;  // FIFO 数据读取索引指针
  char *p_write_addr; // FIFO 数据写入索引指针
} fifo_s_t;

// 初始化静态FIFO结构
bool fifo_s_init(fifo_s_t *p_fifo, void *p_base_addr, int uint_cnt);
// 获取FIFO的元素数量
int fifo_s_used(fifo_s_t *p_fifo);
// 从FIFO获取元素
char fifo_s_get(fifo_s_t *p_fifo);
// 将元素放入FIFO中
int fifo_s_puts(fifo_s_t *p_fifo, char *p_source, int len);

#endif // __FIFO_H__
