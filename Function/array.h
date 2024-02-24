//
// Created by XianY on 2023/5/10.
//

#ifndef ULTRA_VISION_FRAMEWORK_ARRAY_H
#define ULTRA_VISION_FRAMEWORK_ARRAY_H

#include "stdbool.h"

typedef struct {
    unsigned char data[64];
    int len;
} Bytes;

unsigned char *BytesAt(Bytes *bytes, int index);

bool BytesEqual(Bytes *bytes1, Bytes *bytes2);

#endif //ULTRA_VISION_FRAMEWORK_ARRAY_H
