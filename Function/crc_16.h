//
// Created by XianY on 2023/5/10.
//

#ifndef ULTRA_VISION_FRAMEWORK_CRC_16_H
#define ULTRA_VISION_FRAMEWORK_CRC_16_H

#include "array.h"

extern const unsigned char CrcTableHigh[];

extern const unsigned char CrcTableLow[];

Bytes CrC16Encode(Bytes *buffer);

bool Crc16Verify(Bytes *buffer, Bytes *to_check);

#endif // ULTRA_VISION_FRAMEWORK_CRC_16_H
