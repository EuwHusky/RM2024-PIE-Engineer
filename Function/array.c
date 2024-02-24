//
// Created by XianY on 2023/5/2.
//

#include "stddef.h"
#include "array.h"

unsigned char *BytesAt(Bytes *bytes, int index) {
    if (index >= bytes->len) {
        return NULL;
    }
    return bytes->data + index;
}

bool BytesEqual(Bytes *bytes1, Bytes *bytes2) {
    if (bytes1->len != bytes2->len) {
        return false;
    }
    for (int i = 0; i < bytes1->len; ++i) {
        if (bytes1->data[i] != bytes2->data[i]) {
            return false;
        }
    }
    return true;
}
