#include "jy901.h"

#include <string.h>


#define JY901_HEADER            0x55    /* 数据头 */
#define JY901_LENGTH            11      /* 数据包长度 */

#define JY901_MSG_ACCELERATION  0x51    /* 加速度数据包 */
#define JY901_MSG_ANGULAR       0x52    /* 角速度数据包 */
#define JY901_MSG_RPY           0x53    /* 姿态角数据包 */
#define JY901_MSG_MAGNETIC      0x54    /* 磁场数据包 */
#define JY901_MSG_GNSS          0x57    /* 经纬度数据包 */

#define JY901_RATIO_ACCELERATION    2048.0  /* 采样值转转换到实际值比例分母 加速度 */
#define JY901_RATIO_ANGULAR         16.384  /* 采样值转转换到实际值比例分母 角速度 */
#define JY901_RATIO_RPY             182.04  /* 采样值转转换到实际值比例分母 姿态角 */
#define JY901_RATIO_MAGNETIC        1.0     /* 采样值转转换到实际值比例分母 磁场数据 */
#define JY901_RATIO_GNSS            1.0     /* 采样值转转换到实际值比例分母 经纬度 */


/* internal 用于存储已经获取的哪些数据，哪些数据还没有，bit 存储，以加速度计数据为锚点 */
#define INTER_FLAG_ACCE     1
#define INTER_FLAG_GYRO     2
#define INTER_FLAG_RPY      4
#define INTER_FLAG_MAGNET   8

#define INTER_FLAG_ENOUGHT  7
#define INTER_IF_ENOUGH(x)  ((x & INTER_FLAG_ENOUGHT) == INTER_FLAG_ENOUGHT)


static short jy901_get_i16(unsigned char b0, unsigned char b1)
{
    union {
        unsigned char b[2];
        short i16;
    } conv;
    conv.b[0] = b0;
    conv.b[1] = b1;
    return conv.i16;
 }


void jy901_imubuf_init(imubuf_t *imubuf)
{
    imubuf->idx = 0;
    imubuf->_internal = 0;
}

int  jy901_imubuf_input(imubuf_t *imubuf, const unsigned char *data, size_t sz)
{
    if (imubuf == NULL || data == NULL || sz == 0 || (imubuf->idx + sz) > sizeof(imubuf->buff)) {
        return -1;
    }
    /* 追加数据 */
    memcpy(imubuf->buff + imubuf->idx, data, sz);
    imubuf->idx += sz;
    /* 解析 */
    size_t idx = 0;
    unsigned char* buf = imubuf->buff;
    while ((imubuf->idx - idx) >= JY901_LENGTH) {
        /* 同步头 */
        if (buf[idx] != JY901_HEADER) {
            idx += 1;
            continue;
        }
        /* 检查校验 */
        unsigned char* data = buf + idx;
        unsigned char sum = 0;
        for (int i = 0; i < (JY901_LENGTH - 1); i++) {
            sum += data[i];
        }
        if (sum != data[JY901_LENGTH - 1]) {
            idx += 1;
            continue;
        }
        /* 解析数据 */
        short d1 = jy901_get_i16(data[2], data[3]);
        short d2 = jy901_get_i16(data[4], data[5]);
        short d3 = jy901_get_i16(data[6], data[7]);
        switch (data[1]) {
        case JY901_MSG_ACCELERATION:
            imubuf->accex = d1 / JY901_RATIO_ACCELERATION;
            imubuf->accey = d2 / JY901_RATIO_ACCELERATION;
            imubuf->accez = d3 / JY901_RATIO_ACCELERATION;
            /* 重置 internal 标记 */
            imubuf->_internal = INTER_FLAG_ACCE;
            break;
        case JY901_MSG_ANGULAR:
            imubuf->gyrox = d1 / JY901_RATIO_ANGULAR;
            imubuf->gyroy = d2 / JY901_RATIO_ANGULAR;
            imubuf->gyroz = d3 / JY901_RATIO_ANGULAR;
            imubuf->_internal |= INTER_FLAG_GYRO;
            break;
        case JY901_MSG_RPY:
            imubuf->roll  = d1 / JY901_RATIO_RPY;
            imubuf->pitch = d2 / JY901_RATIO_RPY;
            imubuf->yaw   = d3 / JY901_RATIO_RPY;
            imubuf->_internal |= INTER_FLAG_RPY;
            break;
        case JY901_MSG_MAGNETIC:
            /*do nothing */
            break;
        default:
            break;
        }
        idx += JY901_LENGTH;
    }
    /* 清除已经解析过的数据 */
    if (idx > 0) {
        memmove(buf, buf + idx, imubuf->idx - idx);
        imubuf->idx -= idx;
    }
    /* 判断数据是否有效 */
    if (INTER_IF_ENOUGH(imubuf->_internal)) {
        imubuf->_internal = 0;
        return 1;
    } else {
        return 0;
    }
}
