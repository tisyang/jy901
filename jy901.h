#ifndef JY901_H
#define JY901_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float accex; /* 9轴传感器x轴加速度 */
    float accey; /* 9轴传感器y轴加速度 */
    float accez; /* 9轴传感器z轴加速度 */

    float gyrox; /* 9轴传感器x轴角速度 */
    float gyroy; /* 9轴传感器y轴角速度 */
    float gyroz; /* 9轴传感器z轴角速度 */

    float roll;  /* 9轴传感器翻滚角 */
    float pitch; /* 9轴传感器俯仰角 */
    float yaw;   /* 9轴传感器航向角 */

    unsigned char buff[512]; /* 数据缓冲区 */
    size_t idx;              /* 缓冲区使用字节数目 */
    unsigned char _internal; /* 内部使用数据 */
} imubuf_t;

/**
 * @brief 初始化一个 imubuf_t
 * 
 * @param imubuf imubuf_t 地址
 */
void jy901_imubuf_init(imubuf_t *imubuf);

/**
 * @brief 向 imubuf_t 写入要读取到的IMU原始数据
 *        请注意单次不要写入超过缓冲区大小的数据
 * 
 * @param imubuf imubut_t 地址
 * @param data   要写入的字节起始地址
 * @param sz     字节数目
 * @return int   返回 0 表示没有完整数据解析，返回 1 表示有完整数据解析出
 *               返回 -1 表示错误，超过缓冲区大小/参数无效
 */
int  jy901_imubuf_input(imubuf_t *imubuf, const unsigned char *data, size_t sz);


#ifdef __cplusplus
}
#endif

#endif /* JY901_H */

