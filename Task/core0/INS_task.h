#ifndef INS_Task_H
#define INS_Task_H

#include "board.h"

#define X 0
#define Y 1
#define Z 2

typedef struct
{
    float q[4]; // 四元数估计值

    float Gyro[3];          // 角速度
    float Accel[3];         // 加速度
    float MotionAccel_b[3]; // 机体坐标加速度
    float MotionAccel_n[3]; // 绝对系加速度

    float AccelLPF; // 加速度低通滤波系数

    // 加速度在绝对系的向量表示
    float xn[3];
    float yn[3];
    float zn[3];

    float atanxz;
    float atanyz;

    // 位姿
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
} INS_t;

typedef struct
{
    uint8_t flag;

    float scale[3];

    float Yaw;
    float Pitch;
    float Roll;
} IMU_Param_t;

extern volatile bool INS_init_finished; // 陀螺仪初始化完成标志位

void INS_task(void *pvParameters);                                      // 陀螺仪解算任务
void INS_Init(void);                                                    // 初始化
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q); // 坐标系转换
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q); // 坐标系转换
INS_t *get_INS_data_point(void);                                        // 返回解算结果结构体

#endif
