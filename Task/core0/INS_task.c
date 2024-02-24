#include "INS_task.h"
#include "BMI088Middleware.h"
#include "BMI088driver.h"
#include "FreeRTOS.h"
#include "algo_ahrs_quaternionekf.h"
#include "cnu_hpm_bsp_cyc.h"
#include "hpm_math.h"
#include "task.h"

#include "Detect_task.h"

volatile bool INS_init_finished = false; // 陀螺仪初始化完成标志位

IMU_Data_t BMI088;     // bmi088原始数据结构体
INS_t INS;             // 解算结果结构体
IMU_Param_t IMU_Param; // 安装误差参数结构体

const float gravity[3] = {0, 0, 9.81f}; // 重力加速度向量
const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

// 计时器相关
static uint32_t INS_CYC_Count = 0;
static float dt = 0, t = 0;

// 返回陀螺仪数据地址
/*
  假设云台向前，角度变化规律：
  YAW：俯视，逆时针增大，顺时针减小
  PITCH：抬头增大，低头减小
  ROLL：左倾增大，右倾减小

  加速度变化规律：
  YAW：gyro[2]，左加右减
  PITCH：gyro[1]，上加下减
  ROLL：gyro[3]，左加右减
 */
INS_t *get_INS_data_point(void) {
  return &INS;
}

// 结构体参数初始化
static void INS_Param_Init(void);
// 用于修正IMU安装误差与标度因数误差,即陀螺仪轴和云台轴的安装偏移
static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3]);

// 陀螺仪姿态解算任务
void INS_task(void *pvParameters) {
  INS_Param_Init();             // 参数初始化
  while (BMI088_init() != true) // 初始化BMI088
    ;
  Calibrate_MPU_Offset(&BMI088); // 计算零飘
  board_beep_open();
  vTaskDelay(500); // 蜂鸣器提示校准完成
  board_beep_close();
  INS_init_finished = true;                   // 至此陀螺仪初始化完成，更新标志位
  TickType_t last_tick = xTaskGetTickCount(); // 获取当前时间
  while (true) {
    dt = GetDeltaT(&INS_CYC_Count); // 获取时间
    t += dt;
    BMI088_Read(&BMI088); // 读取数据
    INS.Accel[X] = BMI088.Accel[X];
    INS.Accel[Y] = BMI088.Accel[Y];
    // INS.Accel[Z] = 0.5f * INS.Accel[Z] + 0.5f * BMI088.Accel[Z];
    INS.Accel[Z] = BMI088.Accel[Z];
    INS.Gyro[X] = BMI088.Gyro[X];
    INS.Gyro[Y] = BMI088.Gyro[Y];
    INS.Gyro[Z] = BMI088.Gyro[Z];

    // 用于修正安装误差,可以不管,本demo暂时没用
    IMU_Param_Correction(&IMU_Param, INS.Gyro, INS.Accel);

    // 计算重力加速度矢量和 b系的XY两轴的夹角,可用作功能扩展,本demo暂时没用
    INS.atanxz = -atan2f(INS.Accel[X], INS.Accel[Z]) * 180 / PI;
    INS.atanyz = atan2f(INS.Accel[Y], INS.Accel[Z]) * 180 / PI;

    // 核心函数,EKF更新四元数
    IMU_QuaternionEKF_Update(INS.Gyro[X], INS.Gyro[Y], INS.Gyro[Z], INS.Accel[X], INS.Accel[Y], INS.Accel[Z], dt);

    memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));
    // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
    BodyFrameToEarthFrame(xb, INS.xn, INS.q);
    BodyFrameToEarthFrame(yb, INS.yn, INS.q);
    BodyFrameToEarthFrame(zb, INS.zn, INS.q);

    // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
    float gravity_b[3];
    EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
    for (uint8_t i = 0; i < 3; i++) // 同样过一个低通滤波
    {
      INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
    }
    BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 转换回导航系n

    // 获取最终数据（将角度转为弧度，并且根据安装方向改变roll和pitch）
    INS.Yaw = QEKF_INS.Yaw;
    INS.Pitch = QEKF_INS.Roll;
    INS.Roll = QEKF_INS.Pitch;
    INS.YawTotalAngle = QEKF_INS.YawTotalAngle;
    vTaskDelayUntil(&last_tick, 1);
  }
}

// 结构体参数初始化
static void INS_Param_Init(void) {
  IMU_Param.scale[X] = 1;
  IMU_Param.scale[Y] = 1;
  IMU_Param.scale[Z] = 1;
  IMU_Param.Yaw = 0;
  IMU_Param.Pitch = 0;
  IMU_Param.Roll = 0;
  IMU_Param.flag = 1;
  IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0);
  INS.AccelLPF = 0.0085;
}

/**
 * @brief reserved.用于修正IMU安装误差与标度因数误差,即陀螺仪轴和云台轴的安装偏移
 *
 *
 * @param param IMU参数
 * @param gyro  角速度
 * @param accel 加速度
 */
static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3]) {
  static float lastYawOffset, lastPitchOffset, lastRollOffset;
  static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
  float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

  if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
      fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
      fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag) {

    cosYaw = hpm_dsp_cos_f32(param->Yaw / 57.295779513f);
    cosPitch = hpm_dsp_cos_f32(param->Pitch / 57.295779513f);
    cosRoll = hpm_dsp_cos_f32(param->Roll / 57.295779513f);
    sinYaw = hpm_dsp_sin_f32(param->Yaw / 57.295779513f);
    sinPitch = hpm_dsp_sin_f32(param->Pitch / 57.295779513f);
    sinRoll = hpm_dsp_sin_f32(param->Roll / 57.295779513f);

    // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
    c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
    c_12 = cosPitch * sinYaw;
    c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
    c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
    c_22 = cosYaw * cosPitch;
    c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
    c_31 = -cosPitch * sinRoll;
    c_32 = sinPitch;
    c_33 = cosPitch * cosRoll;
    param->flag = 0;
  }
  float gyro_temp[3];
  for (uint8_t i = 0; i < 3; i++)
    gyro_temp[i] = gyro[i] * param->scale[i];

  gyro[X] = c_11 * gyro_temp[X] +
            c_12 * gyro_temp[Y] +
            c_13 * gyro_temp[Z];
  gyro[Y] = c_21 * gyro_temp[X] +
            c_22 * gyro_temp[Y] +
            c_23 * gyro_temp[Z];
  gyro[Z] = c_31 * gyro_temp[X] +
            c_32 * gyro_temp[Y] +
            c_33 * gyro_temp[Z];

  float accel_temp[3];
  for (uint8_t i = 0; i < 3; i++)
    accel_temp[i] = accel[i];

  accel[X] = c_11 * accel_temp[X] +
             c_12 * accel_temp[Y] +
             c_13 * accel_temp[Z];
  accel[Y] = c_21 * accel_temp[X] +
             c_22 * accel_temp[Y] +
             c_23 * accel_temp[Z];
  accel[Z] = c_31 * accel_temp[X] +
             c_32 * accel_temp[Y] +
             c_33 * accel_temp[Z];

  lastYawOffset = param->Yaw;
  lastPitchOffset = param->Pitch;
  lastRollOffset = param->Roll;
}

void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q) {
  vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                        (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                        (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

  vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                        (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                        (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

  vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                        (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                        (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q) {
  vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                        (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                        (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

  vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                        (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                        (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

  vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                        (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                        (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}
