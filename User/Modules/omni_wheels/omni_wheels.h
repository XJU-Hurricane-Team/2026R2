 /**
 * @file    omni_wheels.h
 * @author  Dominate0017
 * @brief   全向轮运动学模块
 * @version 0.1
 * @date    2026-04-2
 */

#ifndef OMNI_WHEELS_H
#define OMNI_WHEELS_H

#include <stdint.h>
#include "DJI-Motor/dji_bldc_motor.h"
#include "pid/pid.h"
#include "fdcan.h"

// 宏观小车机械参数定义 (单位：米)
#define CHASSIS_RADIUS 0.25f       // 旋转中心到轮子的距离
#define WHEEL_RADIUS   0.076f      // 轮子半径 152mm/2 = 76mm
#define GEAR_RATIO     19.0f       // M3508减速比


#define PI 3.1415926535f
#define SQRT2_2 0.70710678f // sqrt(2)/2 的常数，用于45度麦克纳姆/全向轮运动学投影

// 速度限制参数
#define MAX_SPEED_XY             1.0f
#define MAX_SPEED_W              4.0f
#define MAX_ACCEL_XY             1.5f
#define MAX_ACCEL_W              2.0f

// 宏定义：轮子线速度(m/s)转化为电机转子转速(rpm)
// v = (rpm / 60) * (1 / GEAR_RATIO) * (2 * PI * WHEEL_RADIUS)
// rpm = v * 60 * GEAR_RATIO / (2 * PI * WHEEL_RADIUS)
#define MPS_TO_RPM(v) ((v) * 60.0f * GEAR_RATIO / (2.0f * PI * WHEEL_RADIUS))
#define RPM_TO_MPS(rpm) ((rpm) * 2.0f * PI * WHEEL_RADIUS / (60.0f * GEAR_RATIO))

// 规划后的目标速度
typedef struct {
    float vx;  // x轴平移速度 (m/s)
    float vy;  // y轴平移速度 (m/s)
    float vw;  // 偏航角速度 (rad/s)
} chassis_slope_t; // 线速度结构体，单位 m/s

// 底盘各量纲所需的速度
typedef struct {
    float target_rpm[4];               /* 量纲：规划后每个轮子的转速，单位rpm */
} chassis_speed_t;

extern volatile float wheel_rpm[4]; // 4个轮子的转子转速 (rpm)

/**
 * @brief 世界坐标系速度向车身坐标系速度转换
 * @param speed 指向底盘目标线速度结构体的指针
 * @param yaw_angle 当前小车的偏航角(Yaw)，单位rad
 */
void omni_wheels_world_transform(chassis_slope_t *speed, float yaw_angle);

/**
 * @brief 将底盘宏观速度解算为4个车轮的转子目标转速
 * @param chassis_speed 宏观速度输入
 * @param wheel_speed 轮子转速输出
 */
void omni_wheels_resolve(const chassis_slope_t *target_speed, volatile float *out_wheel_rpm);

/**
 * @brief 综合控制接口：逆解算 + PID + 输出控制电流
 * @param ch_tgt 目标宏观速度
 * @param motor_handle 电机句柄数组指针
 * @param dji_3508_speed_pid PID控制数组指针
 */
void omni_wheels_drive(const chassis_speed_t *ch_tgt, dji_motor_handle_t *motor_handle, pid_t *dji_3508_speed_pid);

/**
 * @brief 将4个车轮的转子实际转速正解算为底盘的宏观实际速度
 * @param motor_handle 电机句柄数组（用于读取 speed_rpm）
 * @param chassis_speed 输出的宏观速度
 */
void omni_wheels_forward(const dji_motor_handle_t *motor_handle, chassis_slope_t *chassis_speed);

#endif // OMNI_WHEELS_H
