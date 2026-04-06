/**
 * @file    omni_wheels.c
 * @author  Dominate0017
 * @brief   全向轮运动学模块
 * @version 0.1
 * @date    2026-04-2
 */

#include "omni_wheels.h"
#include <math.h>

/* ======================================================= 全向轮底盘相关函数 ======================================================= */

/**
 * @brief 世界坐标系速度向车身坐标系速度转换
 * @param speed 指向底盘线速度结构体的指针
 * @param yaw_angle 当前小车的偏航角(Yaw)，单位rad
 */
void omni_wheels_world_transform(chassis_slope_t *speed, float yaw_angle)
{
    float cos_yaw = cosf(yaw_angle);
    float sin_yaw = sinf(yaw_angle);
    
    // 计算公式待定
    float vx_world = speed->vx * cos_yaw - speed->vy * sin_yaw;
    float vy_world = speed->vx * sin_yaw + speed->vy * cos_yaw;
    
    speed->vx = vx_world;
    speed->vy = vy_world;
}

/**
 * @brief 四轮全向轮底盘运动学逆解算
 *        4个轮子分别安装在四角且呈45度分布
 *        正前向为X正向，正左为Y正向，逆时针为Yaw正向
 *        v1 (右前), v2 (左前), v3 (左后), v4 (右后)
 */
void omni_wheels_resolve(const chassis_speed_t *chassis_speed, volatile float *out_wheel_rpm)
{
    float vx = chassis_speed->target_speed.vx;
    float vy = chassis_speed->target_speed.vy;
    float vw = chassis_speed->target_speed.vw;

    // 设底盘中心为坐标原点, 车头正向为 X 轴正方向, 车身正左为 Y 轴正方向。
    // 设4个全向轮分别安装在四个角, 每个轮子的法线与坐标轴成 45 度角 (即 𝛑/4)。
    // COS(45°) = SIN(45°) ≈ 0.707 (通过 sqrt(2)/2 计算)。
    // 在全向轮等效速度模型中, 要将底盘的线速度(Vx, Vy)投影到每个轮子的法线上。
    // 此外, 底盘旋转(Vw)使得每个轮子产生一个切向线速度, 大小为 Vw * 旋转半径(R)。输出轴逆时针转动

    float v1 = -SQRT2_2 * vx - SQRT2_2 * vy + vw * CHASSIS_RADIUS; // 轮1：右上 (前偏右)
    float v2 =  SQRT2_2 * vx - SQRT2_2 * vy + vw * CHASSIS_RADIUS; // 轮2：左上 (前偏左)
    float v3 =  SQRT2_2 * vx + SQRT2_2 * vy + vw * CHASSIS_RADIUS; // 轮3：左下 (后偏左)
    float v4 = -SQRT2_2 * vx + SQRT2_2 * vy + vw * CHASSIS_RADIUS; // 轮4：右下 (后偏右)

    /* 转化为RPM */ 
    out_wheel_rpm[0] = MPS_TO_RPM(v1);
    out_wheel_rpm[1] = MPS_TO_RPM(v2);
    out_wheel_rpm[2] = MPS_TO_RPM(v3);
    out_wheel_rpm[3] = MPS_TO_RPM(v4);
}

/**
 * @brief 综合控制接口：逆解算 + PID + 输出控制电流
 * @param ch_tgt 目标宏观速度
 * @param motor_handle 电机句柄数组指针
 * @param s_planner S曲线规划器数组指针
 * @param dji_3508_speed_pid PID控制数组指针
 */
void omni_wheels_drive(const chassis_speed_t *ch_tgt, dji_motor_handle_t *motor_handle, pid_t *dji_3508_speed_pid)
{
   
    float wheel_rpm[4] = {0}; 
    int16_t motor_out_current[4] = {0}; 
    /* 1. 运动学逆解算：从宏观速度解算到各个轮子的目标RPM */ 
    omni_wheels_resolve(ch_tgt, wheel_rpm);

    /* 2. 将各个轮子独立进行PID计算 */
    for(int i = 0; i < 4; i++) {
        // 通过PID将平滑后的目标转速计算为期望的电机控制电流
        float real_rpm = motor_handle[i].speed_rpm;
        float calc_current = pid_calc(&dji_3508_speed_pid[i], wheel_rpm[i], real_rpm);
        motor_out_current[i] = (int16_t)calc_current;
    }

    /* 3. 统一输出电流到四个底盘电机 */
    dji_motor_set_current(can1_selected, 0x200, motor_out_current[0], 
                          motor_out_current[1], motor_out_current[2], motor_out_current[3]);
}

/**
 * @brief 底盘运动学正解算：根据当前各自电机的真实RPM转速，反推底盘在宏观坐标系下的真实运行速度
 *        (可用于里程计、闭环校准等)
 * @param motor_handle 电机句柄数组（读取其 speed_rpm）
 * @param chassis_speed 解算后输出的宏观速度
 */
void omni_wheels_forward(const dji_motor_handle_t *motor_handle, chassis_speed_t *chassis_speed)
{
    /* 1. 将RPM转化为轮子的切向线速度 (m/s) */
    float v1 = RPM_TO_MPS((float)motor_handle[0].speed_rpm);
    float v2 = RPM_TO_MPS((float)motor_handle[1].speed_rpm);
    float v3 = RPM_TO_MPS((float)motor_handle[2].speed_rpm);
    float v4 = RPM_TO_MPS((float)motor_handle[3].speed_rpm);

    /* 2. 运动学正解算 (逆向求解) */
    // 根据方程推导的最小二乘解或由于机械对称性得出的逆矩阵结果:
    chassis_speed->target_speed.vx = ( v1 + v2 - v3 - v4) / (4.0f * SQRT2_2);
    chassis_speed->target_speed.vy = (-v1 + v2 + v3 - v4) / (4.0f * SQRT2_2);
    chassis_speed->target_speed.vw = ( v1 - v2 - v3 + v4) / (4.0f * CHASSIS_RADIUS);
}

