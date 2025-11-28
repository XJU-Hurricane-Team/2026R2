/**
 * @file s_curve_programing.h
 * @author meiwenhuaqingnian
 * @brief 
 * @version 0.1
 * @date 2025-11-09
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef S_CURVE_PROGRAMING
#define S_CURVE_PROGRAMING

#include <CSP_Config.h>

#include <stdio.h>
#include <math.h>


/**
 * @brief 运动状态标志
 * 
 */
typedef enum {
    ACCELERATING, // 加速阶段
    UNIFORM_VELOCITY, // 匀速阶段
    DECELERATING, // 减速阶段
    FINISHED // 运动结束
} TrajectoryState;


/**
 * @brief 轨迹参数结构体
 * 
 */
typedef struct {
    double p_start;        // 起始位置 (rad)
    double p_goal;         // 目标位置 (rad)
    double v_max;          // 最大速度 (rad/s)
    double a_max;          // 最大加速度 (rad/s^2)
    double dt;             // 控制周期时间 (s)
    double total_time;     // 总运动时间 (s)
    double ta;             // 加速/减速时间 (s)
    double tv;             // 匀速时间 (s)
    double p_accel;        // 加速段距离 (rad)
    TrajectoryState state; // 当前运动状态
    double current_time;   // 运动已进行时间 (s)
    int is_negative;       // 运动方向标记 (1或-1)
} Trajectory_Handler_t;

void t_trajectory_init(Trajectory_Handler_t *traj, float p_start, float p_goal, float v_max, float a_max, float dt);
int t_trajectory_update(Trajectory_Handler_t *traj, float *p_des, float *w_des);
float s_trajectory_update(float start_angle, float target_angle, float current_time, float total_time);

#endif /* S_CURVE_PROGRAMING */

