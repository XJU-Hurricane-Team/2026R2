/**
 * @file robot_arm.h
 * @author meiwenhuaqingnian
 * @brief 
 * @version 1.2
 * @date 2025-11-30
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef ROBOT_ARM
#define ROBOT_ARM

#include <CSP_Config.h>
#include "./Damiao-Motor/damiao.h"
#include "./unitree_motor/unitree_motor.h"
#include "./trajectory_plan/trajectory_plan.h"


// 定义事件标志位
#define EVENT_TRAJ_FINISHED (1 << 0) // 轨迹完成事件标志
#define EVENT_CATCH     (1 << 1) // 启动抓取序列事件标志
#define EVENT_PLACE     (1 << 2) // 启动放置序列事件标志
#define EVENT_READY     (1 << 3) // 启动就绪序列事件标志

/**
 * @brief 
 * 
 * 
 */
typedef struct {
    unitree_motor_handle_t *motor1;
    unitree_motor_handle_t *motor2;
    dm_handle_t *motor3;

    float end_pos[3];
    int status;

} arm_handle_t;

/**
 * @brief 机械臂运行状态
 * 
 */
typedef enum {
    DEFAULT,        // 上电默认状态
    READY,          // 机械臂就绪状态
    CATCH,          // 执行 CATCH 动作
    PLACE,          // 执行 PLACE 动作
    MOVING_TO_READY // 回复 READY 状态

} arm_status_t;

void arm_pos_angle(float x1, float y1, float z1, float angle[3]);
void arm_angle_pos(float input_angle1, float input_angle2, float input_angle3,
                   float *x1, float *y1, float *z1);
uint8_t arm_angle_pos_2(float input_angle1, float input_angle2,
                        float input_angle3, float *x, float *y, float *z);
int arm_angle_drive(Trajectory_Handler_t trajectory[3], arm_handle_t *myarm,
                    float target_m1, float target_m2, float target_dm);

#endif /* ROBOT_ARM */
