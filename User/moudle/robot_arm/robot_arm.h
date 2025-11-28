/**
 * @file robot_arm.h
 * @author meiwenhuaqingnian
 * @brief 
 * @version 0.1
 * @date 2025-11-05
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

/**
 * @brief 
 * 
 * 
 */
typedef struct {
    void *motor1;
    void *motor2;
    void *motor3;

    bool status;


} arm_handle_t;

/**
 * @brief 机械臂运行状态
 * 
 */
typedef enum {
    DEFAULT,
    READY,
    CATCH,
    PLACE

} arm_status_t;

void arm_ctrl(float x1, float y1, float z1, float angle[3]);
uint8_t arm_inverse_solution(float angle[3], float *x, float *y, float *z);
int arm_action_set(Trajectory_Handler_t trajectory[3], float current_m1,float current_m2,
                          float current_m3,float target_m1,float target_m2, float target_dm);

#endif /* ROBOT_ARM */

