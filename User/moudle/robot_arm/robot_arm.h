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

void arm_ctrl(float x1, float y1, float z1, float angle[3]);
uint8_t arm_forward_kinematics(dm_handle_t *pan_motor, 
                               unitree_motor_handle_t *arm1_motor,
                               unitree_motor_handle_t *arm2_motor,
                               float *x1, float *y1, float *z1);
uint8_t arm_get_end_position(dm_handle_t *pan_motor,
                             unitree_motor_handle_t *arm1_motor,
                             unitree_motor_handle_t *arm2_motor,
                             float end_pos[3]);
uint8_t arm_inverse_solution(float angle[3], float *x, float *y, float *z);

#endif /* ROBOT_ARM */

