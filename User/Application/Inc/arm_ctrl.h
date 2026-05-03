/**
 * @file arm_ctrl.h
 * @author xinglu
 * @brief 
 * @version 0.1
 * @date 2026-04-30
 * 
 * @copyright Copyright (c) 2026
 * 
 */

 #ifndef __ARM_CTRL_H
 #define __ARM_CTRL_H

#include "robot_arm/robot_arm.h"

void arm_microros_set_state(arm_status_t state);
void arm_microros_clear_target(void);
void robot_arm_set_dynamic_catch_target(float y, float z, float pitch);

#endif /* __ARM_CTRL_H */