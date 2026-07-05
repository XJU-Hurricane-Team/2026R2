/**
 * @file    lift.h
 * @author  Dominate0017
 * @brief   抬升与2006控制模块
 * @version 1.0
 * @date    2026-04-28
 */

#ifndef LIFT_H
#define LIFT_H

#include <stdbool.h>
#include <stdint.h>
#include "remote_ctrl/remote_ctrl.h"

#define LIFT_TARGET_UP_R1_DEG    5.42f

void lift_init(void);
void lift_switch_mode(uint8_t key, remote_key_event_t event);
void lift_begin_catch_up_until_proximity(void);
void lift_on_catch_proximity_falling_edge(void);
void lift_set_chassis_mode(bool is_auto_mode);
void lift_set_stair_mode(uint8_t mode);
void lift_set_target(float target_degree);
bool lift_is_sequence_running(void);
void chassis_proximity_switch(void);

#endif /* LIFT_H */
