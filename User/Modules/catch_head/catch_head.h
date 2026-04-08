 /**
 * @file robot_arm.c
 * @author xinglu
 * @brief Ã¬Í·¼ÐÈ¡Ä£¿é
 * @version 1.0
 * @date 2026-04-08
 */

#ifndef __CATCH_HEAD_H
#define __CATCH_HEAD_H

#include <bsp.h>
#include <stdint.h>
#include "servo/servo.h"

#define CATCH_HEAD_DJI_TARGET_COUNT 2
#define CATCH_HEAD_DM_TARGET_COUNT  2
#define CATCH_HEAD_SERVO_TARGET_COUNT 2

#define CATCH_HEAD_DEG_TO_RAD 0.0174533f
#define CATCH_HEAD_RAD_TO_DEG 57.29578f
#define CATCH_HEAD_RAD_S_TO_RPM 9.549296f

#define DM_SPEED 0.6f

typedef struct {
	dji_motor_handle_t *motor_handle;
	float reduction_ratio;
	pid_t angle_pid;
	pid_t speed_pid;
	Trajectory trajectory;
	float target_angle_deg[CATCH_HEAD_DJI_TARGET_COUNT];
	uint8_t target_index;
} catch_head_dji_joint_t;

typedef struct {
	dm_handle_t *motor_handle;
	float target_position_deg[CATCH_HEAD_DM_TARGET_COUNT];
	uint8_t target_index;
} catch_head_dm_joint_t;

typedef struct {
	servo_t *servo_handle;
	uint8_t target_index;
} catch_head_servo_joint_t;

void catch_head_init(void);
void catch_head(void);

void catch_head_on_key(key_press_t key);

void catch_head_set_dji_target(uint8_t target_index);
void catch_head_set_dm_target(uint8_t target_index);
void catch_head_set_servo_target(uint8_t target_index);

#endif /* __CATCH_HEAD_H */
