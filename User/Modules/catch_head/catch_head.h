/**
 * @file catch_head.h
 * @author xinglu
 * @brief 矛头夹取模块
 * @version 1.2
 * @date 2026-04-13
 */

#ifndef __CATCH_HEAD_H
#define __CATCH_HEAD_H

#include <bsp.h>
#include <stdint.h>
#include "servo/servo.h"

#define CATCH_HEAD_DJI_TARGET_COUNT 2
#define CATCH_HEAD_DM_TARGET_COUNT  3
#define CATCH_HEAD_SERVO_TARGET_COUNT 2

#define CATCH_HEAD_DEG_TO_RAD 0.0174533f
#define CATCH_HEAD_RAD_TO_DEG 57.29578f
#define CATCH_HEAD_RAD_S_TO_RPM 9.549296f

#define DM_SPEED 0.6f

typedef enum {
	CATCH_HEAD_DJI_TARGET_HOME = 0,
	CATCH_HEAD_DJI_TARGET_ASSEMBLY,
} catch_head_dji_target_t;

typedef enum {
	CATCH_HEAD_DM_TARGET_RETRACT = 0,
	CATCH_HEAD_DM_TARGET_EXTEND,
	CATCH_HEAD_DM_TARGET_CHECK,
} catch_head_dm_target_t;

typedef enum {
	CATCH_HEAD_SERVO_TARGET_CLOSE = 0,
	CATCH_HEAD_SERVO_TARGET_OPEN,
} catch_head_servo_target_t;

/**
 * @brief DJI 电机旋转关节控制结构体
 */
typedef struct {
	dji_motor_handle_t *motor_handle;
	float reduction_ratio;
	pid_t angle_pid;
	pid_t speed_pid;
	Trajectory_Handler_t trajectory;
	float target_angle_deg[CATCH_HEAD_DJI_TARGET_COUNT];
	uint8_t target_index;
} catch_head_dji_joint_t;

/**
 * @brief DM 电机执行关节控制结构体
 * @note 主要用于夹取机构的位姿切换控制
 */
typedef struct {
	dm_handle_t *motor_handle;
	float target_position_rad[CATCH_HEAD_DM_TARGET_COUNT];
	uint8_t target_index;
} catch_head_dm_joint_t;

/**
 * @brief 舵机执行关节控制结构体
 * @note 用于控制夹爪开合状态
 */
typedef struct {
	servo_t *servo_handle;
	uint8_t target_index;
} catch_head_servo_joint_t;

void catch_head_init(void);   //初始化夹取模块
void catch_head(void);        // 夹取模块周期更新函数

// 目标位设置函数
void catch_head_set_dji_target(uint8_t target_index);
void catch_head_set_dm_target(uint8_t target_index);
void catch_head_set_servo_target(uint8_t target_index);

#endif /* __CATCH_HEAD_H */
