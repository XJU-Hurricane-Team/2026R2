/**
 * @file catch_head.c
 * @author xinglu
 * @brief 矛头夹取模块
 * @version 1.1
 * @date 2026-04-11
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

/**
 * @brief DJI 电机夹取关节控制结构体
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
	float target_position_deg[CATCH_HEAD_DM_TARGET_COUNT];
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

void catch_head_on_key(key_press_t key); //调试接口：通过按键循环切换 DJI 电机、DM 电机和舵机目标

// 目标位设置函数
void catch_head_set_dji_target(uint8_t target_index);
void catch_head_set_dm_target(uint8_t target_index);
void catch_head_set_servo_target(uint8_t target_index);

#endif /* __CATCH_HEAD_H */
