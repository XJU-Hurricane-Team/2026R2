/**
 * @file robot_arm.h
 * @author xinglu
 * @brief 机械臂驱动模块
 * @version 1.3
 * @date 2026-04-04
 */

#ifndef ROBOT_ARM
#define ROBOT_ARM

#include <cubemx.h>
#include "./Damiao-Motor/damiao.h"
#include "./trajectory_plan/trajectory_plan.h"

/**
 * @brief 机械臂状态定义
 */
typedef enum {
	ARM_DEFAULT = 0,           // 默认状态
	ARM_MOVING_TO_READY = 1,   // 移动到准备位置
	ARM_READY = 2,             // 准备就绪
	ARM_CATCHING = 3,          // 抓取动作
	ARM_PLACING = 4,           // 放置动作
} arm_status_t;

/**
 * @brief 机械臂事件标志定义
 */
typedef enum {
	EVENT_READY = 0,           // 准备位置事件
	EVENT_CATCH = 1,           // 抓取事件
	EVENT_PLACE = 2,           // 放置事件
	EVENT_TRAJ_FINISHED = 3,   // 轨迹完成事件
} arm_event_t;

typedef struct {
	dm_handle_t damiao_1;          
	dm_handle_t damiao_2;             
	dm_handle_t damiao_3;             
	dm_handle_t damiao_4;              

	// 目标位置信息
	float arm_target_y;                // 机械臂目标Y坐标
	float arm_target_z;                // 机械臂目标Z坐标
	float arm_target_pitch;            // 机械臂目标俯仰角

	float dm_debug_target_position[4]; // 四个电机独立调试目标位置

	// 机械臂轨迹规划
	float arm_start_pos[3]; // 机械臂三轴起始位置
	float arm_goal_pos[3];  // 机械臂三轴目标位置
	Trajectory arm_traj[3]; // 机械臂三轴梯形轨迹
	uint8_t arm_motion_active;         // 机械臂运动是否激活标志

	// 电机轨迹规划
	float dm_start_pos[4];   // 四个电机起始位置
	float dm_goal_pos[4];    // 四个电机目标位置
	Trajectory dm_traj[4];   // 四个电机梯形轨迹
	uint8_t dm_motion_active;          // 电机运动是否激活标志

	// 状态管理
	arm_status_t status;               // 当前机械臂状态
} RobotArm;

// 初始化和更新函数
void robot_arm_system_init(RobotArm *arm);
void robot_arm_test_update(RobotArm *arm);
void robot_arm_dm_debug_update(RobotArm *arm);
void robot_arm_dm_set_target(RobotArm *arm, uint8_t motor_index, float position);

// 运动控制函数
void arm_pos_angle(float y1, float z1, float pitch_angle, float angle[3]);
void robot_arm_update(RobotArm *arm);
void robot_arm_set_target(RobotArm *arm, float y, float z, float pitch);

// 事件触发函数
void robot_arm_event_ready(RobotArm *arm);
void robot_arm_event_catch(RobotArm *arm, float y, float z);

#endif /* ROBOT_ARM */

