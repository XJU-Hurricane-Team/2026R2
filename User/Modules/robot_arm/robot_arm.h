/**
 * @file robot_arm.h
 * @author xinglu
 * @brief 机械臂驱动模块
 * 
 * @version 2.0
 * @date 2026-04-24
 */

#ifndef ROBOT_ARM
#define ROBOT_ARM

#include <cubemx.h>
#include "./Damiao-Motor/damiao.h"
#include "arm_math.h"

/**
 * @brief 机械臂状态定义，为上位机预留的接口
 */
typedef enum {
	ARM_STATE_INIT = 0,        // 初始态
	ARM_STATE_READY = 1,       // 就绪态
	ARM_STATE_CATCH = 2,       // 抓取态
	ARM_STATE_PLACE = 3,       // 放置态
} arm_status_t;

/**
 * @brief 机械臂控制目标类型
 */
typedef enum {
	ARM_TARGET_CARTESIAN = 0,  // 末端位置目标(y/z/pitch)
	ARM_TARGET_JOINT = 1       // 关节角目标(joint1/2/3)
} arm_target_mode_t;

typedef struct {
	dm_handle_t damiao_1;          
	dm_handle_t damiao_2;             
	dm_handle_t damiao_3;             
	dm_handle_t damiao_4;              

	// 目标位置信息
	float arm_target_y;         // 机械臂目标Y坐标
	float arm_target_z;         // 机械臂目标Z坐标
	float arm_target_pitch;     // 机械臂目标俯仰角
	float arm_joint_target[3];  // 机械臂三个关节目标角

	// 机械臂轨迹规划
	uint8_t arm_motion_active;  // 机械臂运动是否激活标志
	arm_target_mode_t target_mode; // 当前目标模式

	// 状态管理
	arm_status_t status;        // 当前机械臂状态

	// 大臂过冲控制（用于关键点位避碰）
	float big_arm_overshoot_rad;    // 大臂过冲幅值上限(rad)
	float big_arm_final_joint;      // 过冲完成后回落的最终关节目标(rad)
	float big_arm_overshoot_joint;  // 过冲阶段的中间目标关节角(rad)
	uint8_t big_arm_overshoot_armed; // 过冲武装标志: 1=等待触发过冲, 0=不触发
	uint8_t big_arm_overshoot_phase; // 过冲阶段: 0=关闭, 1=去过冲点, 2=回落终点
} RobotArm;

// 初始化和更新函数
void robot_arm_init(void);
void robot_arm_system_init(RobotArm *arm);
void robot_arm_update(RobotArm *arm);
void robot_arm_task(void *pvParameters);
void robot_arm_apply_target(uint8_t index);

// 运动控制函数
void arm_pos_angle(float y1, float z1, float pitch_angle, float angle[3]);
void arm_apply_ctrl(RobotArm *arm, const float joint_des[3]);
void robot_arm_set_target(RobotArm *arm, float y, float z, float pitch);
void robot_arm_set_joint_target(RobotArm *arm, float joint1, float joint2,
								float joint3);
void robot_arm_set_big_arm_overshoot(RobotArm *arm, float overshoot_rad);
void robot_arm_set_ctrl_dt(float dt_s);

#endif /* ROBOT_ARM */

