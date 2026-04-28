/**
 * @file robot_arm.h
 * @author xinglu
 * @brief 机械臂底层驱动模块 (头文件)
 * @version 2.0
 * @date 2026-04-26
 */

#ifndef ROBOT_ARM_H
#define ROBOT_ARM_H

#include <cubemx.h>
#include "./Damiao-Motor/damiao.h"
#include "arm_math.h"

/**
 * @brief 机械臂工作状态枚举
 * 用于应用层与驱动层进行状态同步，决定当前所处的运动阶段。
 */
typedef enum {
	ARM_STATE_INIT = 0,        // 初始零点状态
	ARM_STATE_READY = 1,       // 准备抓取状态
	ARM_STATE_CATCH = 2,       // 抓取动作执行状态
	ARM_STATE_PLACE = 3,       // 放置动作执行状态
	ARM_STATE_TAKEOUT = 4,     // 取出动作执行状态
} arm_status_t;

/**
 * @brief 机械臂目标控制模式枚举
 */
typedef enum {
	ARM_TARGET_CARTESIAN = 0,  // 笛卡尔坐标系模式 (输入 Y, Z 坐标及 Pitch 姿态)
	ARM_TARGET_JOINT = 1       // 关节空间模式 (直接输入各关节角度)
} arm_target_mode_t;

/**
 * @brief 机械臂核心控制结构体
 * 包含电机句柄、目标点位、实时状态、状态机变量及运行上下文。
 */
typedef struct {
	/* 电机驱动句柄定义 */
	dm_handle_t damiao_1;       // 大臂关节电机1 (双电机同轴驱动)
	dm_handle_t damiao_2;       // 大臂关节电机2 (双电机同轴驱动，与电机1互为反向)
	dm_handle_t damiao_3;       // 小臂关节电机
	dm_handle_t damiao_4;       // 末端吸盘/夹爪电机

	/* 目标点位变量 (笛卡尔空间/关节空间) */
	float arm_target_y;         // 当前正在追踪的过渡/中间目标 Y 坐标
	float arm_target_z;         // 当前正在追踪的过渡/中间目标 Z 坐标
	float arm_target_pitch;     // 当前正在追踪的过渡/中间目标 Pitch 倾角
	float arm_joint_target[3];  // 关节模式下的目标角度数组
	
	/* 最终目标点位暂存 (用于过渡动作完成后恢复最终目标) */
	float final_target_y;
	float final_target_z;
	float final_target_pitch;
	
	/* 实时姿态反馈 (通过正向运动学 FK 解算得出) */
	float current_y;
	float current_z;
	float current_pitch;

	/* 多点过渡控制参数 (专门针对 CATCH -> PLACE 这种需要避障的长距离运动) */
	uint8_t multi_trans_active; // 多点过渡标志位 (1:正在进行多点过渡)
	uint8_t multi_trans_index;  // 当前执行到的过渡点位索引
	uint8_t multi_trans_count;  // 总共设定的过渡点数量
	float trans_y[4];           // 过渡点 Y 坐标数组
	float trans_z[4];           // 过渡点 Z 坐标数组
	float trans_pitch[4];       // 过渡点 Pitch 姿态数组

	/* 运行状态反馈与模式 */
	uint8_t arm_motion_active;     // 机械臂运动活跃标志 (1:正在移动, 0:已到达目标并静止)
	arm_target_mode_t target_mode; // 当前采用的控制模式 (坐标系解算/关节直驱)
	arm_status_t status;           // 当前应用层设定的目标状态

	/* 大臂过冲控制 (用于跨越奇点或特定姿态时的力矩补偿) */
	float big_arm_overshoot_rad;     // 设定的过冲角度大小 (弧度)
	float big_arm_final_joint;       // 过冲结束后的最终目标角度
	float big_arm_overshoot_joint;   // 叠加过冲量后的临时目标角度
	uint8_t big_arm_overshoot_armed; // 过冲动作使能标志 (1:准备执行过冲)
	uint8_t big_arm_overshoot_phase; // 过冲所处的阶段 (0:无, 1:正在过冲, 2:过冲完成正在回弹)

	/* 内部运行上下文与滤波缓冲 */
	float ctrl_dt;                  // 控制周期时间 (秒)，用于速度和积分计算
	float joint_cmd_prev[3];        // 上一控制周期的指令下发值 (用于计算导数或平滑)

	float big_arm_cmd_filtered;     // 大臂指令位置的低通滤波结果
	float big_arm_speed_filtered;   // 大臂指令速度的低通滤波结果
	float big_arm_ff;               // 大臂前馈量 (Feed-Forward)

	/* 定点锁死等待逻辑参数 (用于等待某一个关节到达指定位置再继续后续动作) */
	float suction_wait_locked_pos;   // 吸盘等待期间的锁定角度
	float small_arm_wait_locked_pos; // 小臂等待期间的锁定角度
	uint32_t wait_start_tick;        // 锁死等待动作的开始时间戳 (用于超时强制退出)

	arm_status_t last_status;        // 上一时刻的状态机状态 (用于检测状态切换边沿)
	int8_t last_target_quadrant;     // 上一次目标所处的象限 (1: 正向象限, -1: 反向象限)
	int8_t flip_transition_dir;      // 机械臂象限翻转的方向指示

	/* 位域压缩标志位 (节省内存，优化布尔变量存储) */
	struct {
		uint16_t joint_cmd_inited : 1;             // 初始关节指令是否已初始化
		uint16_t big_arm_filter_inited : 1;        // 大臂滤波器是否已初始化
		uint16_t suction_wait_latched : 1;         // 吸盘是否处于等待状态
		uint16_t takeout_wait_latched : 1;         // 是否处于取出等待状态
		uint16_t suction_wait_locked_inited : 1;   // 吸盘等待时的角度是否已锁存
		uint16_t small_arm_wait_locked_inited : 1; // 小臂等待时的角度是否已锁存
		uint16_t small_arm_wait_latched : 1;       // 小臂是否处于等待状态
		
		uint16_t has_transition : 1;               // 当前动作是否需要执行单点过渡
		uint16_t transition_done : 1;              // 单点过渡动作是否已执行完成
	} flags;

} RobotArm;

/* === 函数前置声明 === */
void robot_arm_system_init(RobotArm *arm);
void robot_arm_update(RobotArm *arm);
void robot_arm_set_target(RobotArm *arm, float y, float z, float pitch);
void robot_arm_set_joint_target(RobotArm *arm, float joint1, float joint2, float joint3);
void robot_arm_set_big_arm_overshoot(RobotArm *arm, float overshoot_rad);
void robot_arm_set_ctrl_dt(RobotArm *arm, float dt_s);
void robot_arm_fk(float joint1, float joint2, float joint3, float *y_out, float *z_out, float *pitch_out);

#endif /* ROBOT_ARM_H */