/**
 * @file robot_arm.c
 * @author xinglu
 * @brief 机械臂驱动模块 (核心算法与动力学层)
 * @version 2.0 
 * @date 2026-04-30
 */

#include "robot_arm.h"
#include <math.h>
#include <string.h> 

/**
 * @brief 浮点数限幅函数
 */
static float arm_clampf(float v, float lo, float hi) {
	if (v < lo) return lo;
	if (v > hi) return hi;
	return v;
}

/**
 * @brief 角度归一化函数，将角度限制在 [-PI, PI] 之间
 */
static float arm_wrap_pi(float angle) {
	while (angle > PI) angle -= 2.0f * PI;
	while (angle < -PI) angle += 2.0f * PI;
	return angle;
}

/**
 * @brief 判断小臂是否接近目标以触发三关节同步修正
 */
static uint8_t arm_small_arm_sync_ready(float target, float current) {
	return (fabsf(current - target) <= ARM_TAKEOUT_SMALL_ARM_SYNC_ERR_RAD);
}

/**
 * @brief 获取逆解算坐标所处的象限 (判断目标在正前还是正后)
 * @return 1 为正向(前)，-1 为反向(后)
 */
static int8_t arm_get_ik_quadrant(float y, float z, float pitch) {
	// 根据连杆公式推断腕关节(手腕)的世界坐标 X 轴
	float x_w = y + DEFAULT_X - ARM_3 * cosf(pitch) - (DEFAULT_ARM3_X - DEFAULT_ARM_1_2) * cosf(pitch);
	(void)z; // 忽略 Z 轴影响
	return (x_w >= 0.0f) ? 1 : -1;
}

/**
 * @brief 获取逆运动学解算角度，并进行物理约束与最短路径映射
 * 本函数解决多圈旋转和物理限位问题，防止电机为了到达目标而进行无意义的多绕一圈，
 * 同时防止末端吸盘打到本体发生物理干涉。
 */
static void arm_get_unwrapped_target(RobotArm *arm, float y, float z, float pitch, float out_joints[3]) {
	// 1. 调用基础数学逆解算法获得理论角度
	arm_pos_angle(y, z, pitch, out_joints);
	
	// 2. 对大臂关节进行最短路径映射
	// 算出当前位置与目标位置的差值，如果绝对值大于PI，则加减 2PI，走较短的圆弧。
	float diff0 = out_joints[0] - arm->damiao_1.position;
	while (diff0 > PI) { diff0 -= 2.0f * PI; out_joints[0] -= 2.0f * PI; }
	while (diff0 < -PI) { diff0 += 2.0f * PI; out_joints[0] += 2.0f * PI; }

	// 大臂硬限位：任何模式下都不允许小于 0
	if (out_joints[0] < ARM_BIG_ARM_MIN_ANGLE_RAD) out_joints[0] = ARM_BIG_ARM_MIN_ANGLE_RAD;

	// 3. 末端吸盘 (damiao_4) 的物理防撞限位保护 (允许区间: -0.1 到 2.8 弧度)
	float target2 = out_joints[2];
	float diff2 = target2 - arm->damiao_4.position;
	
	// 尝试做最短路径映射，但如果映射后的角度超出了物理限位 (-0.1 ~ 2.8)，则放弃该映射方向
	while (diff2 > PI) { 
		if (target2 - 2.0f * PI < -0.1f) {
			break;  // 超出下限，放弃映射
		}
		diff2 -= 2.0f * PI; 
		target2 -= 2.0f * PI; 
	}
	while (diff2 < -PI) { 
		if (target2 + 2.0f * PI > 2.8f) {
			break;  // 超出上限，放弃映射
		}
		diff2 += 2.0f * PI; 
		target2 += 2.0f * PI; 
	}
	
	// 4. 进行最终的硬限幅
	if (target2 < -0.1f) target2 = -0.1f;
	if (target2 > 2.8f) target2 = 2.8f;
	
	out_joints[2] = target2;
}

/**
 * @brief 更新动作过渡的逻辑门限 (状态机锁存器)
 * 当检测到机械臂从一个工作状态(如CATCH)转入另一个状态(如PLACE)时，
 * 初始化防碰撞的等待逻辑(例如先举起大臂，再转动小臂)，记录开始时间以供超时检测。
 */
static void arm_update_transition_gate(RobotArm *arm) {
	arm_status_t prev = arm->last_status;
	if (arm->status != arm->last_status) {
		if ((prev == ARM_STATE_CATCH) && (arm->status == ARM_STATE_PLACE)) {
			// 从抓取转放置：锁存吸盘状态，防止带着货物转吸盘导致碰撞
			arm->flags.suction_wait_latched = 1;
			arm->flags.takeout_wait_latched = 0;
			arm->flags.small_arm_wait_latched = 0; 
			arm->flags.suction_wait_locked_inited = 0;
			arm->wait_start_tick = HAL_GetTick(); 
		} else if ((prev == ARM_STATE_PLACE) && (arm->status == ARM_STATE_TAKEOUT)) {
			// 从放置转取出：先锁住小臂/吸盘，让大臂先过冲
			arm->flags.takeout_wait_latched = 1;
			arm->flags.suction_wait_latched = 0;
			arm->flags.small_arm_wait_latched = 0;
			arm->flags.small_arm_wait_locked_inited = 0;
			arm->flags.suction_wait_locked_inited = 0;
			arm->wait_start_tick = HAL_GetTick();

			arm->takeout_sync_active = 1;
			arm->takeout_sync_phase = 1;
			arm->takeout_small_target_inited = 0;
			arm->big_arm_overshoot_rad = ARM_PLACE_TAKEOUT_BIG_ARM_OVERSHOOT_RAD;
			arm->big_arm_overshoot_armed = 1;
			arm->big_arm_overshoot_phase = 0;
		} else if ((prev == ARM_STATE_PLACE) && (arm->status == ARM_STATE_CATCH)) {
			// 从放置转抓取：小臂锁定等待，确保空手状态下安全复位
			arm->flags.small_arm_wait_latched = 1;
			arm->flags.suction_wait_latched = 0;
			arm->flags.takeout_wait_latched = 0;
			arm->flags.small_arm_wait_locked_inited = 0;
			arm->wait_start_tick = HAL_GetTick(); 
		}
		arm->last_status = arm->status;
	}

	if (arm->status != ARM_STATE_TAKEOUT) {
		arm->takeout_sync_active = 0;
		arm->takeout_sync_phase = 0;
		arm->takeout_small_target_inited = 0;
	}
}

/**
 * @brief 初始化整个驱动层上下文
 */
void robot_arm_system_init(RobotArm *arm) {
	memset(arm, 0, sizeof(RobotArm)); 

	// 初始化达妙电机驱动 (ID, NodeID, PVT控制模式, 电机型号, 最大扭矩等)
	dm_motor_init(&arm->damiao_1, 0x11, 0x01, DM_MODE_PVT, DM_J8006, 3.14f, 10.0f, 10.0f, can3_selected);
	dm_motor_init(&arm->damiao_2, 0x12, 0x02, DM_MODE_PVT, DM_J8006, 3.14f, 10.0f, 10.0f, can3_selected);
	dm_motor_init(&arm->damiao_3, 0x13, 0x03, DM_MODE_PVT, DM_J4340, 3.14f, 10.0f, 10.0f, can3_selected);
	dm_motor_init(&arm->damiao_4, 0x14, 0x04, DM_MODE_PVT, DM_J4310, 3.14f, 10.0f, 10.0f, can3_selected);

	// 使能所有电机
	dm_motor_enable(&arm->damiao_1);
	dm_motor_enable(&arm->damiao_2);
	dm_motor_enable(&arm->damiao_3);
	dm_motor_enable(&arm->damiao_4);

	// 设置初始默认值
	arm->target_mode = ARM_TARGET_CARTESIAN;
	arm->status = ARM_STATE_INIT;
	arm->last_status = ARM_STATE_INIT;
	arm->ctrl_dt = ARM_CTRL_DT_DEFAULT;

	arm->joint_cmd_prev[0] = DEFAULT_ANGLE_1;
	arm->joint_cmd_prev[1] = DEFAULT_ANGLE_2;
	arm->joint_cmd_prev[2] = DEFAULT_ANGLE_3;
	arm->big_arm_cmd_filtered = DEFAULT_ANGLE_1;
}

/**
 * @brief 设置控制循环的时间步长
 */
void robot_arm_set_ctrl_dt(RobotArm *arm, float dt_s) {
	arm->ctrl_dt = arm_clampf(dt_s, 0.001f, 0.2f);
}

/**
 * @brief 核心函数：下发笛卡尔空间目标坐标，并生成避障路径规划
 * @param y 前后坐标
 * @param z 上下高度坐标
 * @param pitch 末端姿态
 */
void robot_arm_set_target(RobotArm *arm, float y, float z, float pitch) {
	arm->target_mode = ARM_TARGET_CARTESIAN;
	arm->flags.place_exit_safety_active = 0;

	int8_t target_quadrant = arm_get_ik_quadrant(y, z, pitch);
	arm->flip_transition_dir = 0;
	arm->big_arm_overshoot_armed = 0;
	arm->big_arm_overshoot_phase = 0;

	// 检测目标是否需要跨越基座中心(发生象限翻转)
	if (arm->last_target_quadrant != 0) {
		if ((arm->last_target_quadrant < 0) && (target_quadrant > 0)) {
			arm->flip_transition_dir = 1;   // 从后翻向前
		} else if ((arm->last_target_quadrant > 0) && (target_quadrant < 0)) {
			arm->flip_transition_dir = -1;  // 从前翻向后
		}
	}

	uint8_t delay_overshoot = 0; 

	// === 多点路径规划逻辑 ===
	// 从放置态退出到任意非放置态时，先走一个放置点(y/z)+100mm的安全过渡点
	// 但 PLACE->TAKEOUT 不走该过渡点
	if (arm->last_status == ARM_STATE_PLACE && arm->status != ARM_STATE_PLACE &&
	    arm->status != ARM_STATE_TAKEOUT && arm->status != ARM_STATE_INIT) {
		arm->multi_trans_active = 1;
		arm->multi_trans_index = 0;
		arm->multi_trans_count = 1;

		arm->trans_y[0] = arm->final_target_y + 100.0f;
		arm->trans_z[0] = arm->final_target_z + 100.0f;
		arm->trans_pitch[0] = pitch;

		arm->final_target_y = y;
		arm->final_target_z = z;
		arm->final_target_pitch = pitch;

		arm->arm_target_y = arm->trans_y[0];
		arm->arm_target_z = arm->trans_z[0];
		arm->arm_target_pitch = arm->trans_pitch[0];

		arm->flags.has_transition = 0;
		arm->flags.place_exit_safety_active = 1;
	}

	// 如果发生从 CATCH 到 PLACE 的长距离运动，自动插入 3 个高空过渡点，防止刮蹭底盘
	else if (arm->last_status == ARM_STATE_CATCH && arm->status == ARM_STATE_PLACE) {
		arm->multi_trans_active = 1;
		arm->multi_trans_index = 0;
		arm->multi_trans_count = 3; 
		
		// 预设三个安全提升和转移路点
		arm->trans_y[0] = 468.158f; arm->trans_z[0] = 183.510f; arm->trans_pitch[0] = 0.0f;
		arm->trans_y[1] = 422.826f; arm->trans_z[1] = 775.271f; arm->trans_pitch[1] = PI/2.0f;
		arm->trans_y[2] = 229.258f; arm->trans_z[2] = 785.030f; arm->trans_pitch[2] = PI/2.0f;
		
		arm->final_target_y = y;
		arm->final_target_z = z;
		arm->final_target_pitch = pitch;
		
		// 优先执行第1个过渡点
		arm->arm_target_y = arm->trans_y[0];
		arm->arm_target_z = arm->trans_z[0];
		arm->arm_target_pitch = arm->trans_pitch[0];
		
		arm->flags.has_transition = 0; 
		delay_overshoot = 1; // 延迟使能过冲，等到达高空再进行翻转
		
	}

	else if (arm->status == ARM_STATE_CATCH || arm->status == ARM_STATE_PLACE) {
		// 单点防撞避让逻辑：生成一个偏离目标点高度(Z)和距离(Y)的临时点
		arm->multi_trans_active = 0;
		arm->flags.has_transition = 1;
		arm->flags.transition_done = 0;
		arm->final_target_y = y;
		arm->final_target_z = z;
		arm->final_target_pitch = pitch;

		float offset_y = ARM_TRANS_Y_OFFSET;
		if (y >= 0.0f) {
			arm->arm_target_y = (y > offset_y) ? (y - offset_y) : 0.0f;
		} else {
			arm->arm_target_y = (y < -offset_y) ? (y + offset_y) : 0.0f;
		}
		arm->arm_target_z = z + ARM_TRANS_Z_OFFSET;
		arm->arm_target_pitch = pitch;
	} else {
		// 无需过渡，直接直达目标
		arm->multi_trans_active = 0;
		arm->flags.has_transition = 0;
		arm->flags.transition_done = 1; 
		arm->final_target_y = y;
		arm->final_target_z = z;
		arm->final_target_pitch = pitch;
		arm->arm_target_y = y;
		arm->arm_target_z = z;
		arm->arm_target_pitch = pitch;
	}

	// === 大臂过冲策略 (翻越死区) ===
	if (!delay_overshoot) {
		if (arm->status == ARM_STATE_PLACE || arm->status == ARM_STATE_TAKEOUT) {
			arm->big_arm_overshoot_rad = ARM_BIG_ARM_FLIP_OVERSHOOT_FORCE_RAD;
			arm->big_arm_overshoot_armed = 1;
		} else if (arm->flip_transition_dir != 0) {
			if (z < 800.0f) {
				arm->big_arm_overshoot_rad = ARM_BIG_ARM_FLIP_OVERSHOOT_FORCE_RAD;
				arm->big_arm_overshoot_armed = 1;
			} else {
				arm->big_arm_overshoot_armed = 0; // 高度足够安全，无需过冲
			}
		}
	}
	arm->last_target_quadrant = target_quadrant;
}

/**
 * @brief 直接下发关节角度 (用于调试或特殊动作)
 */
void robot_arm_set_joint_target(RobotArm *arm, float joint1, float joint2, float joint3) {
	arm->arm_joint_target[0] = (joint1 < ARM_BIG_ARM_MIN_ANGLE_RAD) ? ARM_BIG_ARM_MIN_ANGLE_RAD : joint1;
	arm->arm_joint_target[1] = joint2;
	arm->arm_joint_target[2] = joint3;
	arm->target_mode = ARM_TARGET_JOINT;
	arm->big_arm_overshoot_armed = 0;
	arm->big_arm_overshoot_phase = 0;
}

/**
 * @brief 手动设置过冲量
 */
void robot_arm_set_big_arm_overshoot(RobotArm *arm, float overshoot_rad) {
	if (overshoot_rad <= 0.0f) {
		arm->big_arm_overshoot_rad = 0.0f;
		arm->big_arm_overshoot_armed = 0;
		arm->big_arm_overshoot_phase = 0;
		return;
	}
	arm->big_arm_overshoot_rad = overshoot_rad;
	arm->big_arm_overshoot_armed = 1;
}

/**
 * @brief 驱动层主循环任务
 * 包含了过渡点的到位检测更新、防死锁钳位、以及底层电机速度和力矩的更新分发。
 */
void robot_arm_update(RobotArm *arm) {
	float joint_target[3];
	float joint_cmd[3];

	// 初始化初始指令值，防止突变
	if (!arm->flags.joint_cmd_inited) {
		arm->flags.joint_cmd_inited = 1;
		arm->joint_cmd_prev[0] = arm->damiao_1.position;
		arm->joint_cmd_prev[1] = arm->damiao_3.position;
		arm->joint_cmd_prev[2] = arm->damiao_4.position;
	}

	arm_update_transition_gate(arm);

	if (arm->target_mode == ARM_TARGET_JOINT) {
		// 关节直驱模式
		joint_target[0] = arm->arm_joint_target[0];
		joint_target[1] = arm->arm_joint_target[1];
		joint_target[2] = arm->arm_joint_target[2];
		
		// 强制防碰撞保护
		if (joint_target[2] < -0.1f) joint_target[2] = -0.1f;
		if (joint_target[2] > 2.8f) joint_target[2] = 2.8f;
		
	} else {
		// 笛卡尔模式：首先求取当前目标点对应的 IK 逆解关节角
		arm_get_unwrapped_target(arm, arm->arm_target_y, arm->arm_target_z, arm->arm_target_pitch, joint_target);
		
		// === 1. 处理多点过渡逻辑 (Spline waypoints) ===
		if (arm->multi_trans_active) {
			float actual_suction_target = joint_target[2];
			if (arm->status == ARM_STATE_PLACE) {
				actual_suction_target -= ARM_PLACE_SUCTION_OFFSET_RAD; // 附加一个下压预紧力
			}
			// 限幅
			if (actual_suction_target < -0.1f) actual_suction_target = -0.1f;
			if (actual_suction_target > 2.8f) actual_suction_target = 2.8f;

			// 计算三个关节与当前目标的误差
			float err0 = fabsf(arm->damiao_1.position - joint_target[0]);
			float err1 = fabsf(arm->damiao_3.position - joint_target[1]);
			float err2 = fabsf(arm->damiao_4.position - actual_suction_target);

			if (arm->status == ARM_STATE_TAKEOUT && arm->takeout_sync_active && arm->takeout_sync_phase >= 2) {
				err0 = 0.0f; // 大臂过冲保持阶段允许途经点继续推进
			}
			
			// 如果全部到达允许的容差内 (< 0.15弧度)，切换至下一个途经点
			if (err0 < 0.15f && err1 < 0.15f && err2 < 0.15f) {
				arm->multi_trans_index++;
				if (arm->multi_trans_index < arm->multi_trans_count) {
					arm->arm_target_y = arm->trans_y[arm->multi_trans_index];
					arm->arm_target_z = arm->trans_z[arm->multi_trans_index];
					arm->arm_target_pitch = arm->trans_pitch[arm->multi_trans_index];
					arm_get_unwrapped_target(arm, arm->arm_target_y, arm->arm_target_z, arm->arm_target_pitch, joint_target);
				} else {
					// 途径点全部走完，恢复最终目标
					arm->multi_trans_active = 0;
					arm->flags.place_exit_safety_active = 0;
					arm->arm_target_y = arm->final_target_y;
					arm->arm_target_z = arm->final_target_z;
					arm->arm_target_pitch = arm->final_target_pitch;
					arm_get_unwrapped_target(arm, arm->arm_target_y, arm->arm_target_z, arm->arm_target_pitch, joint_target);
					
					// 使能过冲动作确保到达 (PLACE->TAKEOUT 特殊序列期间不重复触发)
					if (!arm->takeout_sync_active) {
						arm->big_arm_overshoot_rad = ARM_BIG_ARM_FLIP_OVERSHOOT_FORCE_RAD;
						arm->big_arm_overshoot_armed = 1;
					}
				}
			}
		} 
		// === 2. 处理单点防撞过渡逻辑 ===
		else if (arm->flags.has_transition && !arm->flags.transition_done) {
			float err0 = fabsf(arm->damiao_1.position - joint_target[0]);
			float err1 = fabsf(arm->damiao_3.position - joint_target[1]);
			float err2 = fabsf(arm->damiao_4.position - joint_target[2]);
			
			// 当到达临时安全点后，将目标切回真实的最终点位
			if (arm->big_arm_overshoot_phase == 0 && err0 < 0.12f && err1 < 0.12f && err2 < 0.12f) {
				arm->flags.transition_done = 1; 
				arm->arm_target_y = arm->final_target_y;
				arm->arm_target_z = arm->final_target_z;
				arm->arm_target_pitch = arm->final_target_pitch;
				arm_get_unwrapped_target(arm, arm->arm_target_y, arm->arm_target_z, arm->arm_target_pitch, joint_target);
			}
		}
	}

	// PLACE->TAKEOUT 小臂接近目标判定的目标锁存
	if (arm->status == ARM_STATE_TAKEOUT && arm->takeout_sync_active && !arm->takeout_small_target_inited) {
		if (arm->target_mode == ARM_TARGET_JOINT) {
			arm->takeout_small_target = arm->arm_joint_target[1];
		} else {
			float final_joint_target[3];
			arm_get_unwrapped_target(arm, arm->final_target_y, arm->final_target_z, arm->final_target_pitch, final_joint_target);
			arm->takeout_small_target = final_joint_target[1];
		}
		arm->takeout_small_target_inited = 1;
	}

	// === 处理翻转过冲状态机 (Phase 0 -> 1 -> 2 -> 0) ===
	if (arm->big_arm_overshoot_armed) {
		float overshoot_sign = -1.0f;
		arm->big_arm_final_joint = joint_target[0];

		// 根据方向决定过冲是+还是-
		if (arm->status == ARM_STATE_PLACE || arm->status == ARM_STATE_TAKEOUT) {
			overshoot_sign = (joint_target[0] >= arm->damiao_1.position) ? 1.0f : -1.0f;
		} else {
			if (arm->flip_transition_dir > 0) overshoot_sign = 1.0f;
			else if (arm->flip_transition_dir < 0) overshoot_sign = -1.0f;
			else overshoot_sign = (joint_target[0] >= arm->damiao_1.position) ? 1.0f : -1.0f;
		}

		arm->big_arm_overshoot_joint = joint_target[0] + overshoot_sign * arm->big_arm_overshoot_rad;

		if (arm->big_arm_overshoot_joint < ARM_BIG_ARM_MIN_ANGLE_RAD) {
        	arm->big_arm_overshoot_joint = ARM_BIG_ARM_MIN_ANGLE_RAD;
    	} 
		
		arm->big_arm_overshoot_phase = 1; // 进入过冲阶段1：施加额外角度
		arm->big_arm_overshoot_armed = 0;
	}

	if (arm->big_arm_overshoot_phase == 1) {
		joint_target[0] = arm->big_arm_overshoot_joint; 

		// 等待到达过冲临时点位
		if (fabsf(arm->damiao_1.position - arm->big_arm_overshoot_joint) < 0.12f) {
			if (arm->status == ARM_STATE_TAKEOUT && arm->takeout_sync_active) {
				if (arm->takeout_sync_phase == 1) {
					// 过冲到位后释放小臂/吸盘，让其先走半程
					arm->flags.takeout_wait_latched = 0;
					arm->flags.small_arm_wait_locked_inited = 0;
					arm->flags.suction_wait_locked_inited = 0;
					arm->takeout_sync_phase = 2;
				}

				if (arm->takeout_sync_phase == 2 && arm->takeout_small_target_inited) {
					uint8_t sync_ready = arm_small_arm_sync_ready(
						arm->takeout_small_target,
						arm->damiao_3.position
					);
					if (sync_ready) {
						arm->big_arm_overshoot_phase = 2;
						arm->takeout_sync_phase = 3;
					}
				}
			} else {
				uint8_t suction_ready = 0;
				uint8_t small_arm_ready = 0;

				// 检查吸盘和小臂是否也已准备就绪，确保整体姿态安全
				if (arm->status == ARM_STATE_TAKEOUT) {
					suction_ready = 1;
					small_arm_ready = 1;
				} else {
					float actual_suction_target = joint_target[2];
					if (arm->status == ARM_STATE_PLACE) {
						actual_suction_target -= ARM_PLACE_SUCTION_OFFSET_RAD;
					}
					
					if (actual_suction_target < -0.1f) actual_suction_target = -0.1f;
					if (actual_suction_target > 2.8f) actual_suction_target = 2.8f;

					float suction_err = arm_wrap_pi(arm->damiao_4.position - actual_suction_target);
					if (fabsf(suction_err) < 0.3f) suction_ready = 1;

					float small_arm_err = arm_wrap_pi(arm->damiao_3.position - joint_target[1]);
					if (fabsf(small_arm_err) < 0.3f) small_arm_ready = 1;
				}

				// 都准备好了，进入阶段2：回弹到真实的设定点
				if (suction_ready && small_arm_ready) {
					arm->big_arm_overshoot_phase = 2; 
				}
			}
		}
	} else if (arm->big_arm_overshoot_phase == 2) {
		joint_target[0] = arm->big_arm_final_joint; 
		// 到达真实设定点后，结束过冲
		if (fabsf(arm->damiao_1.position - arm->big_arm_final_joint) < 0.05f) {
			arm->big_arm_overshoot_phase = 0;
			arm->flip_transition_dir = 0;
			if (arm->takeout_sync_phase == 3) {
				arm->takeout_sync_active = 0;
				arm->takeout_sync_phase = 0;
				arm->takeout_small_target_inited = 0;
			}
		}
	}

	if (joint_target[0] < ARM_BIG_ARM_MIN_ANGLE_RAD) {
		joint_target[0] = ARM_BIG_ARM_MIN_ANGLE_RAD;
	}

	// 装载待下发指令
	joint_cmd[0] = joint_target[0]; 
	joint_cmd[1] = joint_target[1];
	joint_cmd[2] = joint_target[2];
	

	arm->joint_cmd_prev[0] = joint_cmd[0];
	arm->joint_cmd_prev[1] = joint_cmd[1];
	arm->joint_cmd_prev[2] = joint_cmd[2];

	// 交给底层控制逻辑进行滤波和速度分配
	arm_apply_ctrl(arm, joint_cmd);

	// 判断系统是否仍处于活跃运动中 (未到达静态死区)
	arm->arm_motion_active =
		(fabsf(arm->damiao_1.position - joint_cmd[0]) > 0.01f) ||
		(fabsf(arm->damiao_3.position - joint_cmd[1]) > 0.01f) ||
		(fabsf(arm->damiao_4.position - joint_cmd[2]) > 0.01f);
		
	// 顺带做一次正运动学反馈更新，用于给UI或者遥测使用
	robot_arm_fk(arm->damiao_1.position, arm->damiao_3.position, arm->damiao_4.position, 
	             &arm->current_y, &arm->current_z, &arm->current_pitch);
}

/**
 * @brief 执行平滑滤波并调用达妙电机的 PVT (位置速度时间) 控制接口
 */
void arm_apply_ctrl(RobotArm *arm, const float joint_des[3]) {
	float joint0_cmd = joint_des[0];                 
	if (joint0_cmd < ARM_BIG_ARM_MIN_ANGLE_RAD) {
		joint0_cmd = ARM_BIG_ARM_MIN_ANGLE_RAD;
	}
	float joint0_err = joint0_cmd - arm->damiao_1.position;  
	float err_abs = fabsf(joint0_err);               
	float cmd_speed, max_step, delta;

	float joint1_cmd = joint_des[1];                                
	float joint2_cmd = joint_des[2];
	float joint1_final_cmd, joint1_final_speed;
	float joint2_final_cmd, joint2_final_speed;
    
	float small_speed_base, small_speed_gain, small_speed_max;                           
	float place_speed_min, suction_speed_max;                         
	uint8_t place_like_mode;                         

	if (!arm->flags.big_arm_filter_inited) {
		arm->flags.big_arm_filter_inited = 1;
		arm->big_arm_cmd_filtered = arm->damiao_1.position;
	}

	suction_speed_max = ARM_SUCTION_SPEED_MAX;
	
	// 根据状态配置速度包络
	if (arm->status == ARM_STATE_CATCH) {
		small_speed_base = ARM_SMALL_CATCH_SPEED_BASE;
		small_speed_gain = ARM_SMALL_CATCH_SPEED_GAIN;
		small_speed_max = ARM_SMALL_CATCH_SPEED_MAX;
	} else if (arm->flags.place_exit_safety_active) {
		small_speed_base = ARM_SMALL_PLACE_EXIT_SPEED_BASE;
		small_speed_gain = ARM_SMALL_PLACE_EXIT_SPEED_GAIN;
		small_speed_max = ARM_SMALL_PLACE_EXIT_SPEED_MAX;
	} else {
		small_speed_base = ARM_SMALL_SPEED_BASE;
		small_speed_gain = ARM_SMALL_SPEED_GAIN;
		small_speed_max = ARM_SMALL_SPEED_MAX;
	}

	// === 大臂低通滤波与速度规划 ===
	// 采用阶跃限制 (Rate Limit) + 指数滤波 (Alpha Filter) 解决信号剧烈抖动
	max_step = ARM_J8006_CMD_RATE_LIMIT * arm->ctrl_dt;
	delta = arm_clampf(ARM_J8006_FILTER_ALPHA * (joint0_cmd - arm->big_arm_cmd_filtered), -max_step, max_step);
	arm->big_arm_cmd_filtered += delta;
	if (arm->big_arm_cmd_filtered < ARM_BIG_ARM_MIN_ANGLE_RAD) {
		arm->big_arm_cmd_filtered = ARM_BIG_ARM_MIN_ANGLE_RAD;
	}

	// 前馈速度估算 (导数)
	float est_ff = delta / arm->ctrl_dt;
	arm->big_arm_speed_filtered = 0.6f * fabsf(est_ff) + 0.4f * arm->big_arm_speed_filtered; // 低通
	cmd_speed = ARM_J8006_CMD_SPEED_BASE + arm->big_arm_speed_filtered;

	// 分段式速度包络：远距离满速，接近减速，到达后微调慢速
	if (err_abs > ARM_J8006_NEAR_ERR_RAD) {
		cmd_speed = fminf(cmd_speed, ARM_J8006_CMD_SPEED_MAX);
	} else if (err_abs > ARM_J8006_FINE_ERR_RAD) {
		float blend = (err_abs - ARM_J8006_FINE_ERR_RAD) / (ARM_J8006_NEAR_ERR_RAD - ARM_J8006_FINE_ERR_RAD);
		cmd_speed = fminf(cmd_speed, ARM_J8006_FINE_SPEED_MAX + blend * (ARM_J8006_NEAR_SPEED_MAX - ARM_J8006_FINE_SPEED_MAX));
	} else {
		cmd_speed = fminf(cmd_speed, ARM_J8006_FINE_SPEED_MAX);
	}

	// 如果误差方向和当前物理转速反向 (超调)，执行强力刹车限速
	if ((joint0_err * arm->damiao_1.speed) < 0.0f) cmd_speed = fminf(cmd_speed, ARM_J8006_FINE_SPEED_MAX);
	
	// 在过冲回弹的第2阶段限速，防止太猛
	if (arm->big_arm_overshoot_phase == 2) {
		float phase2_limit = (arm->flip_transition_dir != 0) ? ARM_J8006_PHASE2_Q2TOQ1_SPEED_MAX : ARM_J8006_NEAR_SPEED_MAX;
		cmd_speed = fminf(cmd_speed, phase2_limit);
	}

	if (arm->flags.place_exit_safety_active) {
		cmd_speed = fminf(cmd_speed, ARM_BIG_PLACE_EXIT_SPEED_MAX);
	}
	cmd_speed = arm_clampf(cmd_speed, 0.06f, ARM_J8006_CMD_SPEED_MAX);

	if (arm->status == ARM_STATE_PLACE) joint2_cmd -= ARM_PLACE_SUCTION_OFFSET_RAD;

	// === 各类定点锁死等待逻辑 (处理关节间的相对运动干涉) ===
	if (arm->flags.takeout_wait_latched && arm->status == ARM_STATE_TAKEOUT) {
		uint8_t big_arm_lifted = (arm->big_arm_overshoot_phase == 2) || (arm->big_arm_overshoot_phase == 0);
		float big_arm_err = fabsf(arm->damiao_1.position - joint_des[0]);
		uint8_t wait_timeout = (HAL_GetTick() - arm->wait_start_tick >= ARM_TAKEOUT_WAIT_TIMEOUT_MS);

		if ((big_arm_lifted && big_arm_err < 0.40f) || wait_timeout) {
			arm->flags.takeout_wait_latched = 0; 
			arm->flags.small_arm_wait_locked_inited = 0;
			arm->flags.suction_wait_locked_inited = 0;
		}
	} else if (arm->status != ARM_STATE_TAKEOUT) {
		arm->flags.takeout_wait_latched = 0;
	}

	if (arm->flags.suction_wait_latched && arm->status == ARM_STATE_PLACE) {
		float small_arm_err = arm_wrap_pi(arm->damiao_3.position - joint_des[1]);
		uint8_t wait_timeout = (HAL_GetTick() - arm->wait_start_tick >= ARM_SUCTION_WAIT_TIMEOUT_MS);
		if ((fabsf(small_arm_err) <= ARM_PLACE_SMALL_ARM_READY_ERR_RAD) || wait_timeout) {
			arm->flags.suction_wait_latched = 0; 
			arm->flags.suction_wait_locked_inited = 0;
		}
	} else if (arm->status != ARM_STATE_PLACE) {
		arm->flags.suction_wait_latched = 0;
	}

	if (arm->flags.small_arm_wait_latched && arm->status == ARM_STATE_CATCH) {
		float suction_err = arm_wrap_pi(arm->damiao_4.position - joint_des[2]);
		uint8_t wait_timeout = (HAL_GetTick() - arm->wait_start_tick >= ARM_SUCTION_WAIT_TIMEOUT_MS);
		if ((fabsf(suction_err) <= ARM_PLACE_SMALL_ARM_READY_ERR_RAD) || wait_timeout) {
			arm->flags.small_arm_wait_latched = 0; 
			arm->flags.small_arm_wait_locked_inited = 0;
		}
	} else if (arm->status != ARM_STATE_CATCH) {
		arm->flags.small_arm_wait_latched = 0;
	}

	// 处理锁定指令
	if (arm->flags.takeout_wait_latched || arm->flags.small_arm_wait_latched) {
		if (!arm->flags.small_arm_wait_locked_inited) {
			arm->small_arm_wait_locked_pos = arm->damiao_3.position;
			arm->flags.small_arm_wait_locked_inited = 1;
		}
		joint1_final_cmd = arm->small_arm_wait_locked_pos; // 冻结位置
		joint1_final_speed = ARM_SMALL_WAIT_HOLD_SPEED;
	} else {
		joint1_final_cmd = joint1_cmd; // 正常跟随
		joint1_final_speed = arm_clampf(small_speed_base + small_speed_gain * fabsf(joint1_cmd - arm->damiao_3.position), 0.04f, small_speed_max);
	}

	place_like_mode = (arm->status == ARM_STATE_PLACE) || (arm->status == ARM_STATE_TAKEOUT);
	
	if (arm->flags.takeout_wait_latched || arm->flags.suction_wait_latched) {
		if (!arm->flags.suction_wait_locked_inited) {
			arm->suction_wait_locked_pos = arm->damiao_4.position;
			arm->flags.suction_wait_locked_inited = 1;
		}
		joint2_final_cmd = arm->suction_wait_locked_pos;
		joint2_final_speed = ARM_SUCTION_WAIT_HOLD_SPEED;
	} else if (place_like_mode) {
		joint2_final_cmd = joint2_cmd;
		float joint2_err = joint2_cmd - arm->damiao_4.position;
		joint2_final_speed = ARM_SUCTION_PLACE_SPEED_BASE + ARM_SUCTION_PLACE_SPEED_GAIN * fabsf(joint2_err);
		place_speed_min = (fabsf(joint2_err) < ARM_SUCTION_PLACE_FINE_ERR_RAD) ? ARM_SUCTION_PLACE_FINE_SPEED_MIN : ARM_SUCTION_PLACE_SPEED_MIN;
		
		if ((arm->flip_transition_dir != 0) || (arm->big_arm_overshoot_phase != 0)) {
			joint2_final_speed = arm_clampf(joint2_final_speed, place_speed_min, ARM_SUCTION_FLIP_SLOW_SPEED_MAX);
		} else {
			joint2_final_speed = arm_clampf(joint2_final_speed, place_speed_min, suction_speed_max);
		}
		// 反向超调减速刹车
		if ((joint2_err * arm->damiao_4.speed) < 0.0f) joint2_final_speed = fminf(joint2_final_speed, ARM_SUCTION_PLACE_BRAKE_SPEED_MAX);
	} else {
		joint2_final_cmd = joint2_cmd;
		joint2_final_speed = ARM_SUCTION_SPEED_MAX;
	}

	// 最终安全限幅
	if (joint2_final_cmd < -0.1f) joint2_final_cmd = -0.1f;
	if (joint2_final_cmd > 2.8f) joint2_final_cmd = 2.8f;

	dm_pvt_ctrl(&arm->damiao_1, arm->big_arm_cmd_filtered, cmd_speed, 0.65f);
	dm_pvt_ctrl(&arm->damiao_2, -arm->big_arm_cmd_filtered, cmd_speed, 0.65f);
	dm_pvt_ctrl(&arm->damiao_3, joint1_final_cmd, joint1_final_speed, 0.90f);
	dm_pvt_ctrl(&arm->damiao_4, joint2_final_cmd, joint2_final_speed, 0.90f);
}

/**
 * @brief 输入指定的末端位置和吸盘姿态，计算 3 个电机的关节角度
 * @note 单位：长度(mm)，角度(rad)
 * @note 传入的x1, z1为相对于基座的坐标，基座即为两个8006的连接中心点
 * @param x1 目标末端的水平前向坐标 (沿X轴)
 * @param z1 目标末端的垂直坐标
 * @param pitch_angle 吸盘末端期望的绝对俯仰角
 * @param angle 输出的三个电机角度：angle[0]大臂, angle[1]小臂, angle[2]吸盘
 */
void arm_pos_angle(float x1, float z1, float pitch_angle, float angle[3]) {
    float x, z;
    float x_w, z_w; // 小臂与吸盘连接处的坐标
    float m_2, m;
    float a, a2, b, b2;
    float angle1_1, angle1_2;
    float angle2_inner; // 大臂与小臂的实际几何内角

    // 1. 坐标平移 
    x = x1 + DEFAULT_X; 
    z = z1 + DEFAULT_Z;

	if(x < 0)
	{
		x += ARM_3;
		z += ARM_3;
	}

    // 2. 姿态解耦，求腕关节坐标
    x_w = x - ARM_3 * cosf(pitch_angle) - (DEFAULT_ARM3_X - DEFAULT_ARM_1_2) * cosf(pitch_angle);
    z_w = z - ARM_3 * sinf(pitch_angle);
    m_2 = x_w * x_w + z_w * z_w;
    arm_sqrt_f32(m_2, &m);
	
	// 3. 求大臂角度
	b = (ARM_1 * ARM_1 + m_2 - ARM_2 * ARM_2) / (2 * ARM_1 * m);
	b = arm_clampf(b, -1.0f, 1.0f); // 防除以0或无效开方崩溃
	arm_sqrt_f32(1 - b * b, &b2);
	arm_atan2_f32(b2, b, &angle1_1);
	arm_atan2_f32(z_w, x_w, &angle1_2);

	// 4. 求小臂角度
	a = (ARM_1 * ARM_1 + ARM_2 * ARM_2 - m_2) / (2 * ARM_1 * ARM_2);
	a = arm_clampf(a, -1.0f, 1.0f);
	arm_sqrt_f32(1 - a * a, &a2);
	arm_atan2_f32(a2, a, &angle2_inner);

	// 根据目标所在的象限分别计算输出角度
	if (x1 >= 0) {
		// 第一象限：保持原有的几何构型解
		angle[0] = DEFAULT_ANGLE_1 + PI / 2 - angle1_1 - angle1_2;
		angle[1] = angle2_inner - DEFAULT_ANGLE_2;
	} else {
		// 第二象限：切换到另一个解
		angle[0] = DEFAULT_ANGLE_1 + PI / 2 + angle1_1 - angle1_2;
		angle[1] = 2 * PI + (DEFAULT_ANGLE_2 - angle2_inner);
	}

	// 保证大臂不往后倒撞到已放置的箱子
	if(angle[0] < 0.0f){
		angle[0] = 0;
	}
	
	// 5. 求吸盘角度
	angle[2] = -pitch_angle - angle[0] + angle[1] + DEFAULT_ANGLE_3;
	
	// 防止吸盘反转
	if (angle[2] < -0.1f) {
		angle[2] += 2 * PI;
	}
	if (angle[2] > 2 * PI) {
		angle[2] -= 2 * PI;
	}
}

/**
 * @brief 核心数学：正运动学(FK)解算
 * 根据当前三个电机的真实物理角度，正向推导算出机械臂末端的空间坐标 (Y, Z) 和姿态角 (Pitch)。
 * 可用于上位机遥测、UI显示以及闭环校验。
 */
void robot_arm_fk(float joint1, float joint2, float joint3, float *y_out, float *z_out, float *pitch_out) {
	float joint2_norm = joint2;
	float theta_L1, theta_L2, angle2_inner;
	float pitch;

	// 将第二关节规范到 [0, 2PI)，保证跨零点时仍能落到和逆解一致的支链分支
	while (joint2_norm < 0.0f) {
		joint2_norm += 2.0f * PI;
	}
	while (joint2_norm >= 2.0f * PI) {
		joint2_norm -= 2.0f * PI;
	}

	// 通过各个电机的角度换算出真实的 Pitch 倾角
	pitch = -joint3 - joint1 + joint2_norm + DEFAULT_ANGLE_3;

	// 由逆解的两支公式反推腕点所在的两连杆空间构型
	theta_L1 = DEFAULT_ANGLE_1 + (PI / 2.0f) - joint1;
	if (joint2_norm < PI) {
		angle2_inner = joint2_norm + DEFAULT_ANGLE_2;
		theta_L2 = theta_L1 - (PI - angle2_inner);
	} else {
		angle2_inner = 2.0f * PI + DEFAULT_ANGLE_2 - joint2_norm;
		theta_L2 = theta_L1 + (PI - angle2_inner);
	}

	// 用三角函数投影回推腕点坐标，再叠加吸盘末端长度得到末端坐标
	float x_w = ARM_1 * cosf(theta_L1) + ARM_2 * cosf(theta_L2);
	float z_w = ARM_1 * sinf(theta_L1) + ARM_2 * sinf(theta_L2);
	float eff_L3_x = ARM_3 + (DEFAULT_ARM3_X - DEFAULT_ARM_1_2);
	float x = x_w + eff_L3_x * cosf(pitch);
	float z = z_w + ARM_3 * sinf(pitch);

	// 减去默认基座原点偏移后输出
	*y_out = x - DEFAULT_X;
	*z_out = z - DEFAULT_Z;
	*pitch_out = pitch;
}