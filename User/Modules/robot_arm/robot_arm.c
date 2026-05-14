/**
 * @file robot_arm.c
 * @author xinglu
 * @brief 机械臂驱动模块 (核心算法与动力学层)
 * @version 2.2 (撤销小臂最短路径映射，保留过冲死锁修复)
 * @date 2026-04-30
 */

#include "robot_arm.h"
#include <math.h>
#include <string.h> 

/* ---------------- 静态内联工具函数 ---------------- */
static float arm_clampf(float v, float lo, float hi) {
	if (v < lo) return lo;
	if (v > hi) return hi;
	return v;
}

static float arm_wrap_pi(float angle) {
	while (angle > PI) angle -= 2.0f * PI;
	while (angle < -PI) angle += 2.0f * PI;
	return angle;
}

static uint8_t arm_small_arm_sync_ready(float target, float current) {
	return (fabsf(current - target) <= ARM_TAKEOUT_SMALL_ARM_SYNC_ERR_RAD);
}

static uint8_t arm_status_place_like(arm_status_t status) {
	return (status == ARM_STATE_PLACE) || (status == ARM_STATE_WAIT_TAKEOUT);
}

static int8_t arm_get_ik_quadrant(float y, float z, float pitch) {
	float x_w = y + DEFAULT_X - ARM_3 * cosf(pitch) - (DEFAULT_ARM3_X - DEFAULT_ARM_1_2) * cosf(pitch);
	(void)z;
	return (x_w >= 0.0f) ? 1 : -1;
}

/* ---------------- 内部逻辑辅助函数 (解耦嵌套) ---------------- */

static void arm_get_unwrapped_target(RobotArm *arm, float y, float z, float pitch, float out_joints[3]) {
	arm_pos_angle(y, z, pitch, out_joints);
	
	// 1. 大臂最短路径映射与硬限位
	float diff0 = out_joints[0] - arm->damiao_1.position;
	while (diff0 > PI) { diff0 -= 2.0f * PI; out_joints[0] -= 2.0f * PI; }
	while (diff0 < -PI) { diff0 += 2.0f * PI; out_joints[0] += 2.0f * PI; }
	if (out_joints[0] < ARM_BIG_ARM_MIN_ANGLE_RAD) out_joints[0] = ARM_BIG_ARM_MIN_ANGLE_RAD;

	// 2. 吸盘的最短路径映射与安全限幅
	float target2 = out_joints[2];
	float diff2 = target2 - arm->damiao_4.position;
	
	while (diff2 > PI) { 
		if (target2 - 2.0f * PI < -0.1f) break;
		diff2 -= 2.0f * PI; target2 -= 2.0f * PI; 
	}
	while (diff2 < -PI) { 
		if (target2 + 2.0f * PI > 2.8f) break;
		diff2 += 2.0f * PI; target2 += 2.0f * PI; 
	}
	out_joints[2] = arm_clampf(target2, -0.1f, 2.8f);
}

static void arm_update_transition_gate(RobotArm *arm) {
	arm_status_t prev = arm->last_status;
	if (arm->status != arm->last_status) {
		if ((prev == ARM_STATE_CATCH) && arm_status_place_like(arm->status)) {
			arm->flags.suction_wait_latched = 1;
			arm->flags.takeout_wait_latched = 0;
			arm->flags.small_arm_wait_latched = 0; 
			arm->flags.suction_wait_locked_inited = 0;
			arm->wait_start_tick = HAL_GetTick(); 
		} else if (arm_status_place_like(prev) && (arm->status == ARM_STATE_TAKEOUT)) {
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
		} else if (arm_status_place_like(prev) && (arm->status == ARM_STATE_CATCH)) {
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
 * @brief 处理多点避障过渡路径
 */
static void process_multi_transition(RobotArm *arm, float *joint_target) {
	float actual_suction_target = joint_target[2];
	if (arm_status_place_like(arm->status)) {
		actual_suction_target -= ARM_PLACE_SUCTION_OFFSET_RAD;
	}
	actual_suction_target = arm_clampf(actual_suction_target, -0.1f, 2.8f);

	float err0 = fabsf(arm_wrap_pi(arm->damiao_1.position - joint_target[0]));
	float err1 = fabsf(arm_wrap_pi(arm->damiao_3.position - joint_target[1]));
	float err2 = fabsf(arm_wrap_pi(arm->damiao_4.position - actual_suction_target));

	if (arm->status == ARM_STATE_TAKEOUT && arm->takeout_sync_active && arm->takeout_sync_phase >= 2) {
		err0 = 0.0f; 
	}
	
	// 放宽了多点过渡容差至 0.20f，避免物理零位引发的卡死
	if (err0 >= 0.20f || err1 >= 0.20f) return;

	arm->multi_trans_index++;
	if (arm->multi_trans_index < arm->multi_trans_count) {
		arm->arm_target_y = arm->trans_y[arm->multi_trans_index];
		arm->arm_target_z = arm->trans_z[arm->multi_trans_index];
		arm->arm_target_pitch = arm->trans_pitch[arm->multi_trans_index];
		arm_get_unwrapped_target(arm, arm->arm_target_y, arm->arm_target_z, arm->arm_target_pitch, joint_target);
		return;
	}

	arm->multi_trans_active = 0;
	arm->flags.place_exit_safety_active = 0;
	arm->arm_target_y = arm->final_target_y;
	arm->arm_target_z = arm->final_target_z;
	arm->arm_target_pitch = arm->final_target_pitch;
	arm_get_unwrapped_target(arm, arm->arm_target_y, arm->arm_target_z, arm->arm_target_pitch, joint_target);
	
	if (!arm->takeout_sync_active) {
		arm->big_arm_overshoot_rad = ARM_BIG_ARM_FLIP_OVERSHOOT_FORCE_RAD;
		arm->big_arm_overshoot_armed = 1;
	}
}

/**
 * @brief 处理单点安全过渡
 */
static void process_single_transition(RobotArm *arm, float *joint_target) {
	float err0 = fabsf(arm->damiao_1.position - joint_target[0]);
	float err1 = fabsf(arm->damiao_3.position - joint_target[1]);
	float err2 = fabsf(arm->damiao_4.position - joint_target[2]);
	
	if (arm->big_arm_overshoot_phase == 0 && err0 < 0.12f && err1 < 0.12f && err2 < 0.12f) {
		arm->flags.transition_done = 1; 
		arm->arm_target_y = arm->final_target_y;
		arm->arm_target_z = arm->final_target_z;
		arm->arm_target_pitch = arm->final_target_pitch;
		arm_get_unwrapped_target(arm, arm->arm_target_y, arm->arm_target_z, arm->arm_target_pitch, joint_target);
	}
}

/**
 * @brief 处理大臂过冲阶段 1 逻辑
 */
static void process_overshoot_phase1(RobotArm *arm, float *joint_target) {
	joint_target[0] = arm->big_arm_overshoot_joint; 

    // 放宽大臂过冲到位的判定容差 (0.25f)
	if (fabsf(arm->damiao_1.position - arm->big_arm_overshoot_joint) >= 0.25f) return;

	if (arm->status == ARM_STATE_TAKEOUT && arm->takeout_sync_active) {
		if (arm->takeout_sync_phase == 1) {
			arm->flags.takeout_wait_latched = 0;
			arm->flags.small_arm_wait_locked_inited = 0;
			arm->flags.suction_wait_locked_inited = 0;
			arm->takeout_sync_phase = 2;
		} else if (arm->takeout_sync_phase == 2 && arm->takeout_small_target_inited) {
			if (arm_small_arm_sync_ready(arm->takeout_small_target, arm->damiao_3.position)) {
				arm->big_arm_overshoot_phase = 2;
				arm->takeout_sync_phase = 3;
			}
		}
		return;
	}

	uint8_t suction_ready = (arm->status == ARM_STATE_TAKEOUT) ? 1 : 0;
	uint8_t small_arm_ready = suction_ready;

	if (!suction_ready) {
		float actual_suction_target = joint_target[2];
		if (arm_status_place_like(arm->status)) actual_suction_target -= ARM_PLACE_SUCTION_OFFSET_RAD;
		actual_suction_target = arm_clampf(actual_suction_target, -0.1f, 2.8f);

        // 引入稳态速度判断，防止小臂走远路时引发的死锁
		float err_suction = fabsf(arm_wrap_pi(arm->damiao_4.position - actual_suction_target));
		float err_small   = fabsf(arm_wrap_pi(arm->damiao_3.position - joint_target[1]));

		if (err_suction < 0.40f || fabsf(arm->damiao_4.speed) < 0.10f) suction_ready = 1;
		if (err_small < 0.40f || fabsf(arm->damiao_3.speed) < 0.10f) small_arm_ready = 1;
	}

    // 当小臂和吸盘都到位（或停稳）时，立即触发大臂归位 (Phase 2)
	if (small_arm_ready) {
		arm->big_arm_overshoot_phase = 2; 
	}
}

/* ---------------- 驱动层主接口 ---------------- */

void robot_arm_system_init(RobotArm *arm) {
	memset(arm, 0, sizeof(RobotArm)); 

	dm_motor_init(&arm->damiao_1, 0x11, 0x01, DM_MODE_PVT, DM_J8006, 3.14f, 10.0f, 10.0f, can3_selected);
	dm_motor_init(&arm->damiao_2, 0x12, 0x02, DM_MODE_PVT, DM_J8006, 3.14f, 10.0f, 10.0f, can3_selected);
	dm_motor_init(&arm->damiao_3, 0x13, 0x03, DM_MODE_PVT, DM_J4340, 3.14f, 10.0f, 10.0f, can3_selected);
	dm_motor_init(&arm->damiao_4, 0x14, 0x04, DM_MODE_PVT, DM_J4310, 3.14f, 10.0f, 10.0f, can3_selected);

	dm_motor_enable(&arm->damiao_1);
	dm_motor_enable(&arm->damiao_2);
	dm_motor_enable(&arm->damiao_3);
	dm_motor_enable(&arm->damiao_4);

	arm->target_mode = ARM_TARGET_CARTESIAN;
	arm->status = ARM_STATE_INIT;
	arm->last_status = ARM_STATE_INIT;
	arm->ctrl_dt = ARM_CTRL_DT_DEFAULT;

	arm->joint_cmd_prev[0] = DEFAULT_ANGLE_1;
	arm->joint_cmd_prev[1] = DEFAULT_ANGLE_2;
	arm->joint_cmd_prev[2] = DEFAULT_ANGLE_3;
	arm->big_arm_cmd_filtered = DEFAULT_ANGLE_1;
}

void robot_arm_set_ctrl_dt(RobotArm *arm, float dt_s) {
	arm->ctrl_dt = arm_clampf(dt_s, 0.001f, 0.2f);
}

void robot_arm_set_target(RobotArm *arm, float y, float z, float pitch) {
	arm->target_mode = ARM_TARGET_CARTESIAN;
	arm->flags.place_exit_safety_active = 0;
	arm->flags.place_entry_sync_active = 0;

	int8_t target_quadrant = arm_get_ik_quadrant(y, z, pitch);
	arm->flip_transition_dir = 0;
	arm->big_arm_overshoot_armed = 0;
	arm->big_arm_overshoot_phase = 0;

	if (arm->last_target_quadrant != 0) {
		if ((arm->last_target_quadrant < 0) && (target_quadrant > 0)) {
			arm->flip_transition_dir = 1;   
		} else if ((arm->last_target_quadrant > 0) && (target_quadrant < 0)) {
			arm->flip_transition_dir = -1;  
		}
	}

	uint8_t delay_overshoot = 0; 

	if (arm_status_place_like(arm->last_status) && !arm_status_place_like(arm->status) &&
	    arm->status != ARM_STATE_TAKEOUT && arm->status != ARM_STATE_INIT) {
		// 1. 从货架退出的安全抽离过渡
		arm->multi_trans_active = 1;
		arm->multi_trans_index = 0;
		arm->multi_trans_count = 1;

		arm->trans_y[0] = arm->final_target_y + 100.0f;
		arm->trans_z[0] = arm->final_target_z + 100.0f;
		arm->trans_pitch[0] = pitch;

		arm->final_target_y = y; arm->final_target_z = z; arm->final_target_pitch = pitch;
		arm->arm_target_y = arm->trans_y[0]; arm->arm_target_z = arm->trans_z[0]; arm->arm_target_pitch = arm->trans_pitch[0];

		arm->flags.has_transition = 0;
		arm->flags.place_exit_safety_active = 1;
	} else if (arm->last_status != arm->status && arm_status_place_like(arm->status)) {
		// 2. 抓取到放置的三点高空过渡
		arm->multi_trans_active = 1;
		arm->multi_trans_index = 0;
		arm->multi_trans_count = 3; 
		
		arm->trans_y[0] = 468.158f; arm->trans_z[0] = 183.510f; arm->trans_pitch[0] = 0.0f;
		arm->trans_y[1] = 422.826f; arm->trans_z[1] = 775.271f; arm->trans_pitch[1] = PI/2.0f;
		arm->trans_y[2] = 280.000f; arm->trans_z[2] = 785.030f; arm->trans_pitch[2] = PI/2.0f;
		
		arm->final_target_y = y; 
		arm->final_target_z = z; 
		arm->final_target_pitch = pitch;
		arm->arm_target_y = arm->trans_y[0]; 
		arm->arm_target_z = arm->trans_z[0]; 
		arm->arm_target_pitch = arm->trans_pitch[0];
		
		arm->flags.has_transition = 0; 
		delay_overshoot = 1; 
		arm->flags.place_entry_sync_active = 1;

	// 用象限翻转标志作为唯一判定条件
	} else if (arm->flip_transition_dir != 0) {
		// 3. 只有跨越象限时，才触发单点防撞过渡
		arm->multi_trans_active = 0;
		arm->flags.has_transition = 1;
		arm->flags.transition_done = 0;
		arm->final_target_y = y; 
		arm->final_target_z = z; 
		arm->final_target_pitch = pitch;

		// 生成跨象限时的过渡点
		float offset_y = ARM_TRANS_Y_OFFSET;
		arm->arm_target_y = (y >= 0.0f) ? ((y > offset_y) ? (y - offset_y) : 0.0f) : ((y < -offset_y) ? (y + offset_y) : 0.0f);
		arm->arm_target_z = z + ARM_TRANS_Z_OFFSET;
		arm->arm_target_pitch = pitch;
	} else {
		// 4. 兜底策略：只要不跨象限 (包括 INIT->READY, READY->CATCH 等)，全部直达！
		arm->multi_trans_active = 0;
		arm->flags.has_transition = 0;
		arm->flags.transition_done = 1; 
		arm->final_target_y = y; arm->final_target_z = z; arm->final_target_pitch = pitch;
		arm->arm_target_y = y; arm->arm_target_z = z; arm->arm_target_pitch = pitch;
	}

	if (!delay_overshoot) {
		if (arm_status_place_like(arm->status) || arm->status == ARM_STATE_TAKEOUT) {
			arm->big_arm_overshoot_rad = ARM_BIG_ARM_FLIP_OVERSHOOT_FORCE_RAD;
			arm->big_arm_overshoot_armed = 1;
		} else if (arm->flip_transition_dir != 0 && z < 800.0f) {
			arm->big_arm_overshoot_rad = ARM_BIG_ARM_FLIP_OVERSHOOT_FORCE_RAD;
			arm->big_arm_overshoot_armed = 1;
		}
	}
	arm->last_target_quadrant = target_quadrant;
}

void robot_arm_set_joint_target(RobotArm *arm, float joint1, float joint2, float joint3) {
	arm->arm_joint_target[0] = (joint1 < ARM_BIG_ARM_MIN_ANGLE_RAD) ? ARM_BIG_ARM_MIN_ANGLE_RAD : joint1;
	arm->arm_joint_target[1] = joint2;
	arm->arm_joint_target[2] = joint3;
	arm->target_mode = ARM_TARGET_JOINT;
	arm->big_arm_overshoot_armed = 0;
	arm->big_arm_overshoot_phase = 0;
}

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

void robot_arm_update(RobotArm *arm) {
	float joint_target[3];
	float joint_cmd[3];

	if (!arm->flags.joint_cmd_inited) {
		arm->flags.joint_cmd_inited = 1;
		arm->joint_cmd_prev[0] = arm->damiao_1.position;
		arm->joint_cmd_prev[1] = arm->damiao_3.position;
		arm->joint_cmd_prev[2] = arm->damiao_4.position;
	}

	arm_update_transition_gate(arm);

	if (arm->target_mode == ARM_TARGET_JOINT) {
		joint_target[0] = arm->arm_joint_target[0];
		joint_target[1] = arm->arm_joint_target[1];
		joint_target[2] = arm_clampf(arm->arm_joint_target[2], -0.1f, 2.8f);
	} else {
		arm_get_unwrapped_target(arm, arm->arm_target_y, arm->arm_target_z, arm->arm_target_pitch, joint_target);
		
		if (arm->multi_trans_active) {
			process_multi_transition(arm, joint_target);
		} else if (arm->flags.has_transition && !arm->flags.transition_done) {
			process_single_transition(arm, joint_target);
		}
	}

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

	if (arm->big_arm_overshoot_armed) {
		float overshoot_sign = -1.0f;
		arm->big_arm_final_joint = joint_target[0];

		if (arm_status_place_like(arm->status) || arm->status == ARM_STATE_TAKEOUT) {
			overshoot_sign = (joint_target[0] >= arm->damiao_1.position) ? 1.0f : -1.0f;
		} else {
			if (arm->flip_transition_dir > 0) overshoot_sign = 1.0f;
			else if (arm->flip_transition_dir < 0) overshoot_sign = -1.0f;
			else overshoot_sign = (joint_target[0] >= arm->damiao_1.position) ? 1.0f : -1.0f;
		}

		arm->big_arm_overshoot_joint = joint_target[0] + overshoot_sign * arm->big_arm_overshoot_rad;
		if (arm->big_arm_overshoot_joint < ARM_BIG_ARM_MIN_ANGLE_RAD) arm->big_arm_overshoot_joint = ARM_BIG_ARM_MIN_ANGLE_RAD;
		
		arm->big_arm_overshoot_phase = 1; 
		arm->big_arm_overshoot_armed = 0;
	}

	if (arm->big_arm_overshoot_phase == 1) {
		process_overshoot_phase1(arm, joint_target);
	} else if (arm->big_arm_overshoot_phase == 2) {
		joint_target[0] = arm->big_arm_final_joint; 
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

	if (joint_target[0] < ARM_BIG_ARM_MIN_ANGLE_RAD) joint_target[0] = ARM_BIG_ARM_MIN_ANGLE_RAD;

	joint_cmd[0] = joint_target[0]; 
	joint_cmd[1] = joint_target[1];
	joint_cmd[2] = joint_target[2];
	
	arm->joint_cmd_prev[0] = joint_cmd[0];
	arm->joint_cmd_prev[1] = joint_cmd[1];
	arm->joint_cmd_prev[2] = joint_cmd[2];

	arm_apply_ctrl(arm, joint_cmd);

	arm->arm_motion_active = (fabsf(arm->damiao_1.position - joint_cmd[0]) > 0.03f) ||
	                         (fabsf(arm->damiao_3.position - joint_cmd[1]) > 0.03f) ||
	                         (fabsf(arm->damiao_4.position - joint_cmd[2]) > 0.03f);
		
	robot_arm_fk(arm->damiao_1.position, arm->damiao_3.position, arm->damiao_4.position, 
	             &arm->current_y, &arm->current_z, &arm->current_pitch);
}

void arm_apply_ctrl(RobotArm *arm, const float joint_des[3]) {
	float joint0_cmd = (joint_des[0] < ARM_BIG_ARM_MIN_ANGLE_RAD) ? ARM_BIG_ARM_MIN_ANGLE_RAD : joint_des[0];
	float joint0_err = joint0_cmd - arm->damiao_1.position;  
	float err_abs = fabsf(joint0_err);               
	float cmd_speed, max_step, delta;

	float joint1_cmd = joint_des[1];                                
	float joint2_cmd = joint_des[2];
	float joint1_final_cmd, joint1_final_speed;
	float joint2_final_cmd, joint2_final_speed;
    
	float small_speed_base, small_speed_gain, small_speed_max;                           
	float place_speed_min, suction_speed_max;                         

	if (!arm->flags.big_arm_filter_inited) {
		arm->flags.big_arm_filter_inited = 1;
		arm->big_arm_cmd_filtered = arm->damiao_1.position;
	}

	suction_speed_max = ARM_SUCTION_SPEED_MAX;
	
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

	max_step = ARM_J8006_CMD_RATE_LIMIT * arm->ctrl_dt;
	delta = arm_clampf(ARM_J8006_FILTER_ALPHA * (joint0_cmd - arm->big_arm_cmd_filtered), -max_step, max_step);
	arm->big_arm_cmd_filtered += delta;
	if (arm->big_arm_cmd_filtered < ARM_BIG_ARM_MIN_ANGLE_RAD){
		arm->big_arm_cmd_filtered = ARM_BIG_ARM_MIN_ANGLE_RAD;
	}

	float est_ff = delta / arm->ctrl_dt;
	arm->big_arm_speed_filtered = 0.6f * fabsf(est_ff) + 0.4f * arm->big_arm_speed_filtered; 
	cmd_speed = ARM_J8006_CMD_SPEED_BASE + arm->big_arm_speed_filtered;

	if (err_abs > ARM_J8006_NEAR_ERR_RAD) {
		cmd_speed = fminf(cmd_speed, ARM_J8006_CMD_SPEED_MAX);
	} else if (err_abs > ARM_J8006_FINE_ERR_RAD) {
		float blend = (err_abs - ARM_J8006_FINE_ERR_RAD) / (ARM_J8006_NEAR_ERR_RAD - ARM_J8006_FINE_ERR_RAD);
		cmd_speed = fminf(cmd_speed, ARM_J8006_FINE_SPEED_MAX + blend * (ARM_J8006_NEAR_SPEED_MAX - ARM_J8006_FINE_SPEED_MAX));
	} else {
		cmd_speed = fminf(cmd_speed, ARM_J8006_FINE_SPEED_MAX);
	}

	if ((joint0_err * arm->damiao_1.speed) < 0.0f) cmd_speed = fminf(cmd_speed, ARM_J8006_FINE_SPEED_MAX);
	
	if (arm->big_arm_overshoot_phase == 2) {
		float phase2_limit = (arm->flip_transition_dir != 0) ? ARM_J8006_PHASE2_Q2TOQ1_SPEED_MAX : ARM_J8006_NEAR_SPEED_MAX;
		cmd_speed = fminf(cmd_speed, phase2_limit);
	}

	if (arm->flags.place_exit_safety_active) cmd_speed = fminf(cmd_speed, ARM_BIG_PLACE_EXIT_SPEED_MAX);
	cmd_speed = arm_clampf(cmd_speed, 0.06f, ARM_J8006_CMD_SPEED_MAX);

	if (arm_status_place_like(arm->status)) joint2_cmd -= ARM_PLACE_SUCTION_OFFSET_RAD;

	if (arm->flags.takeout_wait_latched && arm->status == ARM_STATE_TAKEOUT) {
		uint8_t big_arm_lifted = (arm->big_arm_overshoot_phase == 2) || (arm->big_arm_overshoot_phase == 0);
		if ((big_arm_lifted && fabsf(arm->damiao_1.position - joint_des[0]) < 0.40f) || (HAL_GetTick() - arm->wait_start_tick >= ARM_TAKEOUT_WAIT_TIMEOUT_MS)) {
			arm->flags.takeout_wait_latched = 0; 
			arm->flags.small_arm_wait_locked_inited = 0;
			arm->flags.suction_wait_locked_inited = 0;
		}
	} else if (arm->status != ARM_STATE_TAKEOUT) {
		arm->flags.takeout_wait_latched = 0;
	}

	if (arm->flags.suction_wait_latched && arm_status_place_like(arm->status)) {
		if ((fabsf(arm_wrap_pi(arm->damiao_3.position - joint_des[1])) <= ARM_PLACE_SMALL_ARM_READY_ERR_RAD) || (HAL_GetTick() - arm->wait_start_tick >= ARM_SUCTION_WAIT_TIMEOUT_MS)) {
			arm->flags.suction_wait_latched = 0; 
			arm->flags.suction_wait_locked_inited = 0;
		}
	} else if (!arm_status_place_like(arm->status)) {
		arm->flags.suction_wait_latched = 0;
	}

	if (arm->flags.small_arm_wait_latched && arm->status == ARM_STATE_CATCH) {
		if ((fabsf(arm_wrap_pi(arm->damiao_4.position - joint_des[2])) <= ARM_PLACE_SMALL_ARM_READY_ERR_RAD) || (HAL_GetTick() - arm->wait_start_tick >= ARM_SUCTION_WAIT_TIMEOUT_MS)) {
			arm->flags.small_arm_wait_latched = 0; 
			arm->flags.small_arm_wait_locked_inited = 0;
		}
	} else if (arm->status != ARM_STATE_CATCH) {
		arm->flags.small_arm_wait_latched = 0;
	}

	if (arm->flags.takeout_wait_latched || arm->flags.small_arm_wait_latched) {
		if (!arm->flags.small_arm_wait_locked_inited) {
			arm->small_arm_wait_locked_pos = arm->damiao_3.position;
			arm->flags.small_arm_wait_locked_inited = 1;
		}
		joint1_final_cmd = arm->small_arm_wait_locked_pos; 
		joint1_final_speed = ARM_SMALL_WAIT_HOLD_SPEED;
	} else {
		joint1_final_cmd = joint1_cmd; 
		joint1_final_speed = arm_clampf(small_speed_base + small_speed_gain * fabsf(joint1_cmd - arm->damiao_3.position), 0.04f, small_speed_max);
	}

	uint8_t place_like_mode = arm_status_place_like(arm->status) || (arm->status == ARM_STATE_TAKEOUT);
	
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
		if ((joint2_err * arm->damiao_4.speed) < 0.0f) joint2_final_speed = fminf(joint2_final_speed, ARM_SUCTION_PLACE_BRAKE_SPEED_MAX);
	} else {
		joint2_final_cmd = joint2_cmd;
		joint2_final_speed = ARM_SUCTION_SPEED_MAX;
	}

	joint2_final_cmd = arm_clampf(joint2_final_cmd, -0.1f, 2.8f);

	dm_pvt_ctrl(&arm->damiao_1, arm->big_arm_cmd_filtered, cmd_speed, 0.65f);
	dm_pvt_ctrl(&arm->damiao_2, -arm->big_arm_cmd_filtered, cmd_speed, 0.65f);
	dm_pvt_ctrl(&arm->damiao_3, joint1_final_cmd, joint1_final_speed, 0.90f);
	dm_pvt_ctrl(&arm->damiao_4, joint2_final_cmd, joint2_final_speed, 0.90f);
}

void arm_pos_angle(float x1, float z1, float pitch_angle, float angle[3]) {
    float x = x1 + DEFAULT_X; 
    float z = z1 + DEFAULT_Z;
	
	if (x < 0) {
		x += ARM_3; 
		z += ARM_3;
	}

    float x_w = x - ARM_3 * cosf(pitch_angle) - (DEFAULT_ARM3_X - DEFAULT_ARM_1_2) * cosf(pitch_angle);
    float z_w = z - ARM_3 * sinf(pitch_angle);
    float m_2 = x_w * x_w + z_w * z_w;
    float m, b, b2, angle1_1, angle1_2, a, a2, angle2_inner;
    arm_sqrt_f32(m_2, &m);
	
	b = arm_clampf((ARM_1 * ARM_1 + m_2 - ARM_2 * ARM_2) / (2 * ARM_1 * m), -1.0f, 1.0f); 
	arm_sqrt_f32(1 - b * b, &b2);
	arm_atan2_f32(b2, b, &angle1_1);
	arm_atan2_f32(z_w, x_w, &angle1_2);

	a = arm_clampf((ARM_1 * ARM_1 + ARM_2 * ARM_2 - m_2) / (2 * ARM_1 * ARM_2), -1.0f, 1.0f);
	arm_sqrt_f32(1 - a * a, &a2);
	arm_atan2_f32(a2, a, &angle2_inner);

	if (x1 >= 0) {
		angle[0] = DEFAULT_ANGLE_1 + PI / 2 - angle1_1 - angle1_2;
		angle[1] = angle2_inner - DEFAULT_ANGLE_2;
	} else {
		angle[0] = DEFAULT_ANGLE_1 + PI / 2 + angle1_1 - angle1_2;
		angle[1] = 2 * PI + (DEFAULT_ANGLE_2 - angle2_inner);
	}

	if (angle[0] < 0.0f) angle[0] = 0;
	
	angle[2] = -pitch_angle - angle[0] + angle[1] + DEFAULT_ANGLE_3;
	
	if (angle[2] < -0.1f) angle[2] += 2 * PI;
	if (angle[2] > 2 * PI) angle[2] -= 2 * PI;
}

void robot_arm_fk(float joint1, float joint2, float joint3, float *y_out, float *z_out, float *pitch_out) {
	float joint2_norm = joint2;
	while (joint2_norm < 0.0f) joint2_norm += 2.0f * PI;
	while (joint2_norm >= 2.0f * PI) joint2_norm -= 2.0f * PI;

	float pitch = -joint3 - joint1 + joint2_norm + DEFAULT_ANGLE_3;
	float theta_L1 = DEFAULT_ANGLE_1 + (PI / 2.0f) - joint1;
	float theta_L2, angle2_inner;
	
	if (joint2_norm < PI) {
		angle2_inner = joint2_norm + DEFAULT_ANGLE_2;
		theta_L2 = theta_L1 - (PI - angle2_inner);
	} else {
		angle2_inner = 2.0f * PI + DEFAULT_ANGLE_2 - joint2_norm;
		theta_L2 = theta_L1 + (PI - angle2_inner);
	}

	float x_w = ARM_1 * cosf(theta_L1) + ARM_2 * cosf(theta_L2);
	float z_w = ARM_1 * sinf(theta_L1) + ARM_2 * sinf(theta_L2);
	float eff_L3_x = ARM_3 + (DEFAULT_ARM3_X - DEFAULT_ARM_1_2);
	float x = x_w + eff_L3_x * cosf(pitch);
	float z = z_w + ARM_3 * sinf(pitch);

	*y_out = x - DEFAULT_X;
	*z_out = z - DEFAULT_Z;
	*pitch_out = pitch;
}