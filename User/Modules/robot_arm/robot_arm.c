/**
 * @file robot_arm.c
 * @author xinglu
 * @brief 机械臂驱动模块
 *
 * @note 当前策略：
 * 1. 保留大臂独立轨迹规划（含过冲翻转）；
 * 2. 删除小臂与吸盘的轨迹规划，仅做必要的命令平滑；
 * 3. 状态机收敛为初始态/抓取态/放置态；
 * 4. 仅抓取态->放置态时，吸盘等待其他臂运动完成；其余状态切换直接快速下发。
 *
 * @version 2.0
 * @date 2026-04-24
 */

#include "robot_arm.h"
#include <math.h>
#include "trajectory_plan/trajectory_plan.h"

#define ARM_1           450.0f
#define ARM_2           450.0f
#define ARM_3           105.0f
#define DEFAULT_ANGLE_1 0.1645f
#define DEFAULT_ANGLE_2 0.1747f
#define DEFAULT_ANGLE_3 0.9155f
#define DEFAULT_X       0.0f
#define DEFAULT_Z       0.0f
#define DEFAULT_ARM_1_2 66.5f
#define DEFAULT_ARM3_X  85.72f

/* 控制主周期，单位秒；需与任务实际周期匹配，避免轨迹推进与控制下发失配。 */
#define ARM_CTRL_DT_DEFAULT                 0.02f

/* 大臂(J8006)控制参数：负载工况下收敛更稳，降低来回修正导致的抖动。 */
#define ARM_J8006_CMD_SPEED_BASE            0.55f
#define ARM_J8006_CMD_SPEED_MAX             1.60f
#define ARM_J8006_CMD_SPEED_FF_GAIN         0.14f
#define ARM_J8006_FILTER_ALPHA              0.24f
#define ARM_J8006_CMD_RATE_LIMIT            4.20f
#define ARM_J8006_HOLD_ERR_RAD              0.015f
#define ARM_J8006_NEAR_ERR_RAD              0.10f
#define ARM_J8006_FINE_ERR_RAD              0.04f
#define ARM_J8006_NEAR_SPEED_MAX            0.85f
#define ARM_J8006_FINE_SPEED_MAX            0.48f
#define ARM_J8006_SWITCH_SPEED_EPS          0.06f
#define ARM_J8006_PHASE2_Q2TOQ1_SPEED_MAX   0.48f

/* 大臂轨迹与翻转过冲参数。 */
#define ARM_BIG_ARM_PLAN_MAX_SPEED          1.80f
#define ARM_BIG_ARM_PLAN_MAX_ACCEL          2.60f
#define ARM_BIG_ARM_FLIP_OVERSHOOT_FORCE_RAD 0.35f
#define ARM_FLIP_Q2_TO_Q1_OVERSHOOT_GAIN    0.12f
#define ARM_FLIP_Q2_TO_Q1_OVERSHOOT_MIN     0.25f
#define ARM_FLIP_Q2_TO_Q1_OVERSHOOT_MAX     0.40f

/* 小臂(J4340)参数：在负载下提高阻尼，兼顾速度与稳定。 */
#define ARM_SMALL_CMD_ALPHA                 0.16f
#define ARM_SMALL_CMD_RATE_LIMIT            1.80f
#define ARM_SMALL_SPEED_BASE                0.28f
#define ARM_SMALL_SPEED_GAIN                0.90f
#define ARM_SMALL_SPEED_MAX                 1.00f
#define ARM_SMALL_SPEED_FILTER_ALPHA        0.20f

/* 抓取态：小臂提速，缩短切入抓取位耗时。 */
#define ARM_SMALL_CATCH_CMD_ALPHA           0.45f
#define ARM_SMALL_CATCH_CMD_RATE_LIMIT      3.20f
#define ARM_SMALL_CATCH_SPEED_BASE          0.58f
#define ARM_SMALL_CATCH_SPEED_GAIN          1.80f
#define ARM_SMALL_CATCH_SPEED_MAX           2.20f
#define ARM_SMALL_CATCH_SPEED_FILTER_ALPHA  0.30f

/* 吸盘(J4310)参数：全程慢速跟踪，翻转阶段进一步压低速度，避免箱体抢先到位。 */
#define ARM_SUCTION_CMD_ALPHA               0.14f
#define ARM_SUCTION_CMD_RATE_LIMIT          0.60f
#define ARM_SUCTION_SPEED_BASE              0.24f
#define ARM_SUCTION_SPEED_GAIN              0.65f
#define ARM_SUCTION_SPEED_MAX               0.90f
#define ARM_SUCTION_SPEED_FILTER_ALPHA      0.20f

/* 翻转阶段吸盘速度上限：大臂翻转没结束前，吸盘只能慢慢跟。 */
#define ARM_SUCTION_FLIP_SLOW_SPEED_MAX     0.25f

/* 放置态：等小臂到位后再放行大臂回落，并把吸盘速度再提一档。 */
#define ARM_PLACE_SMALL_ARM_READY_ERR_RAD   0.05f
#define ARM_SUCTION_PLACE_SPEED_BASE        0.24f
#define ARM_SUCTION_PLACE_SPEED_GAIN        0.80f
#define ARM_SUCTION_PLACE_SPEED_MIN         0.30f
#define ARM_SUCTION_PLACE_SPEED_MAX         1.00f
#define ARM_SUCTION_PLACE_FINE_ERR_RAD      0.03f
#define ARM_SUCTION_PLACE_FINE_SPEED_MIN    0.08f
#define ARM_SUCTION_PLACE_BRAKE_SPEED_MAX   0.16f

/* 抓取态->放置态等待门控期间，吸盘保持当前位置时使用的低速。 */
#define ARM_SUCTION_WAIT_HOLD_SPEED         0.12f

/* 放置态吸盘目标补偿：少转固定角度，便于后续在线调参。 */
#define ARM_PLACE_SUCTION_OFFSET_RAD        -0.1f

uint8_t g_joint_cmd_inited[3];          /* 三个关节的指令初始标志，避免首次运行时使用未初始化命令。 */
float g_joint_cmd_prev[3];              /* 上一次下发的关节目标，给轨迹重建和过冲计算用。 */

uint8_t g_big_arm_plan_inited;          /* 大臂独立轨迹是否已经初始化。 */
float g_big_arm_start;                  /* 当前大臂轨迹起点。 */
float g_big_arm_target;                 /* 当前大臂轨迹终点。 */
Trajectory_Handler_t g_big_arm_traj;    /* 大臂独立轨迹规划器句柄。 */

float g_arm_ctrl_dt = ARM_CTRL_DT_DEFAULT;  /* 控制周期，轨迹步进必须与任务周期一致。 */

uint8_t g_joint_cmd_filter_inited[3];   /* 三个关节的平滑滤波器初始化标志。 */
float g_joint_cmd_filtered[3];          /* 平滑后的关节目标，用于抑制抖动。 */
float g_joint_speed_filtered[3];        /* 平滑后的关节速度上限。 */
float g_big_arm_ff;                     /* 大臂轨迹速度前馈项。 */

uint8_t g_overshoot_stable_cnt;         /* 过冲阶段稳定计数，避免抖动导致过早切相。 */
int8_t g_last_target_quadrant;          /* 上一次末端目标对应的逆解象限。 */
int8_t g_flip_transition_dir;           /* 翻转方向：1 表示 Q2->Q1，-1 表示 Q1->Q2，0 表示无翻转。 */
arm_status_t g_prev_status = ARM_STATE_INIT;  /* 上一次状态机状态，用于判断状态切换。 */
uint8_t g_suction_wait_latched;         /* 抓取态->放置态时锁存吸盘等待。 */
float g_suction_wait_locked_pos;        /* 吸盘等待时的锁定位置，进入等待时保存，避免命令追踪当前位置。 */
uint8_t g_suction_wait_locked_inited;   /* 吸盘等待锁定位置是否已初始化。 */

arm_status_t g_last_status = ARM_STATE_INIT;  /* 上一次状态机状态，用于检测状态切换。 */

static float arm_clampf(float v, float lo, float hi) {
	if (v < lo) {
		return lo;
	}
	if (v > hi) {
		return hi;
	}
	return v;
}

/**
 * @brief 根据腕点 x_w 的符号判断逆解象限。
 */
static int8_t arm_get_ik_quadrant(float y, float z, float pitch) {
	float x_w = y + DEFAULT_X - ARM_3 * cosf(pitch) -
				(DEFAULT_ARM3_X - DEFAULT_ARM_1_2) * cosf(pitch);
	(void)z;

	return (x_w >= 0.0f) ? 1 : -1;
}

/**
 * @brief 判断大臂独立轨迹目标是否变化。
 */
static uint8_t arm_big_arm_target_changed(float target) {
	const float target_eps = 1e-5f;

	if (!g_big_arm_plan_inited) {
		return 1;
	}

	return fabsf(target - g_big_arm_target) > target_eps;
}

/**
 * @brief 重建大臂独立轨迹。
 */
static void arm_rebuild_big_arm_plan(float target) {
	float delta;
	float alpha_v_max;
	float alpha_a_max;

	g_big_arm_start = g_joint_cmd_prev[0];
	g_big_arm_target = target;
	delta = fabsf(g_big_arm_target - g_big_arm_start);

	if (delta <= 1e-4f) {
		t_trajectory_init(&g_big_arm_traj, 1.0f, 1.0f, 1.0f, 1.0f, g_arm_ctrl_dt);
		g_big_arm_ff = 0.0f;
		g_big_arm_plan_inited = 1;
		return;
	}

	alpha_v_max = ARM_BIG_ARM_PLAN_MAX_SPEED / delta;
	alpha_a_max = ARM_BIG_ARM_PLAN_MAX_ACCEL / delta;

	if (alpha_v_max < 0.08f) {
		alpha_v_max = 0.08f;
	}
	if (alpha_a_max < 0.20f) {
		alpha_a_max = 0.20f;
	}

	t_trajectory_init(&g_big_arm_traj, 0.0f, 1.0f, alpha_v_max, alpha_a_max, g_arm_ctrl_dt);
	g_big_arm_plan_inited = 1;
}

/**
 * @brief 推进大臂轨迹一步。
 */
static void arm_big_arm_step(float target, float *out_cmd) {
	float alpha;
	float alpha_w;

	if (arm_big_arm_target_changed(target)) {
		arm_rebuild_big_arm_plan(target);
	}

	t_trajectory_update(&g_big_arm_traj, &alpha, &alpha_w);
	*out_cmd = g_big_arm_start + (g_big_arm_target - g_big_arm_start) * alpha;
	g_big_arm_ff = (g_big_arm_target - g_big_arm_start) * alpha_w;
	g_joint_cmd_prev[0] = *out_cmd;
}

/**
 * @brief 检查状态切换并更新抓取->放置的时序门控。
 */
static void arm_update_transition_gate(RobotArm *arm) {
	g_prev_status = g_last_status;
	if (arm->status != g_last_status) {
		if ((g_prev_status == ARM_STATE_CATCH) && (arm->status == ARM_STATE_PLACE)) {
			g_suction_wait_latched = 1;
		}
		g_last_status = arm->status;
	}
}

/**
 * @brief 初始化机械臂系统：电机、状态与控制器内部变量。
 */
void robot_arm_system_init(RobotArm *arm) {
	dm_motor_init(&arm->damiao_1, 0x11, 0x01, DM_MODE_POS_SPEED, DM_J8006, 3.14f, 45.0f,
				  20.0f, can3_selected);
	dm_motor_init(&arm->damiao_2, 0x12, 0x02, DM_MODE_POS_SPEED, DM_J8006, 3.14f, 45.0f,
				  20.0f, can3_selected);
	dm_motor_init(&arm->damiao_3, 0x13, 0x03, DM_MODE_POS_SPEED, DM_J4340, 3.14f, 45.0f,
				  20.0f, can3_selected);
	dm_motor_init(&arm->damiao_4, 0x14, 0x04, DM_MODE_POS_SPEED, DM_J4310, 3.14f, 45.0f,
				  20.0f, can3_selected);

	dm_motor_enable(&arm->damiao_1);
	dm_motor_enable(&arm->damiao_2);
	dm_motor_enable(&arm->damiao_3);
	dm_motor_enable(&arm->damiao_4);

	arm->arm_target_y = 0.0f;
	arm->arm_target_z = 0.0f;
	arm->arm_target_pitch = 0.0f;

	arm->arm_motion_active = 0;
	arm->target_mode = ARM_TARGET_CARTESIAN;
	arm->big_arm_overshoot_rad = 0.0f;
	arm->big_arm_final_joint = 0.0f;
	arm->big_arm_overshoot_joint = 0.0f;
	arm->big_arm_overshoot_armed = 0;
	arm->big_arm_overshoot_phase = 0;

	g_joint_cmd_inited[0] = 0;
	g_joint_cmd_inited[1] = 0;
	g_joint_cmd_inited[2] = 0;
	g_big_arm_plan_inited = 0;
	g_big_arm_start = DEFAULT_ANGLE_1;
	g_big_arm_target = DEFAULT_ANGLE_1;
	g_joint_cmd_prev[0] = DEFAULT_ANGLE_1;
	g_joint_cmd_prev[1] = DEFAULT_ANGLE_2;
	g_joint_cmd_prev[2] = DEFAULT_ANGLE_3;

	g_joint_cmd_filter_inited[0] = 0;
	g_joint_cmd_filter_inited[1] = 0;
	g_joint_cmd_filter_inited[2] = 0;
	g_joint_cmd_filtered[0] = DEFAULT_ANGLE_1;
	g_joint_cmd_filtered[1] = DEFAULT_ANGLE_2;
	g_joint_cmd_filtered[2] = DEFAULT_ANGLE_3;
	g_joint_speed_filtered[0] = 0.0f;
	g_joint_speed_filtered[1] = 0.0f;
	g_joint_speed_filtered[2] = 0.0f;
	g_big_arm_ff = 0.0f;

	g_overshoot_stable_cnt = 0;
	g_last_target_quadrant = 0;
	g_flip_transition_dir = 0;
	g_prev_status = ARM_STATE_INIT;
	g_suction_wait_latched = 0;
	g_suction_wait_locked_pos = 0.0f;
	g_suction_wait_locked_inited = 0;
	g_last_status = ARM_STATE_INIT;
	g_arm_ctrl_dt = ARM_CTRL_DT_DEFAULT;

	arm->status = ARM_STATE_INIT;
}

/**
 * @brief 设置控制周期(秒)。
 */
void robot_arm_set_ctrl_dt(float dt_s) {
	if (dt_s < 0.001f) {
		dt_s = 0.001f;
	} else if (dt_s > 0.2f) {
		dt_s = 0.2f;
	}

	g_arm_ctrl_dt = dt_s;
}

/**
 * @brief 设置机械臂末端目标位置，并按需武装大臂过冲。
 */
void robot_arm_set_target(RobotArm *arm, float y, float z, float pitch) {
	arm->arm_target_y = y;
	arm->arm_target_z = z;
	arm->arm_target_pitch = pitch;
	arm->target_mode = ARM_TARGET_CARTESIAN;

	/* 关闭过冲逻辑：大臂始终直接跟踪最终目标。 */
	g_last_target_quadrant = arm_get_ik_quadrant(y, z, pitch);
	g_flip_transition_dir = 0;
	arm->big_arm_overshoot_armed = 0;
	arm->big_arm_overshoot_phase = 0;
}

/**
 * @brief 设置机械臂关节角目标（调试接口）。
 */
void robot_arm_set_joint_target(RobotArm *arm, float joint1, float joint2,
									float joint3) {
	arm->arm_joint_target[0] = joint1;
	arm->arm_joint_target[1] = joint2;
	arm->arm_joint_target[2] = joint3;
	arm->target_mode = ARM_TARGET_JOINT;
	arm->big_arm_overshoot_armed = 0;
	arm->big_arm_overshoot_phase = 0;
}

/**
 * @brief 设置大臂过冲量。
 */
void robot_arm_set_big_arm_overshoot(RobotArm *arm, float overshoot_rad) {
	if (overshoot_rad <= 0.0f) {
		arm->big_arm_overshoot_rad = 0.0f;
		arm->big_arm_overshoot_armed = 0;
		arm->big_arm_overshoot_phase = 0;
		return;
	}

	arm->big_arm_overshoot_rad = overshoot_rad;
}

/**
 * @brief 机械臂周期更新入口。
 */
void robot_arm_update(RobotArm *arm) {
	float joint_target[3];
	float joint_cmd[3];

	if (!g_joint_cmd_inited[0]) {
		g_joint_cmd_inited[0] = 1;
		g_joint_cmd_prev[0] = arm->damiao_1.position;
		g_joint_cmd_prev[1] = arm->damiao_3.position;
		g_joint_cmd_prev[2] = arm->damiao_4.position;
		g_big_arm_plan_inited = 0;
	}

	arm_update_transition_gate(arm);

	if (arm->target_mode == ARM_TARGET_JOINT) {
		joint_target[0] = arm->arm_joint_target[0];
		joint_target[1] = arm->arm_joint_target[1];
		joint_target[2] = arm->arm_joint_target[2];
	} else {
		arm_pos_angle(arm->arm_target_y, arm->arm_target_z, arm->arm_target_pitch,
					  joint_target);
		/* 过冲状态机已关闭，直接使用逆解终点。 */
	}

	arm_big_arm_step(joint_target[0], &joint_cmd[0]);
	joint_cmd[1] = joint_target[1];
	joint_cmd[2] = joint_target[2];
	g_joint_cmd_prev[1] = joint_cmd[1];
	g_joint_cmd_prev[2] = joint_cmd[2];

	arm_apply_ctrl(arm, joint_cmd);

	arm->arm_motion_active =
		(fabsf(arm->damiao_1.position - joint_cmd[0]) > 0.02f) ||
		(fabsf(arm->damiao_3.position - joint_cmd[1]) > 0.02f) ||
		(fabsf(arm->damiao_4.position - joint_cmd[2]) > 0.02f);
}

/**
 * @brief 下发三关节控制。
 */
void arm_apply_ctrl(RobotArm *arm, const float joint_des[3]) {
	float joint0_cmd = joint_des[0];                 /* 大臂目标角。 */
	float joint0_err = joint_des[0] - arm->damiao_1.position;  /* 大臂当前位置误差。 */
	float err_abs = fabsf(joint0_err);               /* 大臂误差绝对值，用于分段限速。 */
	float cmd_speed;
	float ff_abs;                                    /* 大臂前馈速度幅值。 */
	float max_step;
	float delta;
	float joint1_cmd;                                /* 小臂目标角。 */
	float joint2_cmd;                                /* 吸盘目标角。 */
	float joint1_step;
	float joint2_step;
	float joint1_err;
	float joint2_err;
	float joint1_speed;
	float joint2_speed;
	float suction_speed_max;                         /* 吸盘速度上限，会随阶段切换。 */
	float small_cmd_rate_limit;                      /* 小臂命令变化率上限。 */
	float small_speed_base;                          /* 小臂基础速度。 */
	float small_speed_gain;                          /* 小臂误差到速度的增益。 */
	float small_speed_max;                           /* 小臂速度上限。 */
	float small_cmd_alpha;                           /* 小臂命令平滑系数。 */
	float small_speed_filter_alpha;                  /* 小臂速度滤波系数。 */
	float place_speed_min;                           /* 放置态吸盘最小速度，近目标时会降低。 */

	if (!g_joint_cmd_filter_inited[0]) {
		g_joint_cmd_filter_inited[0] = 1;
		g_joint_cmd_filtered[0] = arm->damiao_1.position;
	}
	if (!g_joint_cmd_filter_inited[1]) {
		g_joint_cmd_filter_inited[1] = 1;
		g_joint_cmd_filtered[1] = arm->damiao_3.position;
		g_joint_speed_filtered[1] = 0.0f;
	}
	if (!g_joint_cmd_filter_inited[2]) {
		g_joint_cmd_filter_inited[2] = 1;
		g_joint_cmd_filtered[2] = arm->damiao_4.position;
		g_joint_speed_filtered[2] = 0.0f;
	}
	suction_speed_max = ARM_SUCTION_SPEED_MAX;
	if (g_suction_wait_latched && (arm->status != ARM_STATE_PLACE)) {
		g_suction_wait_latched = 0;
		g_suction_wait_locked_inited = 0;  /* 退出等待状态时重置锁定标志。 */
	}

	if (arm->status == ARM_STATE_CATCH) {
		small_cmd_alpha = ARM_SMALL_CATCH_CMD_ALPHA;
		small_cmd_rate_limit = ARM_SMALL_CATCH_CMD_RATE_LIMIT;
		small_speed_base = ARM_SMALL_CATCH_SPEED_BASE;
		small_speed_gain = ARM_SMALL_CATCH_SPEED_GAIN;
		small_speed_max = ARM_SMALL_CATCH_SPEED_MAX;
		small_speed_filter_alpha = ARM_SMALL_CATCH_SPEED_FILTER_ALPHA;
	} else {
		small_cmd_alpha = ARM_SMALL_CMD_ALPHA;
		small_cmd_rate_limit = ARM_SMALL_CMD_RATE_LIMIT;
		small_speed_base = ARM_SMALL_SPEED_BASE;
		small_speed_gain = ARM_SMALL_SPEED_GAIN;
		small_speed_max = ARM_SMALL_SPEED_MAX;
		small_speed_filter_alpha = ARM_SMALL_SPEED_FILTER_ALPHA;
	}

	/* 大臂：保留轨迹前馈 + 平滑限速 */
	max_step = ARM_J8006_CMD_RATE_LIMIT * g_arm_ctrl_dt;
	delta = ARM_J8006_FILTER_ALPHA * (joint0_cmd - g_joint_cmd_filtered[0]);
	delta = arm_clampf(delta, -max_step, max_step);
	g_joint_cmd_filtered[0] += delta;

	if (fabsf(joint0_err) < ARM_J8006_HOLD_ERR_RAD) {
		g_joint_cmd_filtered[0] = arm->damiao_1.position;
	}

	ff_abs = fabsf(g_big_arm_ff);
	g_joint_speed_filtered[0] = 0.6f * ff_abs + 0.4f * g_joint_speed_filtered[0];

	cmd_speed = ARM_J8006_CMD_SPEED_BASE + ARM_J8006_CMD_SPEED_FF_GAIN * g_joint_speed_filtered[0];

	if (err_abs > ARM_J8006_NEAR_ERR_RAD) {
		cmd_speed = fminf(cmd_speed, ARM_J8006_CMD_SPEED_MAX);
	} else if (err_abs > ARM_J8006_FINE_ERR_RAD) {
		float blend = (err_abs - ARM_J8006_FINE_ERR_RAD) /
					  (ARM_J8006_NEAR_ERR_RAD - ARM_J8006_FINE_ERR_RAD);
		float target_speed = ARM_J8006_FINE_SPEED_MAX + blend *
					 (ARM_J8006_NEAR_SPEED_MAX - ARM_J8006_FINE_SPEED_MAX);
		cmd_speed = fminf(cmd_speed, target_speed);
	} else {
		cmd_speed = fminf(cmd_speed, ARM_J8006_FINE_SPEED_MAX);
	}

	if ((joint0_err * arm->damiao_1.speed) < 0.0f) {
		cmd_speed = fminf(cmd_speed, ARM_J8006_FINE_SPEED_MAX);
	}

	if ((arm->big_arm_overshoot_phase == 2) && (err_abs < 0.20f)) {
		float phase2_limit = ARM_J8006_NEAR_SPEED_MAX;
		if (g_flip_transition_dir == 1) {
			phase2_limit = ARM_J8006_PHASE2_Q2TOQ1_SPEED_MAX;
		}
		cmd_speed = fminf(cmd_speed, phase2_limit);
	}

	cmd_speed = arm_clampf(cmd_speed, 0.06f, ARM_J8006_CMD_SPEED_MAX);

	/* 小臂：不做轨迹规划，只做命令平滑 */
	joint1_cmd = joint_des[1];
	joint1_step = small_cmd_alpha * (joint1_cmd - g_joint_cmd_filtered[1]);
	joint1_step = arm_clampf(joint1_step, -small_cmd_rate_limit * g_arm_ctrl_dt,
					 small_cmd_rate_limit * g_arm_ctrl_dt);
	g_joint_cmd_filtered[1] += joint1_step;

	joint1_err = g_joint_cmd_filtered[1] - arm->damiao_3.position;
	joint1_speed = small_speed_base + small_speed_gain * fabsf(joint1_err);
	g_joint_speed_filtered[1] = small_speed_filter_alpha * joint1_speed +
		(1.0f - small_speed_filter_alpha) * g_joint_speed_filtered[1];
	g_joint_speed_filtered[1] = arm_clampf(g_joint_speed_filtered[1], 0.04f, small_speed_max);

	/* 吸盘：仅在抓取态->放置态时做慢速等待，其余状态直接快速下发。 */
	joint2_cmd = joint_des[2];
	if (arm->status == ARM_STATE_PLACE) {
		joint2_cmd -= ARM_PLACE_SUCTION_OFFSET_RAD;
	}

	if (g_suction_wait_latched) {
		/* 放置态先让小臂到位，再放行吸盘，避免先后顺序错乱。 */
		if (fabsf(arm->damiao_3.position - joint_des[1]) <= ARM_PLACE_SMALL_ARM_READY_ERR_RAD) {
			g_suction_wait_latched = 0;
			g_suction_wait_locked_inited = 0;  /* 退出等待时重置锁定标志。 */
		}
	}

	if (g_suction_wait_latched) {
		/* 第一次进入等待时，记录当前的吸盘目标位置，后续保持不变。 */
		if (!g_suction_wait_locked_inited) {
			g_suction_wait_locked_pos = joint_des[2];
			g_suction_wait_locked_inited = 1;
		}
		/* 使用锁定的目标位置，避免每个控制周期都追踪当前位置导致不稳定。 */
		g_joint_cmd_filtered[2] = g_suction_wait_locked_pos;
		g_joint_speed_filtered[2] = ARM_SUCTION_WAIT_HOLD_SPEED;
	} else if (arm->status == ARM_STATE_PLACE) {
		joint2_step = ARM_SUCTION_CMD_ALPHA * (joint2_cmd - g_joint_cmd_filtered[2]);
		joint2_step = arm_clampf(joint2_step, -ARM_SUCTION_CMD_RATE_LIMIT * g_arm_ctrl_dt,
							 ARM_SUCTION_CMD_RATE_LIMIT * g_arm_ctrl_dt);
		g_joint_cmd_filtered[2] += joint2_step;

		joint2_err = g_joint_cmd_filtered[2] - arm->damiao_4.position;
		joint2_speed = ARM_SUCTION_PLACE_SPEED_BASE +
					   ARM_SUCTION_PLACE_SPEED_GAIN * fabsf(joint2_err);
		suction_speed_max = ARM_SUCTION_PLACE_SPEED_MAX;
		g_joint_speed_filtered[2] = ARM_SUCTION_SPEED_FILTER_ALPHA * joint2_speed +
			(1.0f - ARM_SUCTION_SPEED_FILTER_ALPHA) * g_joint_speed_filtered[2];
		place_speed_min = ARM_SUCTION_PLACE_SPEED_MIN;
		if (fabsf(joint2_err) < ARM_SUCTION_PLACE_FINE_ERR_RAD) {
			place_speed_min = ARM_SUCTION_PLACE_FINE_SPEED_MIN;
		}
		if ((g_flip_transition_dir != 0) || (arm->big_arm_overshoot_phase != 0)) {
			g_joint_speed_filtered[2] = arm_clampf(g_joint_speed_filtered[2], place_speed_min, ARM_SUCTION_FLIP_SLOW_SPEED_MAX);
		}
		g_joint_speed_filtered[2] = arm_clampf(g_joint_speed_filtered[2], place_speed_min, suction_speed_max);
		if ((joint2_err * arm->damiao_4.speed) < 0.0f) {
			g_joint_speed_filtered[2] = fminf(g_joint_speed_filtered[2], ARM_SUCTION_PLACE_BRAKE_SPEED_MAX);
		}
	} else {
		g_joint_cmd_filtered[2] = joint2_cmd;
		g_joint_speed_filtered[2] = ARM_SUCTION_SPEED_MAX;
	}

	dm_pos_speed_ctrl(&arm->damiao_1, g_joint_cmd_filtered[0], cmd_speed);
	dm_pos_speed_ctrl(&arm->damiao_2, -g_joint_cmd_filtered[0], cmd_speed);
	dm_pos_speed_ctrl(&arm->damiao_3, g_joint_cmd_filtered[1], g_joint_speed_filtered[1]);
	dm_pos_speed_ctrl(&arm->damiao_4, g_joint_cmd_filtered[2], g_joint_speed_filtered[2]);
}

/**
 * @brief 输入指定的末端位置和吸盘姿态，计算 3 个电机的关节角度。
 */
void arm_pos_angle(float x1, float z1, float pitch_angle, float angle[3]) {
	float x;                 /* 末端输入坐标 x。 */
	float z;                 /* 末端输入坐标 z。 */
	float x_w;               /* 腕点坐标 x。 */
	float z_w;               /* 腕点坐标 z。 */
	float m_2;               /* 腕点到肩关节的距离平方。 */
	float m;                 /* 腕点到肩关节的距离。 */
	float a;
	float a2;
	float b;
	float b2;
	float angle1_1;
	float angle1_2;
	float angle2_inner;

	x = x1 + DEFAULT_X;
	z = z1 + DEFAULT_Z;

	x_w = x - ARM_3 * cosf(pitch_angle) - (DEFAULT_ARM3_X - DEFAULT_ARM_1_2) * cosf(pitch_angle);
	z_w = z - ARM_3 * sinf(pitch_angle);
	m_2 = x_w * x_w + z_w * z_w;
	arm_sqrt_f32(m_2, &m);

	b = (ARM_1 * ARM_1 + m_2 - ARM_2 * ARM_2) / (2 * ARM_1 * m);
	arm_sqrt_f32(1 - b * b, &b2);
	arm_atan2_f32(b2, b, &angle1_1);
	arm_atan2_f32(z_w, x_w, &angle1_2);
	angle[0] = DEFAULT_ANGLE_1 + PI / 2 - angle1_1 - angle1_2;

	a = (ARM_1 * ARM_1 + ARM_2 * ARM_2 - m_2) / (2 * ARM_1 * ARM_2);
	arm_sqrt_f32(1 - a * a, &a2);
	arm_atan2_f32(a2, a, &angle2_inner);
	angle[1] = angle2_inner - DEFAULT_ANGLE_2;

	if (x_w >= 0) {
		angle[0] = DEFAULT_ANGLE_1 + PI / 2 - angle1_1 - angle1_2;
		angle[1] = angle2_inner - DEFAULT_ANGLE_2;
	} else {
		angle[0] = DEFAULT_ANGLE_1 + PI / 2 + angle1_1 - angle1_2;
		angle[1] = 2 * PI + (DEFAULT_ANGLE_2 - angle2_inner);
	}

	angle[2] = -pitch_angle - angle[0] + angle[1] + DEFAULT_ANGLE_3;
	if (angle[2] < -0.1f) {
		angle[2] += 2 * PI;
	}
	if (angle[2] > 2 * PI) {
		angle[2] -= 2 * PI;
	}
}
