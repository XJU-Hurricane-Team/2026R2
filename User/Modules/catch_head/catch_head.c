/**
 * @file catch_head.c
 * @author xinglu
 * @brief 矛头夹取模块
 * @version 1.3
 * @date 2026-04-15
 */

#include "catch_head.h"

/* 旋转矛头关节管理 */
static dji_motor_handle_t g_catch_rod_motor;
static catch_head_dji_joint_t g_catch_rod_joint = {
	.motor_handle = &g_catch_rod_motor,
	.reduction_ratio = 36.0f,
	.target_angle_deg = {0.0f, 90.0f},
	.target_index = 0,
};

/* 夹取机构的 DM 执行电机及目标位管理 */
static dm_handle_t g_dm_motor;
static catch_head_dm_joint_t g_dm_joint = {
	.motor_handle = &g_dm_motor,
	.target_position_rad = {1.57f, 0.0f , -0.5f},
	.target_index = 0,
};

/* 夹爪开合舵机及目标位管理 */
static servo_t g_gripper_servo;
static catch_head_servo_joint_t g_gripper_joint = {
	.servo_handle = &g_gripper_servo,
	.target_index = 0,
};

/* 模块就绪标志，避免初始化失败后继续执行控制 */
static uint8_t g_ready = 0;

static void catch_head_apply_default_targets(void);

/**
 * @brief 初始化夹爪舵机
 * @note 使用 TIM1 CH1 作为舵机输出通道，并配置上下限脉宽
 */
void catch_head_servo_init(void) {
	servo_init(&g_gripper_servo, &htim1, TIM_CHANNEL_1, 3000, 3850);
}

/**
 * @brief 初始化夹取相关电机及控制参数
 * @note 包括 DJI 电机、DM 电机、PID 参数和默认目标状态
 */
void catch_head_motor_init(void) {
	if (dji_motor_init(&g_catch_rod_motor, DJI_M2006, CAN_Motor1_ID,
					   can2_selected) != 0) {
		g_ready = 0;
		return;
	}

	if (dm_motor_init(&g_dm_motor, 0x15, 0x05, DM_MODE_POS_SPEED, DM_J4310, 3.14f, 45.0f,
                  20.0f, can2_selected) != 0) {
		g_ready = 0;
		return;
	}

	pid_init(&g_catch_rod_joint.speed_pid, 16384, 500, 0, 16384, DELTA_PID, 8, 0.12f, 0.0f);
	pid_init(&g_catch_rod_joint.angle_pid, 16384, 1000, 0.2f, 16384, POSITION_PID, 1.0f, 0.0f, 0.0f);
	dm_motor_enable(&g_dm_motor);

	g_catch_rod_joint.target_index = 0;
	g_dm_joint.target_index = 0;
	g_gripper_joint.target_index = 0;

	catch_head_apply_default_targets();
	g_ready = 1;
}

/**
 * @brief 夹取模块总初始化入口
 * @note 先清除就绪标志，再初始化舵机和电机
 */
void catch_head_init(void) {
	g_ready = 0;

	catch_head_servo_init();
	catch_head_motor_init();
}

/**
 * @brief 夹取模块周期更新
 * @note DJI 电机采用轨迹规划 + 角度/速度双环控制，DM 电机采用位置速度控制
 */
void catch_head(void) {
	if (g_ready == 0) {
		return;
	}

	float p_des_rad = 0.0f;
	float w_des_rad_s = 0.0f;

	t_trajectory_update(&g_catch_rod_joint.trajectory, &p_des_rad,
					   &w_des_rad_s);

	float target_out_angle = p_des_rad * CATCH_HEAD_RAD_TO_DEG;
	float target_out_ff_rpm = w_des_rad_s * CATCH_HEAD_RAD_S_TO_RPM;

	float add_rpm = pid_calc(
		&g_catch_rod_joint.angle_pid, target_out_angle,
		g_catch_rod_joint.motor_handle->rotor_degree);

	float final_rpm = (target_out_ff_rpm + add_rpm) *
					  g_catch_rod_joint.reduction_ratio;
	float set_current = pid_calc(
		&g_catch_rod_joint.speed_pid, final_rpm,
		g_catch_rod_joint.motor_handle->speed_rpm);

	g_catch_rod_joint.motor_handle->set_value = (int16_t)set_current;

	dji_motor_set_current(can2_selected, DJI_MOTOR_GROUP1,
					  g_catch_rod_joint.motor_handle->set_value, 0, 0, 0);

	float dm_target_rad = g_dm_joint.target_position_rad[g_dm_joint.target_index];
	dm_pos_speed_ctrl(g_dm_joint.motor_handle, dm_target_rad, DM_SPEED);
}

static void catch_head_apply_default_targets(void) {
	catch_head_set_dji_target(CATCH_HEAD_DJI_TARGET_HOME);
	catch_head_set_dm_target(CATCH_HEAD_DM_TARGET_RETRACT);
	catch_head_set_servo_target(CATCH_HEAD_SERVO_TARGET_CLOSE);
}

/**
 * @brief 设置 DJI 电机目标角度索引
 * @note 会重新初始化轨迹，使当前角度平滑过渡到目标角度
 * @note target_index == 0 时为 0 度，非 0 时为 90 度
 */
void catch_head_set_dji_target(uint8_t target_index) {
	if (g_catch_rod_joint.motor_handle == NULL) {
		return;
	}

	// target_index %= CATCH_HEAD_DJI_TARGET_COUNT;
	if (target_index >= CATCH_HEAD_DJI_TARGET_COUNT) {
    return;
}

	g_catch_rod_joint.target_index = target_index;
	t_trajectory_init(&g_catch_rod_joint.trajectory,
					 g_catch_rod_joint.motor_handle->rotor_degree * CATCH_HEAD_DEG_TO_RAD,
					 g_catch_rod_joint.target_angle_deg[target_index] * CATCH_HEAD_DEG_TO_RAD,
					 5.0f, 10.0f, 0.005f);
}

/**
 * @brief 设置 DM 电机目标位置索引
 * @note target_index == 0 时达妙位置为 0 rad，非 0 时为 1.57 rad
 */
void catch_head_set_dm_target(uint8_t target_index) {
	if (g_dm_joint.motor_handle == NULL) {
		return;
	}

	if (target_index >= CATCH_HEAD_DM_TARGET_COUNT) {
		return;
	}

	// target_index %= CATCH_HEAD_DM_TARGET_COUNT;
	g_dm_joint.target_index = target_index;
}

/**
 * @brief 设置舵机开合状态
 * @note target_index == 0 时关闭夹爪，非 0 时打开夹爪
 */
void catch_head_set_servo_target(uint8_t target_index) {
	if (g_gripper_joint.servo_handle == NULL || g_gripper_joint.servo_handle->htim == NULL) {
		return;
	}

	if (target_index >= CATCH_HEAD_SERVO_TARGET_COUNT) {
		return;
	}
	// target_index %= CATCH_HEAD_SERVO_TARGET_COUNT;
	g_gripper_joint.target_index = target_index;

	servo_set_state(g_gripper_joint.servo_handle,
				   target_index == 0 ? SERVO_CLOSE : SERVO_OPEN);
}
