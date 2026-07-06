/**
 * @file catch_head.c
 * @author xinglu
 * @brief 矛头夹取模块
 * @version 1.3
 * @date 2026-04-15
 */

#include "catch_head.h"
#include "logger/logger.h"  

/* 夹取机构的 DM 执行电机及目标位管理 */
static dm_handle_t g_dm_motor;
static catch_head_dm_joint_t g_dm_joint = {
	.motor_handle = &g_dm_motor,
	.target_position_rad = {0.00f, 1.52f , 3.10f},
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
	servo_init(&g_gripper_servo, &htim4, TIM_CHANNEL_2, 2500, 1600);
}

/**
 * @brief 初始化夹取相关电机及控制参数
 * @note 包括 DJI 电机、DM 电机、PID 参数和默认目标状态
 */
void catch_head_motor_init(void) {

	int res = dm_motor_init(&g_dm_motor, 0x15, 0x05, DM_MODE_POS_SPEED, DM_J4310, 12.5f, 45.0f,
                  20.0f, can2_selected);
	if (res != 0) {
		g_ready = 0;
		log_message(LOG_ERROR, "catch_head_motor_init: DM motor init failed");
	}

	dm_motor_enable(&g_dm_motor);

	g_dm_joint.target_index = 0;
	g_gripper_joint.target_index = 0;

	// catch_head_apply_default_targets();
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

	float dm_target_rad = g_dm_joint.target_position_rad[g_dm_joint.target_index];
	dm_pos_speed_ctrl(g_dm_joint.motor_handle, dm_target_rad, DM_SPEED);
}

static void catch_head_apply_default_targets(void) {
	catch_head_set_dm_target(CATCH_HEAD_DM_TARGET_RETRACT);
	catch_head_set_servo_target(CATCH_HEAD_SERVO_TARGET_CLOSE);
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

/**
 * @brief 检测动作是否成功
 * 
 * @return true 
 * @return false 
 */
bool catch_head_is_target_reached(void) {
	if (g_ready == 0 ||
		g_dm_joint.motor_handle == NULL) {
		return false;
	}


	float dm_target_rad = g_dm_joint.target_position_rad[g_dm_joint.target_index];
	float dm_err_rad = fabsf(g_dm_joint.motor_handle->position - dm_target_rad);

	uint8_t dm_ok = (uint8_t)(dm_err_rad <= CATCH_HEAD_DM_POS_TOL_RAD);
	uint8_t servo_ok = 1; 
	if(dm_ok && servo_ok)
	LED3_TOGGLE();
	return (bool)(dm_ok && servo_ok);
}
