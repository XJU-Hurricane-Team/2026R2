/**
 * @file catch_head.c
 * @author xinglu
 * @brief ??????
 * @version 1.1
 * @date 2026-04-11
 */

#include "catch_head.h"

/* ?????????????? */
static dji_motor_handle_t s_catch_rod_motor;
static catch_head_dji_joint_t s_catch_rod_joint = {
	.motor_handle = &s_catch_rod_motor,
	.reduction_ratio = 36.0f,
	.target_angle_deg = {0.0f, 90.0f},
	.target_index = 0,
};

/* ????? DM ?????????? */
static dm_handle_t s_dm_motor;
static catch_head_dm_joint_t s_dm_joint = {
	.motor_handle = &s_dm_motor,
	.target_position_deg = {0.0f, 90.0f},
	.target_index = 0,
};

/* ???????????? */
static servo_t s_gripper_servo;
static catch_head_servo_joint_t s_gripper_joint = {
	.servo_handle = &s_gripper_servo,
	.target_index = 0,
};

/* ????????????????????? */
static uint8_t s_ready = 0;

/**
 * @brief ???????
 * @note ?? TIM1 CH1 ?????????????????
 */
void catch_head_servo_init(void) {
	servo_init(&s_gripper_servo, &htim1, TIM_CHANNEL_1, 1400, 1908);
}

/**
 * @brief ??????????????
 * @note ?? DJI ???DM ???PID ?????????
 */
void catch_head_motor_init(void) {
	if (dji_motor_init(&s_catch_rod_motor, DJI_M2006, CAN_Motor1_ID,
					   can1_selected) != 0) {
		s_ready = 0;
		return;
	}

	if (dm_motor_init(&s_dm_motor, 0x15,
					  0x05, DM_MODE_POS_SPEED, DM_J4310,
					  12.5f, 10.0f,
					  45.0f, can1_selected) != 0) {
		s_ready = 0;
		return;
	}

	pid_init(&s_catch_rod_joint.speed_pid, 16384, 500, 0, 16384, DELTA_PID, 8, 0.12f, 0.0f);
	pid_init(&s_catch_rod_joint.angle_pid, 16384, 1000, 0.2f, 16384, POSITION_PID, 1.0f, 0.0f, 0.0f);
	dm_motor_enable(&s_dm_motor);

	s_catch_rod_joint.target_index = 0;
	s_dm_joint.target_index = 0;
	s_gripper_joint.target_index = 0;


	catch_head_set_dji_target(0);
	catch_head_set_servo_target(0);
	s_ready = 1;
}

/**
 * @brief ??????????
 * @note ?????????????????
 */
void catch_head_init(void) {
	s_ready = 0;

	catch_head_servo_init();
	catch_head_motor_init();
}

/**
 * @brief ????????
 * @note DJI ???????? + ??/???????DM ??????????
 */
void catch_head(void) {
	if (s_ready == 0) {
		return;
	}

	float p_des_rad = 0.0f;
	float w_des_rad_s = 0.0f;

	t_trajectory_update(&s_catch_rod_joint.trajectory, &p_des_rad,
					   &w_des_rad_s);

	float target_out_angle = p_des_rad * CATCH_HEAD_RAD_TO_DEG;
	float target_out_ff_rpm = w_des_rad_s * CATCH_HEAD_RAD_S_TO_RPM;

	float add_rpm = pid_calc(
		&s_catch_rod_joint.angle_pid, target_out_angle,
		s_catch_rod_joint.motor_handle->rotor_degree);

	float final_rpm = (target_out_ff_rpm + add_rpm) *
					  s_catch_rod_joint.reduction_ratio;
	float set_current = pid_calc(
		&s_catch_rod_joint.speed_pid, final_rpm,
		s_catch_rod_joint.motor_handle->speed_rpm);

	s_catch_rod_joint.motor_handle->set_value = (int16_t)set_current;

	dji_motor_set_current(can1_selected, DJI_MOTOR_GROUP1,
						  s_catch_rod_joint.motor_handle->set_value, 0, 0, 0);

	float dm_target_rad = s_dm_joint.target_position_deg[s_dm_joint.target_index];
	dm_pos_speed_ctrl(s_dm_joint.motor_handle, dm_target_rad, DM_SPEED);
}

/**
 * @brief ???????????
 * @note KEY0/1/2 ???? DJI ???DM ??????????
 */
void catch_head_on_key(key_press_t key) {
	if (s_ready == 0) {
		return;
	}

	switch (key) {
		case KEY0_PRESS: {
			catch_head_set_dji_target((uint8_t)((s_catch_rod_joint.target_index + 1) % CATCH_HEAD_DJI_TARGET_COUNT));
		} break;

		case KEY1_PRESS: {
			catch_head_set_dm_target((uint8_t)((s_dm_joint.target_index + 1) % CATCH_HEAD_DM_TARGET_COUNT));
		} break;

		case KEY2_PRESS: {
			catch_head_set_servo_target((uint8_t)((s_gripper_joint.target_index + 1) % CATCH_HEAD_SERVO_TARGET_COUNT));
		} break;

		default: {
		} break;
	}
}

/**
 * @brief ?? DJI ????????
 * @note ???????????????????????
 */
void catch_head_set_dji_target(uint8_t target_index) {
	if (s_catch_rod_joint.motor_handle == NULL) {
		return;
	}

	target_index %= CATCH_HEAD_DJI_TARGET_COUNT;
	s_catch_rod_joint.target_index = target_index;
	t_trajectory_init(&s_catch_rod_joint.trajectory,
					 s_catch_rod_joint.motor_handle->rotor_degree * CATCH_HEAD_DEG_TO_RAD,
					 s_catch_rod_joint.target_angle_deg[target_index] * CATCH_HEAD_DEG_TO_RAD,
					 5.0f, 10.0f, 0.005f);
}

/**
 * @brief ?? DM ????????
 */
void catch_head_set_dm_target(uint8_t target_index) {
	if (s_dm_joint.motor_handle == NULL) {
		return;
	}

	target_index %= CATCH_HEAD_DM_TARGET_COUNT;
	s_dm_joint.target_index = target_index;
}

/**
 * @brief ????????
 * @note target_index == 0 ??????? 0 ?????
 */
void catch_head_set_servo_target(uint8_t target_index) {
	if (s_gripper_joint.servo_handle == NULL || s_gripper_joint.servo_handle->htim == NULL) {
		return;
	}

	target_index %= CATCH_HEAD_SERVO_TARGET_COUNT;
	s_gripper_joint.target_index = target_index;

	servo_set_state(s_gripper_joint.servo_handle,
				   target_index == 0 ? SERVO_CLOSE : SERVO_OPEN);
}
