/**
 * @file arm_ctrl.c
 * @author xinglu
 * @brief 机械臂控制 (应用层任务与点位逻辑实现)
 * @version 2.0
 * @date 2026-04-30
 */

#include "includes.h"
#include "arm_ctrl.h"
#include "microros_ctrl.h"
#include "adc.h"

/* ---------------- 函数前置声明 ---------------- */
void robot_arm_task(void *pvParameters);
static float arm_ctrl_wrap_pi(float angle);
static void robot_arm_mark_reach_target(float y, float z, float pitch);
static void robot_arm_check_target_reached(void);
static void robot_arm_wait_takeout_seq_init(float y, float z, float pitch);
static void robot_arm_wait_takeout_seq_update(void);
static void pump_check_ready(void);
static void pump_set_state(uint8_t on);
static uint8_t pump_read_adc(uint16_t *out_value);

#if ARM_USE_REMOTE_KEY
static void arm_remote_state_switch(uint8_t key, remote_key_event_t event);
#endif

/* ---------------- 全局与静态变量定义 ---------------- */
static RobotArm g_robot_arm;                
static uint8_t g_arm_target_index;          
static TaskHandle_t g_robot_arm_task_handle;
static uint8_t g_last_target_index;         

static uint8_t g_place_target_index = 0;    
static uint8_t g_wait_takeout_target_index = 2; 
static arm_target_point_t g_dynamic_target = {0}; 
static uint8_t g_has_dynamic_target = 0; 

static wait_takeout_seq_t g_wait_takeout_seq = WAIT_TAKEOUT_SEQ_NONE;
static float g_wait_takeout_final_y = 0.0f;
static float g_wait_takeout_final_z = 0.0f;
static float g_wait_takeout_final_pitch = 0.0f;
static float g_wait_takeout_pre_big_arm = 0.0f;
static float g_wait_takeout_pre_small_arm = 0.0f;

static float g_arm_reach_target_joint[2] = {0.0f, 0.0f};  // 目标关节角度 [j1, j3]
static uint8_t g_arm_reach_pending = 0; 
static pump_wait_state_t g_pump_wait_state = PUMP_WAIT_NONE;
static uint8_t g_last_switch_key = 0xFF;

/* ---------------- 预设点位数组 ---------------- */

/**
 * @brief 全局预设点位数组 (注: 4已修正为WAIT_TAKEOUT, 5为TAKEOUT)
 */
static const arm_target_point_t g_arm_target_points[6] = {
	{139.95f + 20.0f, 102.70f, 0.8955f},  // 0: INIT
	{200.000f, 10.0f, 0.0f},              // 1: READY
	// {533.142f, 91.5027f, 0.0349f},        // 2: CATCH
	{300.000f, 165.000f,0.0349f},
	{-275.12f, 493.991f, -PI/2.0},        // 3: PLACE
	{533.142f, 300.0f, 0.0f},             // 4: WAIT_TAKEOUT 
	{533.142f, 300.0f, 0.0f},             // 5: TAKEOUT 
};

static const arm_target_point_t g_arm_place_points[3] = {
	{-280.0f, 380.0f, -PI/2},                      // 0: 底层
	{-265.12f + 175.0f, FIRST_POINT_Z_LOW + 175.0f, -PI},   // 1: 中层
	{-265.12f + 175.0f + 30.0f, FIRST_POINT_Z_LOW + 175.0f + 350.0f, -PI},  // 2: 顶层
};

static const arm_target_point_t g_arm_wait_takeout_points[3] = {
	{-330.0f, FIRST_POINT_Z_LOW, -PI/2},                      // 0: 底层
	{-265.12f + 165.0f, FIRST_POINT_Z_LOW + 175.0f, -PI},   // 1: 中层
	{-265.12f + 165.0f + 20.0f, FIRST_POINT_Z_LOW + 175.0f + 350.0f, -PI},  // 2: 顶层
};

/* ---------------- 应用层实现 ---------------- */

static arm_status_t arm_status_from_index(uint8_t index) {
	switch (index) {
	case 0: return ARM_STATE_INIT;
	case 1: return ARM_STATE_READY;
	case 2: return ARM_STATE_CATCH;
	case 3: return ARM_STATE_PLACE;
	case 4: return ARM_STATE_WAIT_TAKEOUT; // 已同步修改
	case 5: return ARM_STATE_TAKEOUT;      // 已同步修改
	default: return ARM_STATE_INIT;
	}
}

void robot_arm_set_dynamic_catch_target(float x, float y, float z) {
	g_dynamic_target.y = y * 1000.0f + 200.0f - 55.0f; 
	g_dynamic_target.z = z * 1000.0f + 10.0f + 90.0f; 
	g_has_dynamic_target = 1; 
	robot_arm_set_state_index((uint8_t)ARM_STATE_CATCH);
}

void robot_arm_set_place_index(uint8_t place_idx) {
	if (place_idx < 3) g_place_target_index = place_idx;
}

void robot_arm_set_wait_takeout_index(uint8_t takeout_idx) {
	if (takeout_idx < 3) g_wait_takeout_target_index = takeout_idx;
}

void robot_arm_init(void) {
	robot_arm_system_init(&g_robot_arm);
	robot_arm_set_ctrl_dt(&g_robot_arm, (float)ARM_TASK_PERIOD_MS * 0.001f);

	g_arm_target_index = 0;
	g_last_target_index = 0;
	g_place_target_index = 0; 
	g_wait_takeout_target_index = 2;
	g_wait_takeout_seq = WAIT_TAKEOUT_SEQ_NONE;
    g_has_dynamic_target = 0;
	g_pump_wait_state = PUMP_WAIT_NONE;
	g_last_switch_key = 0xFF;
	
	pump_set_state(0);
	robot_arm_apply_target(g_arm_target_index);

#if ARM_USE_REMOTE_KEY
	for (uint8_t i = 0; i < ARM_REMOTE_KEY_COUNT; ++i) {
		remote_register_key_callback((uint8_t)(ARM_SWITCH_KEY + i),
		                             REMOTE_KEY_PRESS_UP,
		                             arm_remote_state_switch);
	}
#endif

	xTaskCreate(robot_arm_task, "arm_ctrl_task", 512, NULL, 3, &g_robot_arm_task_handle);
}

void robot_arm_task(void *pvParameters) {
	UNUSED(pvParameters);
	while (1) {
		robot_arm_update(&g_robot_arm);
		robot_arm_wait_takeout_seq_update();
		robot_arm_check_target_reached();
		pump_check_ready();
		vTaskDelay(pdMS_TO_TICKS(ARM_TASK_PERIOD_MS));
	}
}

void robot_arm_set_state_index(uint8_t index) {
	if (index > 5) return;
	g_arm_target_index = index;
	robot_arm_apply_target(index);
}

void robot_arm_apply_target(uint8_t index) {
	if (g_robot_arm.status == ARM_STATE_PLACE && index != ARM_STATE_PLACE) {
		g_place_target_index = (g_place_target_index + 1) % 3;
	}

    // 【修改1】缓存当前取出索引，并安全递减避免 uint8_t 下溢出
    uint8_t current_takeout_idx = g_wait_takeout_target_index;
	if (g_robot_arm.status == ARM_STATE_WAIT_TAKEOUT && index != ARM_STATE_WAIT_TAKEOUT) {
		g_wait_takeout_target_index = (g_wait_takeout_target_index == 0) ? 2 : (g_wait_takeout_target_index - 1);
	}

	g_pump_wait_state = PUMP_WAIT_NONE;
	arm_status_t prev_status = g_robot_arm.status;
	g_robot_arm.status = arm_status_from_index(index);
	g_robot_arm.last_status = prev_status;

	float target_y = g_arm_target_points[index].y;
	float target_z = g_arm_target_points[index].z;
	float target_pitch = g_arm_target_points[index].pitch;

	if (g_robot_arm.status == ARM_STATE_CATCH && g_has_dynamic_target) {
		target_y = g_dynamic_target.y;
		target_z = g_dynamic_target.z;
		g_has_dynamic_target = 0; 
	}

	if (g_robot_arm.status == ARM_STATE_PLACE) {
		target_y = g_arm_place_points[g_place_target_index].y;
		target_z = g_arm_place_points[g_place_target_index].z;
		target_pitch = g_arm_place_points[g_place_target_index].pitch;
	}

	if (g_robot_arm.status == ARM_STATE_WAIT_TAKEOUT) {
		target_y = g_arm_wait_takeout_points[g_wait_takeout_target_index].y;
		target_z = g_arm_wait_takeout_points[g_wait_takeout_target_index].z;
		target_pitch = g_arm_wait_takeout_points[g_wait_takeout_target_index].pitch;
	}

	// 【修改2】解除注释，并使用切换前的 current_takeout_idx (0 表示底层) 进行判断
	if (g_robot_arm.status == ARM_STATE_TAKEOUT && 
	    prev_status == ARM_STATE_WAIT_TAKEOUT && 
	    current_takeout_idx == 0) {
		
		robot_arm_wait_takeout_seq_init(target_y, target_z, target_pitch);
		g_robot_arm.arm_motion_active = 0; 
		g_last_target_index = index;
		return; /* 直接返回，阻断常规的笛卡尔空间直线插补调用 */
	}

	g_wait_takeout_seq = WAIT_TAKEOUT_SEQ_NONE;
	robot_arm_mark_reach_target(target_y, target_z, target_pitch);
	robot_arm_set_target(&g_robot_arm, target_y, target_z, target_pitch);
	g_robot_arm.arm_motion_active = 0; 
	g_last_target_index = index;
}

static float arm_ctrl_wrap_pi(float angle) {
	while (angle > PI) angle -= 2.0f * PI;
	while (angle < -PI) angle += 2.0f * PI;
	return angle;
}

static void robot_arm_wait_takeout_seq_init(float y, float z, float pitch) {
	float final_joint[3];
	arm_pos_angle(y, z, pitch, final_joint);

	float diff0 = arm_ctrl_wrap_pi(final_joint[0] - g_robot_arm.damiao_1.position);
	float diff1 = arm_ctrl_wrap_pi(final_joint[1] - g_robot_arm.damiao_3.position);
	float dir0 = (diff0 >= 0.0f) ? 1.0f : -1.0f;
	float dir1 = (diff1 >= 0.0f) ? 1.0f : -1.0f;

	g_wait_takeout_final_y = y;
	g_wait_takeout_final_z = z;
	g_wait_takeout_final_pitch = pitch;

	g_wait_takeout_pre_big_arm = g_robot_arm.damiao_1.position + dir0 * ARM_WAIT_TAKEOUT_BOTTOM_BIG_ARM_STEP_RAD;
	if (g_wait_takeout_pre_big_arm < ARM_BIG_ARM_MIN_ANGLE_RAD) {
		g_wait_takeout_pre_big_arm = ARM_BIG_ARM_MIN_ANGLE_RAD;
	}
	g_wait_takeout_pre_small_arm = g_robot_arm.damiao_3.position + dir1 * ARM_WAIT_TAKEOUT_BOTTOM_SMALL_ARM_STEP_RAD;

	g_wait_takeout_seq = WAIT_TAKEOUT_SEQ_BIG_ARM;
	g_arm_reach_pending = 0;

	robot_arm_set_joint_target(&g_robot_arm, g_wait_takeout_pre_big_arm, g_robot_arm.damiao_3.position, g_robot_arm.damiao_4.position);
}

static void robot_arm_wait_takeout_seq_update(void) {
	if (g_robot_arm.status != ARM_STATE_TAKEOUT || g_wait_takeout_seq == WAIT_TAKEOUT_SEQ_NONE) return;

	float big_err = fabsf(g_robot_arm.damiao_1.position - g_wait_takeout_pre_big_arm);
	float small_err = fabsf(g_robot_arm.damiao_3.position - g_wait_takeout_pre_small_arm);

	if (g_wait_takeout_seq == WAIT_TAKEOUT_SEQ_BIG_ARM) {
		if (big_err <= ARM_WAIT_TAKEOUT_PRE_STEP_ERR_RAD) {
			g_wait_takeout_seq = WAIT_TAKEOUT_SEQ_SMALL_ARM;
			robot_arm_set_joint_target(&g_robot_arm, g_wait_takeout_pre_big_arm, g_wait_takeout_pre_small_arm, g_robot_arm.damiao_4.position);
		}
		return;
	}

	if (g_wait_takeout_seq == WAIT_TAKEOUT_SEQ_SMALL_ARM) {
		if (small_err <= ARM_WAIT_TAKEOUT_PRE_STEP_ERR_RAD) {
			g_wait_takeout_seq = WAIT_TAKEOUT_SEQ_NONE;
			robot_arm_mark_reach_target(g_wait_takeout_final_y, g_wait_takeout_final_z, g_wait_takeout_final_pitch);
			robot_arm_set_target(&g_robot_arm, g_wait_takeout_final_y, g_wait_takeout_final_z, g_wait_takeout_final_pitch);
			g_robot_arm.arm_motion_active = 0;
		}
	}
}

static void robot_arm_mark_reach_target(float y, float z, float pitch) {
	// 将末端位置转换为关节角度
	float joint_angles[3];
	arm_pos_angle(y, z, pitch, joint_angles);
	g_arm_reach_target_joint[0] = joint_angles[0];  // j1 (大臂)
	g_arm_reach_target_joint[1] = joint_angles[1];  // j3 (小臂)
	g_arm_reach_pending = 1;
}

static void robot_arm_check_target_reached(void) {
	if (!g_arm_reach_pending) return;

	// 判断关节角到位，忽略damiao_4
	float err_j1 = arm_ctrl_wrap_pi(g_robot_arm.damiao_1.position - g_arm_reach_target_joint[0]);
	float err_j3 = arm_ctrl_wrap_pi(g_robot_arm.damiao_3.position - g_arm_reach_target_joint[1]);

	if (fabsf(err_j1) <= ARM_REACH_JOINT_TOL_RAD && fabsf(err_j3) <= ARM_REACH_JOINT_TOL_RAD) {
		g_arm_reach_pending = 0;
		if (g_robot_arm.status == ARM_STATE_CATCH) {
			pump_set_state(1);
#if ARM_USE_PUMP_ADC_CHECK
			g_pump_wait_state = PUMP_WAIT_CATCH;
			return;
#else
			control_dispatch_publish(1);
			return;
#endif
		}
		if (g_robot_arm.status == ARM_STATE_PLACE) {
			pump_set_state(0);
#if ARM_USE_PUMP_ADC_CHECK
			g_pump_wait_state = PUMP_WAIT_PLACE;
			return;
#else
			control_dispatch_publish(1);
			return;
#endif
		}
		control_dispatch_publish(1);
	}
}

static void pump_check_ready(void) {
#if !ARM_USE_PUMP_ADC_CHECK
	(void)g_pump_wait_state;
	return;
#endif
	if (g_pump_wait_state == PUMP_WAIT_NONE) return;

	uint16_t adc_value = 0;
	if (!pump_read_adc(&adc_value)) return;

	if (g_pump_wait_state == PUMP_WAIT_CATCH && adc_value > PUMP_ADC_READY_HIGH) {
		control_dispatch_publish(1);
		g_pump_wait_state = PUMP_WAIT_NONE;
	} else if (g_pump_wait_state == PUMP_WAIT_PLACE && adc_value < PUMP_ADC_READY_LOW) {
		control_dispatch_publish(1);
		g_pump_wait_state = PUMP_WAIT_NONE;
	}
}

static void pump_set_state(uint8_t on) {
	HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief 读取气泵 ADC 压力值
 */
static uint8_t pump_read_adc(uint16_t *out_value) {
    if (out_value == NULL) return 0;

    // 1. 直接启动 ADC 转换
    if (HAL_ADC_Start(&hadc3) != HAL_OK) return 0;

    // 2. 轮询等待转换完成 (超时时间设为 2ms)
    if (HAL_ADC_PollForConversion(&hadc3, 2) != HAL_OK) {
        HAL_ADC_Stop(&hadc3);
        return 0;
    }

    // 3. 读取数据并停止 ADC
    *out_value = (uint16_t)HAL_ADC_GetValue(&hadc3);
    HAL_ADC_Stop(&hadc3);
    
    return 1;
}

#if ARM_USE_REMOTE_KEY
static void arm_remote_state_switch(uint8_t key, remote_key_event_t event) {
	UNUSED(event);
	if (key >= ARM_SWITCH_KEY && key < (uint8_t)(ARM_SWITCH_KEY + ARM_REMOTE_KEY_COUNT)) {
		if (key == g_last_switch_key) {
			return;
		}
		g_last_switch_key = key;
		robot_arm_set_state_index((uint8_t)(key - ARM_SWITCH_KEY));
	}
}
#endif