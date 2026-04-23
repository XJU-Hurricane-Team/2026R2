/**
 * @file arm_ctrl.c
 * @author xinglu
 * @brief 机械臂控制
 *
 * @version 1.0
 * @date 2026-04-24
 */

#include "includes.h"

#define ARM_USE_REMOTE_KEY          0          // 是否启用遥控器按键控制机械臂点位
#define ARM_USE_BOARD_KEY           1          // 是否启用主板KEY0按键控制机械臂点位
#define ARM_SWITCH_KEY              10         // 遥控器按键编号，按下后切换到下一个预设点位
#define ARM_TASK_PERIOD_MS          20         // 机械臂任务周期(ms)

void robot_arm_switch_target(uint8_t key, remote_key_event_t event);

typedef struct {
	float y;
	float z;
	float pitch;
} arm_target_point_t;

static RobotArm g_robot_arm;
static uint8_t g_arm_target_index;
static TaskHandle_t g_robot_arm_task_handle;
static uint8_t g_last_target_index;

static const arm_target_point_t g_arm_target_points[4] = {
	{139.95f + 20.0f, 102.70f, 0.8955f},  // 预设零点
	{533.142f, 400.0f, 0.0f},
	{533.142f, 91.5027f, 0.0f},
	// {-275.12f, 493.99f, -PI/2.0},
	{-275.12f, 653.99f, -PI/2.0},
};

/**
 * @brief 预设点位索引到机械臂状态的映射。
 *
 * 点位0: 初始态；点位1: 就绪态；点位2: 抓取态；点位3: 放置态。
 */
static arm_status_t arm_status_from_index(uint8_t index) {
	switch (index) {
	case 0:
		return ARM_STATE_INIT;
	case 1:
		return ARM_STATE_READY;
	case 2:
		return ARM_STATE_CATCH;
	case 3:
	default:
		return ARM_STATE_PLACE;
	}
}

/**
 * @brief 机械臂应用层初始化：创建任务、初始化机构、设置初始点位并注册按键回调
 * @note 任务句柄和入口函数均由机械臂模块内部管理
 */
void robot_arm_init(void) {
	xTaskCreate(robot_arm_task, "arm_ctrl_task", 512, NULL, 3,
				&g_robot_arm_task_handle);

	robot_arm_system_init(&g_robot_arm);
	robot_arm_set_ctrl_dt((float)ARM_TASK_PERIOD_MS * 0.001f);

	g_arm_target_index = 0;
	g_last_target_index = 0;
	robot_arm_apply_target(g_arm_target_index);

#if ARM_USE_REMOTE_KEY
	remote_register_key_callback(ARM_SWITCH_KEY, REMOTE_KEY_PRESS_UP,
								 robot_arm_switch_target);
#endif
}

/**
 * @brief 机械臂周期任务
 * @note 周期执行目标跟踪与电机控制，以及检测主板按键
 *
 * @param pvParameters 任务参数（未使用）
 */
void robot_arm_task(void *pvParameters) {
	UNUSED(pvParameters);

	while (1) {
		robot_arm_update(&g_robot_arm);

#if ARM_USE_BOARD_KEY
		// 检测主板KEY0按键
		if (key_scan(0) == KEY0_PRESS) {
			g_arm_target_index = (g_arm_target_index + 1) % 4;
			robot_arm_apply_target(g_arm_target_index);
		}
#endif

		vTaskDelay(pdMS_TO_TICKS(ARM_TASK_PERIOD_MS));
	}
}

/**
 * @brief 根据索引应用预设点位
 *
 * @param index 预设点位索引
 */
void robot_arm_apply_target(uint8_t index) {
	float last_y = g_arm_target_points[g_last_target_index].y;
	float next_y = g_arm_target_points[index].y;
	uint8_t catch_to_place = (g_last_target_index == 2) && (index == 3);
	uint8_t quadrant_flipped = ((last_y < 0.0f) != (next_y < 0.0f));
	g_robot_arm.status = arm_status_from_index(index);

	/*
	 * 只要预设点跨过 y 的正负半平面，就认为发生了象限翻转。
	 * 过冲方向仍由 robot_arm.c 固定为正向，这里只负责决定是否启用过冲。
	 */
	if (catch_to_place) {
		robot_arm_set_big_arm_overshoot(&g_robot_arm, 0.10f);
	} else if (quadrant_flipped) {
		robot_arm_set_big_arm_overshoot(&g_robot_arm, 0.45f);
	} else {
		robot_arm_set_big_arm_overshoot(&g_robot_arm, 0.0f);
	}

	robot_arm_set_target(&g_robot_arm, g_arm_target_points[index].y,
						 g_arm_target_points[index].z,
						 g_arm_target_points[index].pitch);
	g_robot_arm.arm_motion_active = 0;
	g_last_target_index = index;
}
    
/**
 * @brief 切换机械臂目标点位
 *
 * @param key 遥控器按键编号
 * @param event 按键事件
 */
void robot_arm_switch_target(uint8_t key, remote_key_event_t event) {
	UNUSED(key);

	if (event != REMOTE_KEY_PRESS_UP) {
		return;
	}

	g_arm_target_index = (g_arm_target_index + 1) % 4;
	robot_arm_apply_target(g_arm_target_index);
}
