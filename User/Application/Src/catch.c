/**
 * @file catch.c
 * @author xinglu
 * @brief 矛头夹取任务
 * @version 1.2
 * @date 2026-04-13
 */

#include "includes.h"
#include "npn_switch/npn_switch.h"

typedef enum {
	CATCH_STATE_INIT = 0,  /* 初始状态，等待进入 READY */
	CATCH_STATE_READY,     /* 准备就绪，等待首次稳定检测到物体 */
	CATCH_STATE_GRAB,      /* 抓取状态，等待二次稳定检测到物体 */
	CATCH_STATE_ASSEMBLY,  /* 拼接状态 */
	CATCH_STATE_COUNT,     /* 状态数量 */
} catch_state_t;

typedef enum {
	CATCH_FLOW_WAIT_FIRST_DETECT = 0,   /* 在 READY 阶段等待首次检测到物体 */
	CATCH_FLOW_WAIT_SECOND_DETECT,      /* 二次检测，确认下一阶段条件成立 */
	CATCH_FLOW_DONE,                    /* 检测完成，进入拼接态*/
} catch_flow_t;

static catch_state_t catch_state = CATCH_STATE_INIT;
static catch_flow_t catch_flow = CATCH_FLOW_WAIT_FIRST_DETECT;

/* 传感器稳定计数：active 连续达到阈值才认为状态有效 */
static uint8_t sensor_active_cnt = 0;
static TaskHandle_t catch_task_handle;
static void catch_task(void *pvParameters);

#define CATCH_SENSOR_COUNT 6

static void catch_tasks_init(void);
static void catch_update(void);
static void catch_set_state(catch_state_t state);

/*
 * @brief 判断目标状态是否为当前状态的下一个合法状态。
 * @param curr 当前状态。
 * @param next 目标状态。
 * @return 1 表示合法顺序跳转，0 表示非法跳转。
 * @note 本函数只做顺序关系判断，不执行状态切换。
 */
static uint8_t catch_is_next_state(catch_state_t curr, catch_state_t next) {
	/* 仅允许按 INIT->READY->GRAB->ASSEMBLY->INIT 的顺序切换 */
	return (uint8_t)(next == (catch_state_t)((curr + 1) % CATCH_STATE_COUNT));
}

/*
 * @brief 根据主状态重置子流程状态，并清空传感器稳定计数。
 * @param state 当前主状态。
 * @note
 * INIT/READY: 进入首次检测流程。
 * GRAB: 进入二次检测流程。
 * ASSEMBLY: 进入流程结束状态。
 * 每次主状态切换后调用，避免沿用旧流程计数。
 */
static void catch_update_flow_for_state(catch_state_t state) {
	switch (state) {
		case CATCH_STATE_INIT:
		case CATCH_STATE_READY: {
			catch_flow = CATCH_FLOW_WAIT_FIRST_DETECT;
		} break;

		case CATCH_STATE_GRAB: {
			catch_flow = CATCH_FLOW_WAIT_SECOND_DETECT;
		} break;

		case CATCH_STATE_ASSEMBLY:
		default: {
			catch_flow = CATCH_FLOW_DONE;
		} break;
	}

	sensor_active_cnt = 0;
}

/*
 * @brief 处理按键触发的手动状态切换。
 * @note
 * KEY0 -> INIT
 * KEY1 -> READY
 * KEY2 -> GRAB
 * WKUP -> ASSEMBLY
 * 实际合法性由 catch_set_state() 再次校验。
 */
static void catch_handle_key_control(void) {
	key_press_t key = key_scan(0);

	switch (key) {
		case KEY0_PRESS: {
			catch_set_state(CATCH_STATE_INIT);
		} break;

		case KEY1_PRESS: {
			catch_set_state(CATCH_STATE_READY);
		} break;

		case KEY2_PRESS: {
			catch_set_state(CATCH_STATE_GRAB);
		} break;

		case WKUP_PRESS: {
			catch_set_state(CATCH_STATE_ASSEMBLY);
		} break;

		default: {
		} break;
	}
}

/*
 * @brief 按状态下发抓头执行器目标值。
 * @param state 需要应用的目标状态。
 * @note
 * 该函数只负责执行器目标配置，不修改状态变量。
 * 不同状态下分别控制舵机、达妙电机、DJI 电机目标位。
 */
static void catch_apply_state(catch_state_t state) {
	switch (state) {
		case CATCH_STATE_INIT: {
			catch_head_set_servo_target(0);
			catch_head_set_dm_target(0);
			catch_head_set_dji_target(0);
		} break;

		case CATCH_STATE_READY: {
			catch_head_set_servo_target(1);
			catch_head_set_dm_target(1);
			catch_head_set_dji_target(0);
		} break;

		case CATCH_STATE_GRAB: {
			catch_head_set_servo_target(0);
		} break;

		case CATCH_STATE_ASSEMBLY: {
			catch_head_set_dji_target(1);
		} break;

		default: {
		} break;
	}
}

/*
 * @brief 抓取模块初始化入口。
 * @note
 * 初始化抓头驱动，设置主状态为 INIT，
 * 同步重置子流程和传感器计数，并下发 INIT 状态对应执行器目标。
 */
void catch_init(void) {
	catch_head_init();

	catch_state = CATCH_STATE_INIT;
	catch_update_flow_for_state(catch_state);

	catch_apply_state(catch_state);
	catch_tasks_init();
}

/*
 * @brief 抓取模块周期更新函数（建议在主循环或任务中周期调用）。
 * @note
 * 处理顺序：
 * 1) 扫描按键并尝试手动切换状态；
 * 2) 采样 NPN 传感器并进行稳定计数；
 * 3) 按子流程条件执行自动状态推进；
 * 4) 调用 catch_head() 执行底层控制更新。
 */
static void catch_update(void) {
	catch_handle_key_control();

	/* NPN 低电平表示检测到物体 */
	uint8_t is_active = (uint8_t)(npn_switch_read_level() == GPIO_PIN_RESET);

	/* 简单消抖：只有连续 N 次同一电平才触发后续状态变更 */
	if (is_active) {
		if (sensor_active_cnt < CATCH_SENSOR_COUNT) {
			sensor_active_cnt++;
		}
	} else {
		sensor_active_cnt = 0;
	}

	switch (catch_flow) {
		case CATCH_FLOW_WAIT_FIRST_DETECT: {
			/* READY 阶段首次稳定检测到物体 -> 进入 GRAB */
			if (catch_state == CATCH_STATE_READY &&
				sensor_active_cnt >= CATCH_SENSOR_COUNT) {
				catch_set_state(CATCH_STATE_GRAB);
			}
		} break;

		case CATCH_FLOW_WAIT_SECOND_DETECT: {
			/* 二次稳定检测到物体 -> 进入 ASSEMBLY */
			if (catch_state == CATCH_STATE_GRAB &&
				sensor_active_cnt >= CATCH_SENSOR_COUNT) {
				catch_set_state(CATCH_STATE_ASSEMBLY);
			}
		} break;

		case CATCH_FLOW_DONE:
		default: {
		} break;
	}

	catch_head();
}

/*
 * @brief 统一状态切换接口。
 * @param state 目标状态。
 * @note
 * 本函数会做三层保护：
 * 1) 目标状态必须在枚举范围内；
 * 2) 目标状态不能与当前状态相同；
 * 3) 目标状态必须是当前状态的下一个合法状态。
 * 校验通过后，按“更新状态 -> 下发执行器 -> 重置流程”顺序生效。
 */
static void catch_set_state(catch_state_t state) {
	if (state >= CATCH_STATE_COUNT) {
		return;
	}

	if (state == catch_state) {
		return;
	}

	if (catch_is_next_state(catch_state, state) == 0) {
		/* 拒绝跨阶段跳转，保证执行器动作按既定顺序发生 */
		return;
	}

	catch_state = state;
	catch_apply_state(catch_state);
	catch_update_flow_for_state(catch_state);
}

static void catch_task(void *pvParameters) {
	UNUSED(pvParameters);

	while (1) {
		catch_update();
		vTaskDelay(2);
	}
}

static void catch_tasks_init(void) {
	xTaskCreate(catch_task, "catch_task", 256, NULL, 2, &catch_task_handle);
}
