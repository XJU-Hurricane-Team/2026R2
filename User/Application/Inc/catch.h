/**
 * @file catch.h
 * @author whyyy
 * @brief 
 * @version 0.1
 * @date 2026-04-21
 * 
 * @copyright Copyright (c) 2026
 * 
 */

 #ifndef __CATCH_H
 #define __CATCH_H

#include "catch_head/catch_head.h"

/**
 * @brief 夹取状态定义
 * 
 */
typedef enum {
	CATCH_STATE_INIT = 0,  /* 初始状态，等待进入 READY */
	CATCH_STATE_READY,     /* 准备就绪，等待首次稳定检测到物体 */
	CATCH_STATE_GRAB,      /* 抓取状态，等待二次稳定检测到物体 */
	CATCH_STATE_CHECK,     /* 检测状态，等待检测完成 */
	CATCH_STATE_RECOGNIZE, /* 识别状态 */
	CATCH_STATE_DONE,      /* 完成状态 */
	CATCH_STATE_COUNT,     /* 状态数量 */
} catch_state_t;

extern TaskHandle_t catch_feedback_handle;

void catch_set_state(catch_state_t state);

#endif /* __CATCH_H */