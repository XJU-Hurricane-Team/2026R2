/**
 * @file    includes.h
 * @author  Deadline039
 * @brief   Include files
 * @version 1.0
 * @date    2024-04-03
 */

#ifndef __INCLUDES_H
#define __INCLUDES_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <bsp.h>

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

#include "math.h"
#include "arm_math.h"

#include "MicroROSConfig.h"

#include <stdio.h>
#include <stdbool.h>

void freertos_start(void);

typedef enum{
    CATCH_STATUS_INIT = 1, // 机械臂初始化，即上电零点位置
    CATCH_STATUS_READY,    // 两机械臂互相垂直，准备抓取
}catch_status_t;

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __INCLUDES_H */
