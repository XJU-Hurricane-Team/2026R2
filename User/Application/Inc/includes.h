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

/* FreeRTOS头文件 */
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "semphr.h"

/* C库文件 */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

/* 公共模块头文件 */
#include "remote_ctrl/remote_ctrl.h"
#include "message-protocol/msg_protocol.h"
#include "logger/logger.h"  
#include "robot_arm/robot_arm.h"


void chassis_init(void);
void catch_init(void);
void msg_process_init(void);
int microros_init(void);
void logger_module_init(void);
void robot_arm_init(void);


void freertos_start(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __INCLUDES_H */
