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

/* C库文件 */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

/* 模块头文件 */
#include "chassis_calculations/chassis_calculations.h"
#include "omni_wheels/omni_wheels.h"
#include "remote_ctrl/remote_ctrl.h"
#include "message-protocol/msg_protocol.h"
#include "logger/logger.h"  


typedef enum {
    NUC_RX_MODE_CTRL = 0, /*!< 接收控制指令 (v/yaw/vw)，默认 */
    NUC_RX_MODE_POSE,     /*!< 接收位姿数据 (x/y/yaw) */
} nuc_rx_mode_t;

typedef struct {
    float x;
    float y;
    float yaw; /*!< 世界坐标 yaw (world frame)，用于坐标系变换 */
} nuc_pos_data_t;

typedef struct {
    float v;   /*!< 平动速度 m/s */
    float yaw; /*!< 期望运动方向，用于极坐标分解，非世界坐标 yaw */
    float vw;  /*!< 旋转速度 rad/s */

    uint8_t lift;      /*!< 抬升标志：1 触发抬升，2 触发下降，0 无动作 */
} nuc_ctrl_data_t;

extern nuc_pos_data_t g_nuc_pos_data;
extern nuc_ctrl_data_t g_nuc_ctrl_data;
extern nuc_rx_mode_t g_nuc_rx_mode;

void chassis_init(void);
void msg_process_init(void);

void freertos_start(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __INCLUDES_H */
