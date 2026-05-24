/**
 * @file    microros_ctrl.h
 * @author  whyyy
 * @brief   MicroROS 控制模块对外接口.
 *          - 提供MicroROS任务入口
 *          - 连接各个功能模块
 * @version 0.1
 * @date    2026-03-11
 */

#ifndef __MICROROS_CTRL_H
#define __MICROROS_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "custom_msg/msg/speed_heading.h"
extern custom_msg__msg__SpeedHeading nav_sub_pram;



void nav_publish(int8_t status);
void control_dispatch_publish(int8_t status);
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __MICROROS_CTRL_H */
