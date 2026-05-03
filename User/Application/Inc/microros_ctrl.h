/**
 * @file    remote_link.h
 * @author  CV-Engineer-Chen
 * @brief   遥控器接收链路对外接口.
 *          - 提供遥控器消息轮询任务入口
 *          - 连接msg_protocol与remote_ctrl两层模块
 *          - 预留后续接入NUC与ACT_POS的位置
 * @version 0.1
 * @date    2026-03-11
 */

#ifndef __REMOTE_LINK_H
#define __REMOTE_LINK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "custom_msg/msg/speed_heading.h"
extern custom_msg__msg__SpeedHeading nav_pram;


void stair_microros_init(void);
void stair_microros_publish(int8_t status);
void grab_microros_publish(int8_t status);
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __REMOTE_LINK_H */
