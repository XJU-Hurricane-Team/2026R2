/**
 * @file arm_ctrl.h
 * @author xinglu
 * @brief 机械臂控制 (应用层接口与状态管理)
 * @version 3.1
 * @date 2026-05-17
 */

#ifndef __ARM_CTRL_H
#define __ARM_CTRL_H

#include "robot_arm/robot_arm.h"

/* ================== 应用层宏定义 ================== */
#define ARM_USE_REMOTE_KEY                                                     \
    0 /**< 是否使用遥控器按键切换状态位 (0:禁用, 1:启用) */
#define ARM_TASK_PERIOD_MS      10    /**< 机械臂控制任务周期 (毫秒) */
#define ARM_REACH_POS_TOL_MM    5.0f  /**< 位置到位判定容差 (mm) */
#define ARM_REACH_JOINT_TOL_RAD 0.10f /**< 关节角到位判定容差 (rad) */

#define ARM_USE_PUMP_ADC_CHECK  1     /**< 是否使用气泵ADC压力检测 */
#define PUMP_ADC_READY_LOW      3700U /**< 抓取成功判定的低压阈值 */
#define PUMP_ADC_READY_HIGH     4000U /**< 释放成功判定的高压阈值 */

#define ARM_PUMP_PLACE_TIMEOUT_ENABLE                                          \
    1               /**< 放置态气泵超时检测使能 (0:禁用, 1:启用) */
#define ARM_PUMP_CATCH_TIMEOUT_ENABLE                                          \
    1 /**< 抓取/待取态纯超时使能 (电机到位后等2秒自动抓取, 0:原ADC重试逻辑) */
#define USE_FLASH 1 /**< 是否使用Flash存储层数 (0:禁用, 1:启用) */

#define CATCH_READY_2_ANGEL                                                    \
    -0.893f /**< READY_2 吸盘关节相对于水平面旋转角度 (rad) */
#define FIRST_POINT_Z     (493.991) /**< 第一排底层基准高度 (mm) */
#define FIRST_POINT_Z_LOW 280.0f    /**< 抓取位置基准高度 (mm) */

#define CAM_TO_CAT_Y_OFFSET                                                    \
    66.445f /**< 摄像头坐标系与吸盘坐标系 Y 轴偏移 (mm) */
#define CAM_TO_CAT_Z_OFFSET                                                    \
    66.374f /**< 摄像头坐标系与吸盘坐标系 Z 轴偏移 (mm) */

/* ================== 应用层结构体与枚举 ================== */

/**
 * @brief 目标点位结构体 (笛卡尔空间)
 */
typedef struct {
    float y;     /**< 前后方向坐标 (mm) */
    float z;     /**< 垂直高度 (mm) */
    float pitch; /**< 末端吸盘姿态 (弧度，水平为0，下倾为负) */
} arm_target_point_t;

/**
 * @brief 气泵等待状态枚举
 */
typedef enum {
    PUMP_WAIT_NONE = 0, /**< 无等待状态 */
    PUMP_WAIT_CATCH,    /**< 等待抓取成功 */
    PUMP_WAIT_PLACE,    /**< 等待释放成功 */
} pump_wait_state_t;

extern bool arm_return_enabel; /* 放置完成后回位功能使能标志 */

/* ================== API 声明 ================== */

void robot_arm_init(void);
void robot_arm_set_state_index(uint8_t index);
void pump_set_state(uint8_t on);
void robot_arm_start_place_return_sequence(uint8_t return_index);
void robot_arm_apply_target(uint8_t index);
void robot_arm_set_dynamic_catch_target_up(float y, float x, float z);
void robot_arm_set_dynamic_catch_target_down(float y, float x, float z);
void robot_arm_set_dynamic_catch_target_up2(float y, float x, float z);
void robot_arm_set_dynamic_catch_target_down2(float y, float x, float z);
uint8_t robot_arm_get_layer_count(void);
void robot_arm_set_layer_count(uint8_t count);

#endif /* __ARM_CTRL_H */
