/**
 * @file    chassis_calculations.h
 * @author  Jackrainman
 * @brief   底盘速度规划模块（梯形/余弦/多项式）
 * @version 2.1
 * @date    2026-03-02
 */

#ifndef _CHASSIS_CALCULATIONS_H_
#define _CHASSIS_CALCULATIONS_H_

typedef enum {      /* 模式枚举 */
    CHASSIS_SPEED_PLAN_TRAPEZOID = 0,  /**< 梯形 */
    CHASSIS_SPEED_PLAN_COSINE,         /**< 余弦 */
    CHASSIS_SPEED_PLAN_POLYNOMIAL,     /**< 五次多项式 */
    CHASSIS_SPEED_PLAN_NUM
} chassis_speed_plan_type_t;

typedef struct {
    float max_accel_xy;         /**< 平面合加速度上限（mm/s^2） */
    float max_accel_w;          /**< W 轴最大角加速度（rad/s^2） */
    float max_speed_xy;         /**< 平面速度模长上限（mm/s） */
    float max_speed_w;          /**< W 轴最大角速度（rad/s） */
} chassis_plan_config_t;

void chassis_plan_config_default(chassis_plan_config_t *cfg); /* 获取默认配置 */
void chassis_plan_init(const chassis_plan_config_t *cfg, chassis_speed_plan_type_t mode); /* 初始化规划器 */
void chassis_plan_reset_state(float speed_x, float speed_y, float speed_w); /* 重置规划器状态 */

void chassis_plan_mode_set(chassis_speed_plan_type_t mode); /* 设置速度规划模式 */
chassis_speed_plan_type_t chassis_plan_mode_get(void); /* 获取当前速度规划模式 */
void chassis_plan_limit_accel_set(float accel_xy, float accel_w); /* 设置加速度上限 */
void chassis_plan_limit_speed_set(float speed_xy, float speed_w); /* 设置速度上限 */

void chassis_plan_step(float target_x, float target_y, float target_yaw,
                       float *output_x, float *output_y, float *output_yaw); /* 执行一步速度规划 */

#endif /* _CHASSIS_CALCULATIONS_H_ */
