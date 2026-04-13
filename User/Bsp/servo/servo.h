/**
 * @file    servo.h
 * @author  xinglu
 * @brief   servo control.
 * @version 1.1
 * @date    2026-04-13
 */

#ifndef __SERVO_H
#define __SERVO_H

#include <cubemx.h>

typedef enum {
    SERVO_OPEN = 0,
    SERVO_CLOSE = 1,
} servo_state_t;

typedef struct {
    TIM_HandleTypeDef *htim; /**< 指向定时器句柄 */
    uint32_t channel;        /**< PWM 通道 */
    uint16_t open_pulse_us;  /**< 打开时的脉冲宽度（微秒） */
    uint16_t close_pulse_us; /**< 关闭时的脉冲宽度（微秒） */
    servo_state_t state;     /**< 当前状态 */
} servo_t;

void servo_init(servo_t *srv, TIM_HandleTypeDef *htim, uint32_t channel,
                uint16_t open_pulse_us, uint16_t close_pulse_us);

HAL_StatusTypeDef servo_set_state(servo_t *srv, servo_state_t state);
void servo_toggle(servo_t *srv);
servo_state_t servo_get_state(servo_t *srv);

#endif /* __SERVO_H */
