/**
 * @file    servo.c
 * @author  xinglu
 * @brief   servo control.
 * @version 1.1
 * @date    2026-04-13
 */

#include "servo.h"

/**
 * @brief 初始化舵机实例并启动 PWM
 *
 * 启动 PWM，并将舵机初始化为关闭位置。
 */
void servo_init(servo_t *srv, TIM_HandleTypeDef *htim, uint32_t channel,
                uint16_t open_pulse_us, uint16_t close_pulse_us) {
    if (srv == NULL || htim == NULL) return;

    srv->htim = htim;
    srv->channel = channel;
    srv->open_pulse_us = open_pulse_us;
    srv->close_pulse_us = close_pulse_us;
    srv->state = SERVO_CLOSE;

    HAL_TIM_PWM_Start(srv->htim, srv->channel);
    __HAL_TIM_SET_COMPARE(srv->htim, srv->channel, srv->close_pulse_us);
}

/**
 * @brief 设置舵机到指定状态（打开/关闭）
 *
 * 将对应的脉冲宽度写入定时器比较寄存器以改变舵机位置。
 */
HAL_StatusTypeDef servo_set_state(servo_t *srv, servo_state_t state) {
    if (srv == NULL || srv->htim == NULL) return HAL_ERROR;

    uint16_t pulse_us = (state == SERVO_CLOSE) ? srv->close_pulse_us : srv->open_pulse_us;
    __HAL_TIM_SET_COMPARE(srv->htim, srv->channel, pulse_us);

    srv->state = state;
    return HAL_OK;
}

/**
 * @brief 在打开和关闭之间切换舵机状态并应用
 */
void servo_toggle(servo_t *srv) {
    if (srv == NULL || srv->htim == NULL) return;

    srv->state = (srv->state == SERVO_CLOSE) ? SERVO_OPEN : SERVO_CLOSE;
    servo_set_state(srv, srv->state);
}

/**
 * @brief 获取当前舵机状态
 */
servo_state_t servo_get_state(servo_t *srv) {
    if (srv == NULL) return SERVO_CLOSE;
    return srv->state;
}
