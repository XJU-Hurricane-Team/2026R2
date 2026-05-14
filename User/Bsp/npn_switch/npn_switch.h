/**
 * @file npn_switch.c
 * @author xinglu
 * @brief NPN¿ª¹ØÄ£¿é
 * @version 1.0
 * @date 2026-04-13
 */

#ifndef __NPN_SWITCH_H
#define __NPN_SWITCH_H

#include <cubemx.h>

#define NPN_SWITCH_GPIO_PORT GPIOE
#define NPN_SWITCH_GPIO_PIN  GPIO_PIN_4

GPIO_PinState npn_switch_read_level(void);

#endif /* __NPN_SWITCH_H */
