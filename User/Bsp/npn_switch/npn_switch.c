/**
 * @file npn_switch.c
 * @author xinglu
 * @brief NPN¿ª¹ØÄ£¿é
 * @version 1.0
 * @date 2026-04-13
 */

#include "npn_switch.h"

GPIO_PinState npn_switch_read_level(void) {
	return HAL_GPIO_ReadPin(NPN_SWITCH_GPIO_PORT, NPN_SWITCH_GPIO_PIN);
}
