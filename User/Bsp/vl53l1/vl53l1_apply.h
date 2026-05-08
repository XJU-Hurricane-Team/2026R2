#ifndef __VL53L1_APPLY_H
#define __VL53L1_APPLY_H

#include <stdbool.h>
#include <stdint.h>

#define VL53L1_APPLY_DISTANCE_THRESHOLD_MM 100U

void vl53l1_apply_init(void);                              // 测距模块初始化
bool vl53l1_apply_get_distance_mm(uint16_t *distance_mm);  // 获取测量数据

#endif /* __VL53L1_APPLY_H */
