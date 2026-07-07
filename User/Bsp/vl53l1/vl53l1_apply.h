#ifndef __VL53L1_APPLY_H
#define __VL53L1_APPLY_H

#include <stdbool.h>
#include <stdint.h>
#include "vl53l1_api.h"

#define VL53L1_APPLY_DISTANCE_THRESHOLD_MM 100U

extern VL53L1_DEV g_vl53l1_handle;
extern VL53L1_DEV g_vl53l1_handle2;
extern VL53L1_DEV g_vl53l1_handle3;

void vl53l1_apply_init(void);                              // 测距模块初始化
bool vl53l1_apply_get_distance_mm(uint16_t *distance_mm,
                                   VL53L1_DEV handle);  // 获取测量数据
void vl53l1_apply_start_measurement(VL53L1_DEV handle);   // 启动连续测距
void vl53l1_apply_stop_measurement(VL53L1_DEV handle);    // 停止连续测距

#endif /* __VL53L1_APPLY_H */
