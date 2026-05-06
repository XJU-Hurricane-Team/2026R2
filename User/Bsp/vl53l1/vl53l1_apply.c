#include "vl53l1_apply.h"

#include "vl53l1_api.h"

#include "iic/iic.h"

#define VL53L1_APPLY_I2C_ADDR                 0x52U
#define VL53L1_APPLY_TIMING_BUDGET_US         50000U
#define VL53L1_APPLY_INTER_MEASUREMENT_MS     20U

static VL53L1_Dev_t g_vl53l1_dev;                      // 数据结构体
static VL53L1_DEV g_vl53l1_handle = &g_vl53l1_dev;     // 设备句柄
static bool g_vl53l1_ready = false;                    // 设备就绪标志

static bool vl53l1_apply_configure(void) {

    // 初始化传感器
	VL53L1_Error status = VL53L1_DataInit(g_vl53l1_handle);
	if (status != VL53L1_ERROR_NONE) {
		return false;
	}

	status = VL53L1_StaticInit(g_vl53l1_handle);
	if (status != VL53L1_ERROR_NONE) {
		return false;
	}

    // 配置测距参数
    // 设置为短距离模式
	status = VL53L1_SetDistanceMode(g_vl53l1_handle,
									VL53L1_DISTANCEMODE_SHORT);
	if (status != VL53L1_ERROR_NONE) {
		return false;
	}

    // 测量时间预算
	status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(
		g_vl53l1_handle, VL53L1_APPLY_TIMING_BUDGET_US);
	if (status != VL53L1_ERROR_NONE) {
		return false;
	}

    // 测量时间间隔
	status = VL53L1_SetInterMeasurementPeriodMilliSeconds(
		g_vl53l1_handle, VL53L1_APPLY_INTER_MEASUREMENT_MS);
	if (status != VL53L1_ERROR_NONE) {
		return false;
	}

    // 启动连续测量
	status = VL53L1_StartMeasurement(g_vl53l1_handle);
	if (status != VL53L1_ERROR_NONE) {
		return false;
	}

	return true;
}

void vl53l1_apply_init(void) {
	iic_init();

	g_vl53l1_ready = false;
	g_vl53l1_handle->I2cDevAddr = VL53L1_APPLY_I2C_ADDR;

	if (VL53L1_WaitDeviceBooted(g_vl53l1_handle) != VL53L1_ERROR_NONE) {
		return;
	}

	g_vl53l1_ready = vl53l1_apply_configure();
	return;
}

bool vl53l1_apply_get_distance_mm(uint16_t *distance_mm) {
	uint8_t data_ready = 0;
	VL53L1_RangingMeasurementData_t ranging_data;

	if ((distance_mm == NULL) || !g_vl53l1_ready) {
		return false;
	}

    // 查询数据是否准备好
	if (VL53L1_GetMeasurementDataReady(g_vl53l1_handle, &data_ready) !=
		VL53L1_ERROR_NONE) {
		return false;
	}

	if (data_ready == 0U) {
		return false;
	}

    // 读取测距数据
	if (VL53L1_GetRangingMeasurementData(g_vl53l1_handle, &ranging_data) !=
		VL53L1_ERROR_NONE) {
		return false;
	}

    // 清除中断状态，触发下一次测量
	VL53L1_ClearInterruptAndStartMeasurement(g_vl53l1_handle);

	if (ranging_data.RangeStatus != VL53L1_RANGESTATUS_RANGE_VALID) {
		return false;
	}

	if (ranging_data.RangeMilliMeter < 0) {
		return false;
	}

	*distance_mm = (uint16_t)ranging_data.RangeMilliMeter;
	return true;
}
