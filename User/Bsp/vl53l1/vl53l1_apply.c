#include "vl53l1_apply.h"
#include "iic/iic.h"
#include "FreeRTOS.h"      
#include "task.h"  

#define VL53L1_APPLY_I2C_ADDR             0x54U
#define VL53L1_2_APPLY_I2C_ADDR           0x52U
#define VL53L1_APPLY_TIMING_BUDGET_US     20000U
#define VL53L1_APPLY_INTER_MEASUREMENT_MS 30U

// XSHUT 引脚定义（拉低=复位，拉高=使能）
#define VL53L1_1_XSHUT_GPIO_PORT          GPIOA
#define VL53L1_1_XSHUT_GPIO_PIN           GPIO_PIN_6 // 根据实际情况填写

#define VL53L1_2_XSHUT_GPIO_PORT          GPIOA
#define VL53L1_2_XSHUT_GPIO_PIN           GPIO_PIN_7 // 根据实际情况填写

VL53L1_Dev_t g_vl53l1_dev;                  // 数据结构体
VL53L1_DEV g_vl53l1_handle = &g_vl53l1_dev; // 设备句柄

VL53L1_Dev_t g_vl53l1_dev2;                   // 数据结构体
VL53L1_DEV g_vl53l1_handle2 = &g_vl53l1_dev2; // 设备句柄

static bool g_vl53l1_ready = false; // 设备就绪标志

static bool vl53l1_apply_configure(VL53L1_DEV handle) {

    // 初始化传感器
    VL53L1_Error status = VL53L1_DataInit(handle);
    if (status != VL53L1_ERROR_NONE) {
        return false;
    }

    status = VL53L1_StaticInit(handle);
    if (status != VL53L1_ERROR_NONE) {
        return false;
    }

    // 配置测距参数
    // 设置为短距离模式
    status = VL53L1_SetDistanceMode(handle, VL53L1_DISTANCEMODE_SHORT);
    if (status != VL53L1_ERROR_NONE) {
        return false;
    }

    // 测量时间预算
    status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(
        handle, VL53L1_APPLY_TIMING_BUDGET_US);
    if (status != VL53L1_ERROR_NONE) {
        return false;
    }

    // 测量时间间隔
    status = VL53L1_SetInterMeasurementPeriodMilliSeconds(
        handle, VL53L1_APPLY_INTER_MEASUREMENT_MS);
    if (status != VL53L1_ERROR_NONE) {
        return false;
    }

    // 启动连续测量
    // status = VL53L1_StartMeasurement(handle);
    // if (status != VL53L1_ERROR_NONE) {
    //     return false;
    // }

    return true;
}

void vl53l1_apply_init(void) {
    iic_init();
    g_vl53l1_ready = false;

    // ========== 第一步：确保两个传感器都在复位状态 ==========
    HAL_GPIO_WritePin(VL53L1_1_XSHUT_GPIO_PORT, VL53L1_1_XSHUT_GPIO_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(VL53L1_2_XSHUT_GPIO_PORT, VL53L1_2_XSHUT_GPIO_PIN, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(10)); // 等待10ms确保复位完成

    // ========== 第二步：初始化传感器1，并修改它的地址 ==========
    HAL_GPIO_WritePin(VL53L1_1_XSHUT_GPIO_PORT, VL53L1_1_XSHUT_GPIO_PIN, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(2)); // 等待2ms确保传感器启动

    // 【修复1】传感器1刚唤醒，硬件地址是默认的 0x52，句柄必须用 0x52 才能连上它
    g_vl53l1_handle->I2cDevAddr = VL53L1_2_APPLY_I2C_ADDR;  // 0x52
    if (VL53L1_WaitDeviceBooted(g_vl53l1_handle) != VL53L1_ERROR_NONE) return;

    // 【修复2】通过 0x52 地址发送指令，把硬件地址修改为 0x54
    if (VL53L1_SetDeviceAddress(g_vl53l1_handle, VL53L1_APPLY_I2C_ADDR) != VL53L1_ERROR_NONE) return;
    
    // 【修复3】硬件地址修改成功后，将句柄目标地址同步为 0x54！
    g_vl53l1_handle->I2cDevAddr = VL53L1_APPLY_I2C_ADDR;    // 0x54

    // 此时句柄为 0x54，开始配置测距参数
    g_vl53l1_ready = vl53l1_apply_configure(g_vl53l1_handle);

    // ========== 第三步：初始化传感器2（用默认地址） ==========
    HAL_GPIO_WritePin(VL53L1_2_XSHUT_GPIO_PORT, VL53L1_2_XSHUT_GPIO_PIN, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(2));

    // 传感器2保持默认的 0x52 即可
    g_vl53l1_handle2->I2cDevAddr = VL53L1_2_APPLY_I2C_ADDR; // 0x52
    if (VL53L1_WaitDeviceBooted(g_vl53l1_handle2) != VL53L1_ERROR_NONE) return;

    g_vl53l1_ready &= vl53l1_apply_configure(g_vl53l1_handle2);

    // HAL_GPIO_WritePin(VL53L1_2_XSHUT_GPIO_PORT, VL53L1_2_XSHUT_GPIO_PIN, GPIO_PIN_SET); 
    // vTaskDelay(pdMS_TO_TICKS(2));
	// g_vl53l1_handle->I2cDevAddr = VL53L1_2_APPLY_I2C_ADDR;

	// if (VL53L1_WaitDeviceBooted(g_vl53l1_handle) != VL53L1_ERROR_NONE) {
	// 	return;
	// }

	// g_vl53l1_ready = vl53l1_apply_configure(g_vl53l1_handle);
	// return;
}


bool vl53l1_apply_get_distance_mm(uint16_t *distance_mm, VL53L1_DEV handle) {
    uint8_t data_ready = 0;
    VL53L1_RangingMeasurementData_t ranging_data;

    if ((distance_mm == NULL) || !g_vl53l1_ready) {
        return false;
    }

    // 查询数据是否准备好
    if (VL53L1_GetMeasurementDataReady(handle, &data_ready) !=
        VL53L1_ERROR_NONE) {
        return false;
    }

    if (data_ready == 0U) {
        return false;
    }

    // 读取测距数据
    if (VL53L1_GetRangingMeasurementData(handle, &ranging_data) !=
        VL53L1_ERROR_NONE) {
             VL53L1_StopMeasurement(handle);
        return false;
    }

    // 清除中断状态，触发下一次测量
    VL53L1_ClearInterruptAndStartMeasurement(handle);

    if (ranging_data.RangeStatus != VL53L1_RANGESTATUS_RANGE_VALID) {
        return false;
    }

    if (ranging_data.RangeMilliMeter < 0) {
        return false;
    }

    *distance_mm = (uint16_t)ranging_data.RangeMilliMeter;
    return true;
}
