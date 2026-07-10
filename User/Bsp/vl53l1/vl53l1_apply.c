#include "vl53l1_apply.h"
#include "iic/iic.h"
#include "iic/iic2.h"
#include "iic/iic3.h"
#include "FreeRTOS.h"
#include "task.h"

/* ========== 所有传感器统一使用地址 0x52（各自独立 I2C 总线） ========== */
#define VL53L1_APPLY_I2C_ADDR             0x52U
#define VL53L1_APPLY_TIMING_BUDGET_US     20000U
#define VL53L1_APPLY_INTER_MEASUREMENT_MS 30U

/* ========== XSHUT 引脚定义（拉低=复位，拉高=使能）请按实际接线修改 ========== */
#define VL53L1_1_XSHUT_GPIO_PORT          GPIOD
#define VL53L1_1_XSHUT_GPIO_PIN           GPIO_PIN_10
#define VL53L1_2_XSHUT_GPIO_PORT          GPIOA
#define VL53L1_2_XSHUT_GPIO_PIN           GPIO_PIN_7
#define VL53L1_3_XSHUT_GPIO_PORT          GPIOD
#define VL53L1_3_XSHUT_GPIO_PIN           GPIO_PIN_11

/* ========== 每个传感器绑定的软件 I2C 总线 ID ========== */
#define VL53L1_1_I2C_BUS_ID               0  /* I2C1: PC6/PC7 */
#define VL53L1_2_I2C_BUS_ID               1  /* I2C2: PB6/PB7 */
#define VL53L1_3_I2C_BUS_ID               2  /* I2C3: PB8/PB9 */

/* ========== 设备句柄 ========== */
VL53L1_Dev_t g_vl53l1_dev;
VL53L1_DEV g_vl53l1_handle = &g_vl53l1_dev;

VL53L1_Dev_t g_vl53l1_dev2;
VL53L1_DEV g_vl53l1_handle2 = &g_vl53l1_dev2;

VL53L1_Dev_t g_vl53l1_dev3;
VL53L1_DEV g_vl53l1_handle3 = &g_vl53l1_dev3;

/* ========== 每个设备独立的就绪标志 ========== */
static bool g_vl53l1_ready[3] = {true, true, true};

/* ---- 辅助：根据句柄获取就绪标志 ---- */
static bool *vl53l1_get_ready_ptr(VL53L1_DEV handle) {
    if (handle == g_vl53l1_handle)  return &g_vl53l1_ready[0];
    if (handle == g_vl53l1_handle2) return &g_vl53l1_ready[1];
    if (handle == g_vl53l1_handle3) return &g_vl53l1_ready[2];
    return NULL;
}

/**
 * @brief 配置单个 VL53L1 传感器的测距参数
 */
static bool vl53l1_apply_configure(VL53L1_DEV handle) {
    VL53L1_Error status;

    status = VL53L1_DataInit(handle);
    if (status != VL53L1_ERROR_NONE) return false;

    status = VL53L1_StaticInit(handle);
    if (status != VL53L1_ERROR_NONE) return false;

    status = VL53L1_SetDistanceMode(handle, VL53L1_DISTANCEMODE_SHORT);
    if (status != VL53L1_ERROR_NONE) return false;

    status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(
        handle, VL53L1_APPLY_TIMING_BUDGET_US);
    if (status != VL53L1_ERROR_NONE) return false;

    status = VL53L1_SetInterMeasurementPeriodMilliSeconds(
        handle, VL53L1_APPLY_INTER_MEASUREMENT_MS);
    if (status != VL53L1_ERROR_NONE) return false;

    return true;
}

/**
 * @brief 初始化单个 VL53L1 传感器
 * @param handle   设备句柄（已预设 I2cDevAddr 和 i2c_bus_id）
 * @param xshut_port XSHUT 引脚端口
 * @param xshut_pin  XSHUT 引脚编号
 */
static bool vl53l1_apply_init_single(VL53L1_DEV handle,
                                      GPIO_TypeDef *xshut_port,
                                      uint16_t xshut_pin) {
    /* 拉高 XSHUT 使能传感器 */
    HAL_GPIO_WritePin(xshut_port, xshut_pin, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(2));

    /* 等待启动完成 */
    if (VL53L1_WaitDeviceBooted(handle) != VL53L1_ERROR_NONE) return false;

    /* 配置测距参数并启动测量 */
    return vl53l1_apply_configure(handle);
}

/**
 * @brief 初始化所有 VL53L1 传感器（3 个，各自独立 I2C 总线，统一地址 0x52）
 */
void vl53l1_apply_init(void) {
    /* ---- 1. 初始化三个软件 I2C 总线 ---- */
    iic_init();
    iic2_init();
    iic3_init();

    /* ---- 2. 预设各设备句柄的 I2C 地址和总线 ID ---- */
    g_vl53l1_handle->I2cDevAddr  = VL53L1_APPLY_I2C_ADDR;
    g_vl53l1_handle->i2c_bus_id  = VL53L1_1_I2C_BUS_ID;

    g_vl53l1_handle2->I2cDevAddr = VL53L1_APPLY_I2C_ADDR;
    g_vl53l1_handle2->i2c_bus_id = VL53L1_2_I2C_BUS_ID;

    g_vl53l1_handle3->I2cDevAddr = VL53L1_APPLY_I2C_ADDR;
    g_vl53l1_handle3->i2c_bus_id = VL53L1_3_I2C_BUS_ID;

    /* ---- 3. 配置 XSHUT 引脚为输出模式（CubeMX 默认可能为模拟） ---- */
    {
        GPIO_InitTypeDef gpio = {0};
        gpio.Mode = GPIO_MODE_OUTPUT_PP;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;

        gpio.Pin = VL53L1_1_XSHUT_GPIO_PIN;
        HAL_GPIO_Init(VL53L1_1_XSHUT_GPIO_PORT, &gpio);
        gpio.Pin = VL53L1_2_XSHUT_GPIO_PIN;
        HAL_GPIO_Init(VL53L1_2_XSHUT_GPIO_PORT, &gpio);
        gpio.Pin = VL53L1_3_XSHUT_GPIO_PIN;
        HAL_GPIO_Init(VL53L1_3_XSHUT_GPIO_PORT, &gpio);
    }

    /* ---- 4. 全部 XSHUT 拉低复位 ---- */
    HAL_GPIO_WritePin(VL53L1_1_XSHUT_GPIO_PORT, VL53L1_1_XSHUT_GPIO_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(VL53L1_2_XSHUT_GPIO_PORT, VL53L1_2_XSHUT_GPIO_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(VL53L1_3_XSHUT_GPIO_PORT, VL53L1_3_XSHUT_GPIO_PIN, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* ---- 5. 逐一初始化各传感器 ---- */
    g_vl53l1_ready[0] = vl53l1_apply_init_single(g_vl53l1_handle,
                            VL53L1_1_XSHUT_GPIO_PORT, VL53L1_1_XSHUT_GPIO_PIN);
    g_vl53l1_ready[1] = vl53l1_apply_init_single(g_vl53l1_handle2,
                            VL53L1_2_XSHUT_GPIO_PORT, VL53L1_2_XSHUT_GPIO_PIN);
    g_vl53l1_ready[2] = vl53l1_apply_init_single(g_vl53l1_handle3,
                            VL53L1_3_XSHUT_GPIO_PORT, VL53L1_3_XSHUT_GPIO_PIN);

    /* ---- 6. 初始化后默认关闭测距 ---- */
    vl53l1_apply_stop_measurement(g_vl53l1_handle);
    vl53l1_apply_stop_measurement(g_vl53l1_handle2);
    vl53l1_apply_stop_measurement(g_vl53l1_handle3);
}

/**
 * @brief 获取指定传感器的测距数据
 * @param distance_mm 输出：距离值（毫米）
 * @param handle      传感器句柄（g_vl53l1_handle / handle2 / handle3）
 * @return true 获取成功，false 失败（未就绪 / 数据无效 / 参数错误）
 */
bool vl53l1_apply_get_distance_mm(uint16_t *distance_mm, VL53L1_DEV handle) {
    uint8_t data_ready = 0;
    VL53L1_RangingMeasurementData_t ranging_data;
    bool *ready;
    static uint8_t err_count[3] = {0, 0, 0}; /* 三个传感器的连续错误计数 */
    uint8_t *err_p;

    if (distance_mm == NULL || handle == NULL) return false;

    /* 检查该设备是否已初始化成功 */
    ready = vl53l1_get_ready_ptr(handle);
    if (ready == NULL || !(*ready)) return false;

    /* 获取对应错误计数器指针 */
    if (handle == g_vl53l1_handle)  err_p = &err_count[0];
    else if (handle == g_vl53l1_handle2) err_p = &err_count[1];
    else err_p = &err_count[2];

    /* 查询数据是否准备好 */
    if (VL53L1_GetMeasurementDataReady(handle, &data_ready) != VL53L1_ERROR_NONE) {
        (*err_p)++;
        return false;
    }

    if (data_ready == 0U) {
        (*err_p) = 0; /* 数据未就绪是正常状态，清除错误计数 */
        return false;
    }

    /* 读取测距数据 */
    if (VL53L1_GetRangingMeasurementData(handle, &ranging_data) != VL53L1_ERROR_NONE) {
        (*err_p)++;
        /* 连续错误超过阈值，尝试重新初始化传感器 */
        if (*err_p >= 10) {
            VL53L1_StopMeasurement(handle);
            if (vl53l1_apply_configure(handle)) {
                VL53L1_StartMeasurement(handle);
            }
            /* 即使重配置失败也不标记永久不可用，下次还会重试 */
            *err_p = 0;
        }
        /* 单次错误不致命，重新触发下一次测量 */
        VL53L1_ClearInterruptAndStartMeasurement(handle);
        return false;
    }

    *err_p = 0; /* 成功读取，清零错误计数 */

    /* 清除中断，触发下一次测量 */
    VL53L1_ClearInterruptAndStartMeasurement(handle);

    if (ranging_data.RangeStatus != VL53L1_RANGESTATUS_RANGE_VALID)
        return false;

    if (ranging_data.RangeMilliMeter < 0)
        return false;

    *distance_mm = (uint16_t)ranging_data.RangeMilliMeter;
    return true;
}

/**
 * @brief 启动指定传感器的连续测距
 */
void vl53l1_apply_start_measurement(VL53L1_DEV handle) {
    bool *ready = vl53l1_get_ready_ptr(handle);
    if (ready == NULL || !(*ready)) return;
    VL53L1_StartMeasurement(handle);
}

/**
 * @brief 停止指定传感器的连续测距
 */
void vl53l1_apply_stop_measurement(VL53L1_DEV handle) {
    bool *ready = vl53l1_get_ready_ptr(handle);
    if (ready == NULL || !(*ready)) return;
    VL53L1_StopMeasurement(handle);
}
