/**
 * @file arm_ctrl.c
 * @author xinglu
 * @brief 机械臂控制 (应用层任务与点位逻辑)
 * @version 2.0
 * @date 2026-04-30
 */

#include "includes.h"
#include "arm_ctrl.h"
#include "microros_ctrl.h"
#include "adc.h"

/* === 宏定义与配置区 === */
#define ARM_USE_REMOTE_KEY     99 // 是否使用遥控器按键切换点位 (0:禁用, 1:启用)
#define ARM_SWITCH_KEY         100 // 遥控器映射键值定义
#define ARM_REMOTE_KEY_COUNT   5U
#define ARM_TASK_PERIOD_MS     20   // 机械臂控制任务周期 (毫秒)
#define ARM_REACH_POS_TOL_MM   5.0f // 位置到位判定误差 (mm)

#define ARM_USE_PUMP_ADC_CHECK 1
#define PUMP_ADC_CHANNEL       ADC_CHANNEL_7
#define PUMP_ADC_GPIO_PORT     GPIOD
#define PUMP_ADC_GPIO_PIN      GPIO_PIN_10
#define PUMP_ADC_READY_HIGH    3000U
#define PUMP_ADC_READY_LOW     300U

#define FIRST_POINT_Z          (493.991) // 放置底层的基准高度 (毫米)
#define FIRST_POINT_Z_LOW      280.0f

//493.991f   245.0f

/* 函数前置声明 */
void robot_arm_apply_target(uint8_t index);
void robot_arm_set_state_index(uint8_t index);
void robot_arm_task(void *pvParameters);
static void robot_arm_mark_reach_target(float y, float z);
static void robot_arm_check_target_reached(void);
static void pump_check_ready(void);
static void pump_set_state(uint8_t on);
static uint8_t pump_read_adc(uint16_t *out_value);
static void pump_adc_init_once(void);

#if ARM_USE_REMOTE_KEY
static void arm_remote_state_switch(uint8_t key, remote_key_event_t event);
#endif

/**
 * @brief 定义目标点位结构体 (笛卡尔空间)
 */
typedef struct {
    float y;     // 前后伸出距离 (mm)
    float z;     // 上下高度 (mm)
    float pitch; // 末端倾角姿态 (弧度，水平面为0，向下为负)
} arm_target_point_t;

typedef enum {
    PUMP_WAIT_NONE = 0,
    PUMP_WAIT_CATCH,
    PUMP_WAIT_PLACE,
} pump_wait_state_t;

/* 全局与静态变量定义 */
static RobotArm g_robot_arm;                 // 全局机械臂实体对象
static uint8_t g_arm_target_index;           // 当前全局目标点位索引 (0~4)
static TaskHandle_t g_robot_arm_task_handle; // RTOS 任务句柄
static uint8_t g_last_target_index;          // 记录上一个点位索引

static uint8_t g_place_target_index =
    0; // 当前选择的放置点索引 (0:底层, 1:中层, 2:顶层)
static arm_target_point_t g_dynamic_target = {0}; // 上位机回传的待抓取目标坐标
static uint8_t g_has_dynamic_target = 0; //标志位：1 表示包含有效的新坐标

// 记录最近下发给驱动层的目标位置（单位：mm），用于到位判定比较
static float g_arm_reach_target_y = 0.0f;
static float g_arm_reach_target_z = 0.0f;
static uint8_t g_arm_reach_pending = 0; // 到位检测挂起标志：1 表示正在等待到位
static pump_wait_state_t g_pump_wait_state = PUMP_WAIT_NONE;
static uint8_t g_pump_adc_inited = 0;

/**
 * @brief 全局预设点位数组
 * 分别对应：零点、准备、抓取、放置、取出。
 */
static const arm_target_point_t g_arm_target_points[5] = {
    {139.95f + 20.0f, 102.70f, 0.8955f}, // 0: 预设零点 (INIT)，姿态朝上折叠
    {200.000f, 10.0f, 0.0f},             // 1: 准备点位 (READY)，抬起手臂
    {533.142f, 91.5027f,
     0.0349f}, // 2: 抓取点位 (CATCH)，下降到抓取高度 (将动态被外部坐标覆盖)
    {-275.12f, 493.991f,
     -PI /
         2.0}, // 3: 放置点位 (PLACE)，向后方放置 (该点实际在应用中被下方数组覆盖)
    {533.142f, 300.0f, 0.0f}, // 4: 取出点位 (TAKEOUT)，抓取后的中间姿态
};

/**
 * @brief 独立的放置点数组 (多层货架逻辑)
 * 根据层数 (g_place_target_index)，Z轴高度依次增加，且姿态发生变化。
 */
static const arm_target_point_t g_arm_place_points[3] = {
    {-295.0f, 380.0f, -PI / 2}, // 放置点 0 (底层): 垂直向下放 (-PI/2)
    {-250.12f + 175.0f, FIRST_POINT_Z_LOW + 175.0f,
     -PI}, // 放置点 1 (中层): 水平向后放 (-PI)
    {-250.12f + 175.0f + 30.0f, FIRST_POINT_Z_LOW + 175.0f + 350.0f,
     -PI}, // 放置点 2 (顶层): Z轴加高 350mm
};

/**
 * @brief 将整数索引映射为驱动层的状态枚举
 */
static arm_status_t arm_status_from_index(uint8_t index) {
    switch (index) {
        case 0:
            return ARM_STATE_INIT;
        case 1:
            return ARM_STATE_READY;
        case 2:
            return ARM_STATE_CATCH;
        case 3:
            return ARM_STATE_PLACE;
        case 4:
            return ARM_STATE_TAKEOUT;
        default:
            return ARM_STATE_INIT;
    }
}

/**
 * @brief 外部调用接口：注入动态抓取坐标
 * @note 供 MicroROS 或视觉模块调用，接收后直接切换到抓取态。
 */
void robot_arm_set_dynamic_catch_target(float x, float y, float z) {
    g_dynamic_target.y = y * 1000.0f + 200.0f - 55.0f; // 转换为毫米
    g_dynamic_target.z = z * 1000.0f + 10.0f + 90.0f;  // 转换为毫米
    g_has_dynamic_target = 1;                          // 标记收到有效坐标
    robot_arm_set_state_index((uint8_t)ARM_STATE_CATCH);
}

/**
 * @brief 外部调用接口：手动设置当前要放置的箱子层数
 * @param place_idx 层数索引 (0~2分别代表底、中、顶层)
 */
void robot_arm_set_place_index(uint8_t place_idx) {
    if (place_idx < 3) {
        g_place_target_index = place_idx;
    }
}

/**
 * @brief 机械臂应用层初始化
 */
void robot_arm_init(void) {
    // 1. 初始化底层硬件与数据结构
    robot_arm_system_init(&g_robot_arm);

    // 2. 传递控制周期 dt 给底层滤波器 (ms 转换为 s)
    robot_arm_set_ctrl_dt(&g_robot_arm, (float)ARM_TASK_PERIOD_MS * 0.001f);

    // 3. 初始状态设定
    g_arm_target_index = 0;
    g_last_target_index = 0;
    g_place_target_index = 0;
    g_has_dynamic_target = 0;
    g_pump_wait_state = PUMP_WAIT_NONE;
    pump_set_state(0);
    robot_arm_apply_target(g_arm_target_index);

#if ARM_USE_REMOTE_KEY
    for (uint8_t i = 0; i < ARM_REMOTE_KEY_COUNT; ++i) {
        remote_register_key_callback((uint8_t)(ARM_SWITCH_KEY + i),
                                     REMOTE_KEY_PRESS_UP,
                                     arm_remote_state_switch);
    }
#endif

    // 4. 将任务创建放到初始化最后，确保参数配置完毕后再启动调度
    xTaskCreate(robot_arm_task, "arm_ctrl_task", 512, NULL, 3,
                &g_robot_arm_task_handle);
}

/**
 * @brief 机械臂周期控制任务 (FreeRTOS 线程)
 */
void robot_arm_task(void *pvParameters) {
    UNUSED(pvParameters);

    while (1) {
        robot_arm_update(&g_robot_arm);
        robot_arm_check_target_reached();
        pump_check_ready();
        vTaskDelay(pdMS_TO_TICKS(ARM_TASK_PERIOD_MS));
    }
}

void robot_arm_set_state_index(uint8_t index) {
    if (index > 4) {
        return;
    }
    g_arm_target_index = index;
    robot_arm_apply_target(index);
}

/**
 * @brief 应用/下发预设目标点位
 */
void robot_arm_apply_target(uint8_t index) {
    // 自动累加放置层数逻辑
    if (g_robot_arm.status == ARM_STATE_PLACE && index != 3) {
        g_place_target_index = (g_place_target_index + 1) % 3;
    }

    g_pump_wait_state = PUMP_WAIT_NONE;

    // 映射状态枚举
    g_robot_arm.status = arm_status_from_index(index);

    // 获取默认数组坐标
    float target_y = g_arm_target_points[index].y;
    float target_z = g_arm_target_points[index].z;
    float target_pitch = g_arm_target_points[index].pitch;

    // 抓取点动态覆盖逻辑
    if (g_robot_arm.status == ARM_STATE_CATCH) {
        if (g_has_dynamic_target) {
            target_y = g_dynamic_target.y;
            target_z = g_dynamic_target.z;
            // target_pitch = g_dynamic_target.pitch;

            // 下发完坐标后，清空标志位，重新等待下一个坐标
            g_has_dynamic_target = 0;
        }
    }

    // 放置点独立覆盖逻辑
    if (g_robot_arm.status == ARM_STATE_PLACE) {
        target_y = g_arm_place_points[g_place_target_index].y;
        target_z = g_arm_place_points[g_place_target_index].z;
        target_pitch = g_arm_place_points[g_place_target_index].pitch;
    }

    // 下发给驱动层
    robot_arm_mark_reach_target(target_y, target_z);
    robot_arm_set_target(&g_robot_arm, target_y, target_z, target_pitch);
    g_robot_arm.arm_motion_active = 0; // 重置运动完成标志
    g_last_target_index = index;
}

static void robot_arm_mark_reach_target(float y, float z) {
    g_arm_reach_target_y = y;
    g_arm_reach_target_z = z;
    g_arm_reach_pending = 1;
}

static void robot_arm_check_target_reached(void) {
    if (!g_arm_reach_pending) {
        return;
    }

    float dy = g_robot_arm.current_y - g_arm_reach_target_y;
    float dz = g_robot_arm.current_z - g_arm_reach_target_z;

    if (fabsf(dy) <= ARM_REACH_POS_TOL_MM &&
        fabsf(dz) <= ARM_REACH_POS_TOL_MM) {
        g_arm_reach_pending = 0;
        if (g_robot_arm.status == ARM_STATE_CATCH) {
            pump_set_state(1);

#if ARM_USE_PUMP_ADC_CHECK
            g_pump_wait_state = PUMP_WAIT_CATCH;
            return;

#else
            control_dispatch_publish(1);
            return;
#endif
        }
        if (g_robot_arm.status == ARM_STATE_PLACE) {
            pump_set_state(0);

#if ARM_USE_PUMP_ADC_CHECK
            g_pump_wait_state = PUMP_WAIT_PLACE;
            return;

#else
            control_dispatch_publish(1);
            return;
#endif
        }
        control_dispatch_publish(1);
    }
}

static void pump_check_ready(void) {
#if !ARM_USE_PUMP_ADC_CHECK
    (void)g_pump_wait_state;
    return;
#endif

    if (g_pump_wait_state == PUMP_WAIT_NONE) {
        return;
    }

    uint16_t adc_value = 0;
    if (!pump_read_adc(&adc_value)) {
        return;
    }

    if (g_pump_wait_state == PUMP_WAIT_CATCH) {
        if (adc_value > PUMP_ADC_READY_HIGH) {
            control_dispatch_publish(1);
            g_pump_wait_state = PUMP_WAIT_NONE;
        }
        return;
    }

    if (g_pump_wait_state == PUMP_WAIT_PLACE) {
        if (adc_value < PUMP_ADC_READY_LOW) {
            control_dispatch_publish(1);
            g_pump_wait_state = PUMP_WAIT_NONE;
        }
    }
}

static void pump_set_state(uint8_t on) {
    HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint8_t pump_read_adc(uint16_t *out_value) {
    if (out_value == NULL) {
        return 0;
    }

    pump_adc_init_once();

    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = PUMP_ADC_CHANNEL;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
        return 0;
    }

    if (HAL_ADC_Start(&hadc3) != HAL_OK) {
        return 0;
    }
    if (HAL_ADC_PollForConversion(&hadc3, 2) != HAL_OK) {
        HAL_ADC_Stop(&hadc3);
        return 0;
    }
    *out_value = (uint16_t)HAL_ADC_GetValue(&hadc3);
    HAL_ADC_Stop(&hadc3);
    return 1;
}

static void pump_adc_init_once(void) {
    if (g_pump_adc_inited) {
        return;
    }

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitStruct.Pin = PUMP_ADC_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(PUMP_ADC_GPIO_PORT, &GPIO_InitStruct);
    g_pump_adc_inited = 1;
}

#if ARM_USE_REMOTE_KEY
static void arm_remote_state_switch(uint8_t key, remote_key_event_t event) {
    UNUSED(event);

    if (key < ARM_SWITCH_KEY ||
        key >= (uint8_t)(ARM_SWITCH_KEY + ARM_REMOTE_KEY_COUNT)) {
        return;
    }

    robot_arm_set_state_index((uint8_t)(key - ARM_SWITCH_KEY));
}
#endif