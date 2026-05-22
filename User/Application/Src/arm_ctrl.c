/**
 * @file arm_ctrl.c
 * @author xinglu
 * @brief 机械臂控制 (应用层接口与状态管理)
 * @version 3.0
 * @date 2026-05-15
 */

#include "includes.h"
#include "arm_ctrl.h"
#include "microros_ctrl.h"
#include "adc.h"

static void robot_arm_task(void *pvParameters);
static float arm_ctrl_wrap_pi(float angle);
static void robot_arm_mark_reach_target(float y, float z, float pitch);
static void robot_arm_check_target_reached(void);
static void pump_check_ready(void);
static void pump_set_state(uint8_t on);
static uint8_t pump_read_adc(uint16_t *out_value);

#if ARM_USE_REMOTE_KEY
static void arm_remote_state_switch(uint8_t key, remote_key_event_t event);
#endif

static RobotArm g_robot_arm;                               /* 机械臂控制对象 */
static uint8_t g_arm_target_index;                         /* 当前目标状态索引 (0~5) */
static TaskHandle_t g_robot_arm_task_handle;               /* 机械臂任务句柄 */
static uint8_t g_last_target_index;                        /* 上一次下发的目标状态索引 */

static uint8_t g_place_target_index = 0;                   /* 放置层级索引 (0~2) */
static uint8_t g_wait_takeout_target_index = 2;            /* 待取出层级索引 (0~2) */
static arm_target_point_t g_dynamic_target = {0};          /* 动态抓取目标点 (mm/rad) */
static uint8_t g_has_dynamic_target = 0;                   /* 是否存在动态抓取目标 */

static float g_arm_reach_target_joint[2] = {0.0f, 0.0f};    /* 到位判定的关节角目标 */
static uint8_t g_arm_reach_pending = 0;                    /* 是否等待到位判定 */
static pump_wait_state_t g_pump_wait_state = PUMP_WAIT_NONE; /* 气泵等待状态 */
static uint8_t g_last_switch_key = 0xFF;                   /* 上一次遥控器切换键值 */

/* ---------------- 预设目标点位 ---------------- */

static const arm_target_point_t g_arm_target_points[6] = {
    {139.95f + 20.0f + 50.0f, 102.70f + 30.0f, 0.6955f},  /* 0: INIT */
    {200.000f, 10.0f, 0.0f},              /* 1: READY */
    {313.142f, 200.0f, 0.0f},             /* 2: CATCH */
    {-275.12f, 493.991f, -PI/2.0},         /* 3: PLACE */
    {533.142f, 300.0f, 0.0f},              /* 4: WAIT_TAKEOUT */
    {533.142f, 300.0f, 0.0f},              /* 5: TAKEOUT */
}; /* 预设状态点位 (0~5: INIT/READY/CATCH/PLACE/WAIT_TAKEOUT/TAKEOUT) */

static const arm_target_point_t g_arm_place_points[3] = {
    {-330.0f, 380.0f, -PI/2},                                              /* 0: 低层 */
    {-265.12f + 175.0f, FIRST_POINT_Z_LOW + 175.0f, -PI},                  /* 1: 中层 */
    {-265.12f + 175.0f + 30.0f, FIRST_POINT_Z_LOW + 175.0f + 350.0f, -PI}, /* 2: 高层 */
}; /* 放置层级点位 */

static const arm_target_point_t g_arm_wait_takeout_points[3] = {
    {-360.0f, FIRST_POINT_Z_LOW, -PI/2},                                                 /* 0: 低层 */
    {-265.12f + 165.0f, FIRST_POINT_Z_LOW + 175.0f - 20.0f , -PI},                       /* 1: 中层 */
    {-265.12f + 165.0f + 20.0f, FIRST_POINT_Z_LOW + 175.0f + 350.0f - 60.0f , -PI},      /* 2: 高层 */
}; /* 待取出层级点位 */

/* ---------------- 应用层实现 ---------------- */

/**
 * @brief 状态索引转换为应用层枚举
 * @param index 状态索引 (0~5)
 * @return 对应的机械臂状态枚举值
 */
static arm_status_t arm_status_from_index(uint8_t index)
{
    switch (index) {
        case 0: return ARM_STATE_INIT;
        case 1: return ARM_STATE_READY;
        case 2: return ARM_STATE_CATCH;
        case 3: return ARM_STATE_PLACE;
        case 4: return ARM_STATE_WAIT_TAKEOUT;
        case 5: return ARM_STATE_TAKEOUT;
        default: return ARM_STATE_INIT;
    }
}

/**
 * @brief 注册动态抓取目标 (由 MicroROS 调用)
 * @param x 动态 X 坐标 (m，当前未参与目标计算)
 * @param y 动态 Y 坐标 (m)
 * @param z 动态 Z 坐标 (m)
 */
void robot_arm_set_dynamic_catch_target(float x, float y, float z)
{
    /* 将米单位转换为毫米，并按标定偏移补偿 */
    g_dynamic_target.y = y * 1000.0f + 200.0f - 55.0f;
    g_dynamic_target.z = z * 1000.0f + 10.0f + 90.0f;
    g_has_dynamic_target = 1;
    robot_arm_set_state_index((uint8_t)ARM_STATE_CATCH);
    (void)x;
}

/**
 * @brief 手动设置放置层级索引 (临时覆盖)
 * @param place_idx 放置层级索引 (0~2)
 */
void robot_arm_set_place_index(uint8_t place_idx)
{
    if (place_idx < 3) {
        g_place_target_index = place_idx;
    }
}

/**
 * @brief 手动设置待取出层级索引 (临时覆盖)
 * @param takeout_idx 待取出层级索引 (0~2)
 */
void robot_arm_set_wait_takeout_index(uint8_t takeout_idx)
{
    if (takeout_idx < 3) {
        g_wait_takeout_target_index = takeout_idx;
    }
}

/**
 * @brief 初始化机械臂应用层与 RTOS 任务
 */
void robot_arm_init(void)
{
    /* 初始化机械臂控制器与控制周期 */
    robot_arm_system_init(&g_robot_arm);
    robot_arm_set_ctrl_dt(&g_robot_arm, (float)ARM_TASK_PERIOD_MS * 0.001f);

    /* 复位应用层状态 */
    g_arm_target_index = 0;
    g_last_target_index = 0;
    g_place_target_index = 0;
    g_wait_takeout_target_index = 2;
    g_has_dynamic_target = 0;
    g_pump_wait_state = PUMP_WAIT_NONE;
    g_last_switch_key = 0xFF;

    /* 初始关闭气泵并下发初始点 */
    pump_set_state(0);
    robot_arm_apply_target(g_arm_target_index);

#if ARM_USE_REMOTE_KEY
    for (uint8_t i = 0; i < ARM_REMOTE_KEY_COUNT; ++i) {
        remote_register_key_callback((uint8_t)(ARM_SWITCH_KEY + i),
                                     REMOTE_KEY_PRESS_UP,
                                     arm_remote_state_switch);
    }
#endif

    /* 启动机械臂控制任务 */
    xTaskCreate(robot_arm_task, "arm_ctrl_task", 512, NULL, 3, &g_robot_arm_task_handle);
}

/**
 * @brief 机械臂控制任务主循环
 * @param pvParameters RTOS 任务参数 (未使用)
 */
static void robot_arm_task(void *pvParameters)
{
    UNUSED(pvParameters);
    while (1) {
        /* 周期更新控制、到位检测与气泵状态 */
        robot_arm_update(&g_robot_arm);
        robot_arm_check_target_reached();
        pump_check_ready();
        vTaskDelay(pdMS_TO_TICKS(ARM_TASK_PERIOD_MS));
    }
}

/**
 * @brief 设置机械臂当前目标状态索引
 * @param index 状态索引 (0~5)
 */
void robot_arm_set_state_index(uint8_t index)
{
    if (index > 5) {
        return;
    }
    g_arm_target_index = index;
    robot_arm_apply_target(index);
}

/**
 * @brief 应用/下发预设目标点位
 * @param index 状态索引
 */
void robot_arm_apply_target(uint8_t index)
{
    arm_status_t prev_status = g_robot_arm.status;
    uint8_t current_takeout_idx = g_wait_takeout_target_index;

    /* 放置层级循环递增 */
    if (prev_status == ARM_STATE_PLACE && index != ARM_STATE_PLACE) {
        g_place_target_index = (g_place_target_index + 1) % 3;
    }

    /* 待取出层级递减 */
    if (prev_status == ARM_STATE_WAIT_TAKEOUT && index != ARM_STATE_WAIT_TAKEOUT) {
        g_wait_takeout_target_index = (g_wait_takeout_target_index == 0) ? 2 : (g_wait_takeout_target_index - 1);
    }

    g_pump_wait_state = PUMP_WAIT_NONE;
    g_robot_arm.status = arm_status_from_index(index);
    g_robot_arm.last_status = prev_status;

    float target_y = g_arm_target_points[index].y;
    float target_z = g_arm_target_points[index].z;
    float target_pitch = g_arm_target_points[index].pitch;

    /* 动态抓取目标覆盖 */
    if (g_robot_arm.status == ARM_STATE_CATCH && g_has_dynamic_target) {
        target_y = g_dynamic_target.y;
        target_z = g_dynamic_target.z;
        g_has_dynamic_target = 0;
    }

    /* 放置层级点位覆盖 */
    if (g_robot_arm.status == ARM_STATE_PLACE) {
        target_y = g_arm_place_points[g_place_target_index].y;
        target_z = g_arm_place_points[g_place_target_index].z;
        target_pitch = g_arm_place_points[g_place_target_index].pitch;
        g_robot_arm.place_layer = g_place_target_index;
    }

    /* 待取出层级点位覆盖 */
    if (g_robot_arm.status == ARM_STATE_WAIT_TAKEOUT) {
        target_y = g_arm_wait_takeout_points[g_wait_takeout_target_index].y;
        target_z = g_arm_wait_takeout_points[g_wait_takeout_target_index].z;
        target_pitch = g_arm_wait_takeout_points[g_wait_takeout_target_index].pitch;
    }

    /**
     * TAKEOUT 采用阶段式动作：需要沿用 WAIT_TAKEOUT 的吸盘角度
     * 使用扩展状态机避免动作分散在多个函数内
     */
    if (g_robot_arm.status == ARM_STATE_TAKEOUT &&
        prev_status == ARM_STATE_WAIT_TAKEOUT ) {

        float wait_takeout_suction_angle = g_arm_reach_target_joint[2];
        robot_arm_start_takeout_sequence(&g_robot_arm, target_y, target_z, target_pitch, wait_takeout_suction_angle);
        robot_arm_mark_reach_target(target_y, target_z, target_pitch);
        g_last_target_index = index;
        return;
    }

    robot_arm_mark_reach_target(target_y, target_z, target_pitch);
    robot_arm_set_target(&g_robot_arm, target_y, target_z, target_pitch);
    g_last_target_index = index;
}

/**
 * @brief 将角度限制到 [-PI, PI]
 * @param angle 输入角度 (rad)
 * @return 归一化角度 (rad)
 */
static float arm_ctrl_wrap_pi(float angle)
{
    while (angle > PI) angle -= 2.0f * PI;
    while (angle < -PI) angle += 2.0f * PI;
    return angle;
}

/**
 * @brief 记录本次目标点对应的关节角，用于到位判定
 * @param y 目标 Y 坐标 (mm)
 * @param z 目标 Z 坐标 (mm)
 * @param pitch 末端姿态角 (rad)
 */
static void robot_arm_mark_reach_target(float y, float z, float pitch)
{
    float joint_angles[3];
    /* 逆解得到关节角目标值 */
    arm_pos_angle(y, z, pitch, joint_angles);

    g_arm_reach_target_joint[0] = joint_angles[0];
    g_arm_reach_target_joint[1] = joint_angles[1];
    g_arm_reach_pending = 1;
}

/**
 * @brief 检查机械臂是否到达目标点，并触发后续动作
 */
static void robot_arm_check_target_reached(void)
{
    if (!g_arm_reach_pending) {
        return;
    }

    /* 关节角误差判定到位 */
    float err_j1 = arm_ctrl_wrap_pi(g_robot_arm.damiao_1.position - g_arm_reach_target_joint[0]);
    float err_j3 = arm_ctrl_wrap_pi(g_robot_arm.damiao_3.position - g_arm_reach_target_joint[1]);

    uint8_t big_small_reached = (fabsf(err_j1) <= ARM_REACH_JOINT_TOL_RAD) &&
                                 (fabsf(err_j3) <= ARM_REACH_JOINT_TOL_RAD);

    if (!big_small_reached) {
        return;
    }

    if (robot_arm_is_takeout_sequence_active(&g_robot_arm)) {
        return;
    }

    g_arm_reach_pending = 0;
    g_robot_arm.arm_motion_active = 0;

    if (g_robot_arm.status == ARM_STATE_CATCH) {
        /* 抓取到位：打开气泵并根据配置等待压力建立 */
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
        /* 放置到位：关闭气泵并根据配置等待压力释放 */
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

/**
 * @brief 检查气泵压力是否达到就绪条件
 */
static void pump_check_ready(void)
{
#if !ARM_USE_PUMP_ADC_CHECK
    (void)g_pump_wait_state;
    return;
#endif
    if (g_pump_wait_state == PUMP_WAIT_NONE) {
        return;
    }

    uint16_t adc_value = 0;
    /* 读取气泵压力 ADC */
    if (!pump_read_adc(&adc_value)) {
        return;
    }

    /* 根据等待状态判断抓取/释放完成 */
    if (g_pump_wait_state == PUMP_WAIT_CATCH && adc_value > PUMP_ADC_READY_HIGH) {
        control_dispatch_publish(1);
        g_pump_wait_state = PUMP_WAIT_NONE;
    }
    else if (g_pump_wait_state == PUMP_WAIT_PLACE && adc_value < PUMP_ADC_READY_LOW) {
        control_dispatch_publish(1);
        g_pump_wait_state = PUMP_WAIT_NONE;
    }
}

/**
 * @brief 设置气泵开关状态
 * @param on 1: 打开气泵, 0: 关闭气泵
 */
static void pump_set_state(uint8_t on)
{
    HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief 读取气泵压力 ADC 值
 * @param out_value 输出 ADC 结果指针
 * @return 1: 读取成功, 0: 读取失败
 */
static uint8_t pump_read_adc(uint16_t *out_value)
{
    if (out_value == NULL) {
        return 0;
    }

    if (HAL_ADC_Start(&hadc1) != HAL_OK) {
        return 0;
    }

    if (HAL_ADC_PollForConversion(&hadc1, 2) != HAL_OK) {
        HAL_ADC_Stop(&hadc1);
        return 0;
    }

    *out_value = (uint16_t)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    return 1;
}

#if ARM_USE_REMOTE_KEY
/**
 * @brief 遥控器按键切换状态回调
 * @param key 按键值
 * @param event 按键事件类型
 */
static void arm_remote_state_switch(uint8_t key, remote_key_event_t event)
{
    UNUSED(event);
    if (key >= ARM_SWITCH_KEY && key < (uint8_t)(ARM_SWITCH_KEY + ARM_REMOTE_KEY_COUNT)) {
        if (key == g_last_switch_key) {
            return;
        }
        g_last_switch_key = key;
        robot_arm_set_state_index((uint8_t)(key - ARM_SWITCH_KEY));
    }
}
#endif
