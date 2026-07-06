/**
 * @file arm_ctrl.c
 * @author xinglu
 * @brief 机械臂控制 (应用层接口与状态管理)
 * @version 3.1
 * @date 2026-05-17
 */

#include "includes.h"
#include "arm_ctrl.h"
#include "microros_ctrl.h"
#include "adc.h"

static void robot_arm_task(void *pvParameters);
static void arm_feedback_task(void *pvParameters);
static float arm_ctrl_wrap_pi(float angle);
static void robot_arm_mark_reach_target(float y, float z, float pitch);
static void arm_pump_place_check(bool publish_result);
static void arm_pump_catch_check(void);
static bool arm_is_motor_reached(void);

static uint8_t pump_read_adc(uint16_t *out_value);
static uint8_t pump_read_adc_filtered(uint16_t *out_value);

#if ARM_USE_REMOTE_KEY
static void arm_remote_state_switch(uint8_t key, remote_key_event_t event);
#endif

static RobotArm g_robot_arm;                  /* 机械臂控制对象 */
static uint8_t g_arm_target_index;            /* 当前目标状态索引 (0~8) */
static TaskHandle_t g_robot_arm_task_handle;  /* 机械臂任务句柄 */
static TaskHandle_t arm_feedback_task_handle; /* 机械臂反馈任务句柄 */
static uint8_t g_last_target_index;           /* 上一次下发的目标状态索引 */
static uint8_t g_place_return_sequence_active = 0; /* 放置后回位组合动作标志 */
static uint8_t g_place_return_target_index = 0; /* 放置完成后跳转的目标状态 */

static uint8_t g_place_target_index = 0;          /* 放置层级索引 (0~2), 初始中层 */
static uint8_t g_wait_takeout_target_index = ARM_TAKEOUT_START_LAYER;   /* 待取出层级索引 (0~2) */
static arm_target_point_t g_dynamic_target = {0}; /* 动态抓取目标点 (mm/rad) */
static uint8_t g_has_dynamic_target = 0;          /* 是否存在动态抓取目标 */

static float g_arm_reach_target_joint[3] = {0.0f, 0.0f,
                                            0.0f}; /* 到位判定的关节角目标 */
static pump_wait_state_t g_pump_wait_state = PUMP_WAIT_NONE; /* 气泵等待状态 */
static uint8_t g_last_switch_key = 0xFF; /* 上一次遥控器切换键值 */

static float g_pump_retry_offset_y = 0.0f; /* 气泵重试Y轴累积偏移量 (mm) */

bool arm_return_enabel = false; /* 放置完成后回位功能使能标志 */

/* ---------------- 预设目标点位 ---------------- */

static const arm_target_point_t g_arm_target_points[11] = {
    {139.95f + 20.0f + 50.0f, 102.70f + 30.0f, 0.6955f}, /* 0: INIT */
    {200.000f, 10.0f, 0.0f},                             /* 1: READY_1 */
    {420.000f, -50.0f, CATCH_READY_2_ANGEL},             /* 2: READY_2 */
    //{570.000f, 180.0f, 0.08f},                         /* 3: READY_3 */
    {270.000f, 180.0f, 0.08f},                           /* 3: READY_3 */
    {250.0f, -230.f, 0.0f},                              /* 4: READY_4 */
    {513.142f, 200.0f, 0.0f},                            /* 5: CATCH */
    {-275.12f, 493.991f, -PI / 2.0},                     /* 6: PLACE */
    {533.142f, 300.0f, 0.0f},                            /* 7: WAIT_TAKEOUT */
    {410.000f, 830.0f, PI / 12.0},                       /* 8: TAKEOUT_1 */
    {740.0f, 670.0f, 0.0f},                              /* 9: TAKEOUT_2  */
    {170.0f, 900.0f, PI * 0.75 + 0.1},                   /* 10: OVERLOOK */
};

static const arm_target_point_t g_arm_place_points[3] = {
    {-330.0f, 380.0f, -PI / 2},                           /* 0: 低层 */
    {-265.12f + 175.0f, FIRST_POINT_Z_LOW + 175.0f, -PI}, /* 1: 中层 */
    // {-265.12f + 175.0f + 30.0f, FIRST_POINT_Z_LOW + 175.0f + 350.0f,
    //  -PI}, /* 2: 高层 */
    {400.0f, 500.0f, -PI / 2}, /* 2: 高层 */
}; /* 放置层级点位 */

static const arm_target_point_t g_arm_wait_takeout_points[3] = {
    {-360.0f, FIRST_POINT_Z_LOW, -PI / 2},                        /* 0: 低层 */
    {-265.12f + 165.0f, FIRST_POINT_Z_LOW + 175.0f - 15.0f, -PI}, /* 1: 中层 */
    // {-265.12f + 165.0f + 20.0f, FIRST_POINT_Z_LOW + 175.0f + 350.0f - 60.0f,
    //  -PI}, /* 2: 高层 */
    {400.0f, 500.0f, -PI / 2}, /* 2: 高层 */
}; /* 待取出层级点位 */

/* ======================== 【配置与外部接口模块】 ============================ */
#pragma region API_Config
/** @defgroup API_Config 系统初始化、动态目标配置及层级设置接口 */
/** @{ */

/**
 * @brief 状态索引转换为应用层枚举
 * @param index 状态索引 (0~10)
 * @return 对应的机械臂状态枚举值
 */
static arm_status_t arm_status_from_index(uint8_t index) {
    switch (index) {
        case 0:
            return ARM_STATE_INIT;
        case 1:
            return ARM_STATE_READY_1;
        case 2:
            return ARM_STATE_READY_2;
        case 3:
            return ARM_STATE_READY_3;
        case 4:
            return ARM_STATE_READY_4;
        case 5:
            return ARM_STATE_CATCH;
        case 6:
            return ARM_STATE_PLACE;
        case 7:
            return ARM_STATE_WAIT_TAKEOUT;
        case 8:
            return ARM_STATE_TAKEOUT_1;
        case 9:
            return ARM_STATE_TAKEOUT_2;
        case 10:
            return ARM_STATE_OVERLOOK;
        case 11:
            return ARM_STATE_CLOSE_PUMP;
        default:
            return ARM_STATE_INIT;
    }
}

/**
 * @brief 注册动态抓取上层目标 (由 MicroROS 调用)
 * @param x 动态 X 坐标 (m，当前未参与目标计算)
 * @param y 动态 Y 坐标 (m)
 * @param z 动态 Z 坐标 (m)
 */
void robot_arm_set_dynamic_catch_target_up(float y, float x, float z) {
    /* 将米单位转换为毫米，并校准摄像头与吸盘中心的偏移补偿 */

    g_dynamic_target.y = y * 1000.0f + 200.0f - CAM_TO_CAT_Y_OFFSET + 20.0f;
    g_dynamic_target.z = z * 1000.0f + 10.0f + CAM_TO_CAT_Z_OFFSET + 30.0f;

    g_has_dynamic_target = 1;
    (void)x; //x不使用，仅用于底盘校准，与机械臂校准无关
}

/**
 * @brief 设置动态抓取上层目标 
 * @param y 动态 Y 坐标 (m)
 * @param x 动态 X 坐标 (m，当前未参与目标计算)
 * @param z 动态 Z 坐标 (m)
 */
void robot_arm_set_dynamic_catch_target_up2(float y, float x, float z) {

    // g_dynamic_target.y = y * 1000.0f + 570.0f - CAM_TO_CAT_Y_OFFSET + 20.0f;
    // g_dynamic_target.z = z * 1000.0f + 180.0f + CAM_TO_CAT_Z_OFFSET;

    g_dynamic_target.y = y * 1000.0f + 270.0f - CAM_TO_CAT_Y_OFFSET + 20.0f;
    g_dynamic_target.z = z * 1000.0f + 130.0f + CAM_TO_CAT_Z_OFFSET;

    g_has_dynamic_target = 1;
    (void)x; //x不使用，仅用于底盘校准，与机械臂校准无关
}

/**
 * @brief 
 * @param x 动态 X 坐标(m，当前未参与目标计算) * @param y 动态 Y 坐标(m) *
 * @param z 动态 Z 坐标(m) * /
 */

void robot_arm_set_dynamic_catch_target_down(float y, float x, float z) {
    /* 将米单位转换为毫米，并校准摄像头与吸盘中心的偏移补偿 */
    float theta = CATCH_READY_2_ANGEL;

    float y_cam = y * cosf(theta) - z * sinf(theta);
    float z_cam = y * sinf(theta) + z * cosf(theta);

    /* 旋转到机械臂水平坐标系 */
    g_dynamic_target.y = y_cam * 1000.0f + 420.0f -
                         (CAM_TO_CAT_Y_OFFSET * cosf(theta) +
                          CAM_TO_CAT_Z_OFFSET * sinf(theta)) +
                         20.0f - 2.0f;
    g_dynamic_target.z = z_cam * 1000.0f - 50.0f +
                         (CAM_TO_CAT_Y_OFFSET * sinf(theta) +
                          CAM_TO_CAT_Z_OFFSET * cosf(theta)) +
                         30.0f;
    g_has_dynamic_target = 1;

    (void)x; //x不使用，仅用于底盘校准，与机械臂校准无关
}

/**
 * @brief 注册动态抓取下层目标 (由 MicroROS 调用)
 * @param x 动态 X 坐标 (m，当前未参与目标计算)
 * @param y 动态 Y 坐标 (m)
 * @param z 动态 Z 坐标 (m)
 */
void robot_arm_set_dynamic_catch_target_down2(float y, float x, float z) {
    /* 将米单位转换为毫米，并校准摄像头与吸盘中心的偏移补偿 */
    g_dynamic_target.y = y * 1000.0f + 250.0f - CAM_TO_CAT_Y_OFFSET + 20.0f;
    g_dynamic_target.z = z * 1000.0f - 230.0f + CAM_TO_CAT_Z_OFFSET;

    g_has_dynamic_target = 1;
    (void)x; //x不使用，仅用于底盘校准，与机械臂校准无关
}

/**
 * @brief 手动设置放置层级索引 (临时覆盖)
 * @param place_idx 放置层级索引 (0~2)
 */
void robot_arm_set_place_index(uint8_t place_idx) {
    if (place_idx < 3) {
        g_place_target_index = place_idx;
    }
}

/**
 * @brief 手动设置待取出层级索引 (临时覆盖)
 * @param takeout_idx 待取出层级索引 (0~2)
 */
void robot_arm_set_wait_takeout_index(uint8_t takeout_idx) {
    if (takeout_idx < 3) {
        g_wait_takeout_target_index = takeout_idx;
    }
}

/**
 * @brief 初始化机械臂应用层与 RTOS 任务
 */
void robot_arm_init(void) {
    /* 初始化机械臂控制器与控制周期 */
    robot_arm_system_init(&g_robot_arm);
    robot_arm_set_ctrl_dt(&g_robot_arm, (float)ARM_TASK_PERIOD_MS * 0.001f);

    /* 复位应用层状态 */
    g_arm_target_index = 0;
    g_last_target_index = 0;
    g_place_return_sequence_active = 0;
    g_place_return_target_index = 0;
    g_place_target_index = 1;
    g_wait_takeout_target_index = 1;
    g_has_dynamic_target = 0;
    g_pump_wait_state = PUMP_WAIT_NONE;
    g_last_switch_key = 0xFF;

    /* 初始关闭气泵并下发初始点 */
    pump_set_state(0);
    robot_arm_apply_target(g_arm_target_index);

#if ARM_USE_REMOTE_KEY
    remote_register_key_callback(10, REMOTE_KEY_PRESS_UP, arm_remote_state_switch); /* INIT        */
    remote_register_key_callback(11, REMOTE_KEY_PRESS_UP, arm_remote_state_switch); /* READY_4     */
    remote_register_key_callback(12, REMOTE_KEY_PRESS_UP, arm_remote_state_switch); /* CATCH       */
    remote_register_key_callback(13, REMOTE_KEY_PRESS_UP, arm_remote_state_switch); /* PLACE       */
    remote_register_key_callback(14, REMOTE_KEY_PRESS_UP, arm_remote_state_switch); /* WAIT_TAKEOUT*/
    remote_register_key_callback(15, REMOTE_KEY_PRESS_UP, arm_remote_state_switch); /* TAKEOUT_1   */
    remote_register_key_callback(16, REMOTE_KEY_PRESS_UP, arm_remote_state_switch); /* TAKEOUT_2   */
#endif

    /* 启动机械臂控制任务 */
    xTaskCreate(robot_arm_task, "arm_ctrl_task", 512, NULL, 3,
                &g_robot_arm_task_handle);
    xTaskCreate(arm_feedback_task, "arm_feedback_task", 256, NULL, 3,
                &arm_feedback_task_handle);
    if (arm_feedback_task_handle == NULL) {
        log_message(LOG_ERROR, "arm_feedback_task creation failed!");
    }
}

/** @} */
#pragma endregion

/* ======================== 【RTOS 任务执行模块】 ============================ */
#pragma region RTOS_Task
/** @defgroup RTOS_Task 机械臂核心轮询任务与事件驱动反馈任务 */
/** @{ */

// uint16_t test_adc_value = 0; /* 气泵 ADC 测试值 */

/**
 * @brief 机械臂控制任务主循环
 * @param pvParameters RTOS 任务参数 (未使用)
 */
static void robot_arm_task(void *pvParameters) {
    UNUSED(pvParameters);
    while (1) {
        /* 周期更新控制、到位检测与气泵状态 */
        robot_arm_update(&g_robot_arm);
        // pump_set_state(1); //气泵测试
        // pump_read_adc_filtered(&test_adc_value);
        vTaskDelay(pdMS_TO_TICKS(ARM_TASK_PERIOD_MS));
    }
}

/**
 * @brief 机械臂反馈检测任务,负责上报任务完成情况。
 */
static void arm_feedback_task(void *pvParameters) {
    UNUSED(pvParameters);
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        arm_status_t current_status = g_robot_arm.status;

        // 直接单纯关闭气泵，不进行后续的到位检测和状态检查
        if (current_status == ARM_STATE_CLOSE_PUMP) {
            arm_pump_place_check(1);
            continue;
        }

        /* 阻塞等待电机物理到位 */
        while (!arm_is_motor_reached()) {
            if (g_robot_arm.status != current_status) {
                break; // 运动中途收到新命令，状态被打断，立刻跳出
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        if (g_robot_arm.status != current_status) {
            continue; // 状态被打断，回到最顶层重新睡眠等待新通知
        }

        g_robot_arm.arm_motion_active = 0;

        /* 电机到位后，根据不同业务状态执行特定检测 */
        switch (current_status) {
            case ARM_STATE_CATCH:
            case ARM_STATE_WAIT_TAKEOUT:
                arm_pump_catch_check();
                break;

            case ARM_STATE_PLACE:
                arm_pump_place_check(arm_return_enabel);
                if (g_robot_arm.status == current_status &&
                    g_place_return_sequence_active) {
                    uint8_t return_index = g_place_return_target_index;
                    robot_arm_set_state_index(return_index);
                }
                break;
            default:
                if (g_place_return_sequence_active == 1) {
                    g_place_return_sequence_active = 0;
                    g_place_return_target_index = 0;
                    if (arm_return_enabel) {
                        control_dispatch_publish(1); // 成功抓取
                    }

                } else {
                    if (arm_return_enabel) {
                        control_dispatch_publish(1); // 成功抓取
                    }
                }
                break;
        }
    }
}

/** @} */
#pragma endregion

/* ======================== 【目标状态下发模块】 ============================ */
#pragma region Target_Dispatch
/** @defgroup Target_Dispatch 目标点位解析、合并与序列下发 */
/** @{ */

/**
 * @brief 设置机械臂当前目标状态索引
 * @param index 状态索引 (0~11)
 */
void robot_arm_set_state_index(uint8_t index) {
    if (index > 11) {
        return;
    }
    g_arm_target_index = index;
    robot_arm_apply_target(index);
}

/**
 * @brief 启动放置后自动回位/识别的组合动作
 * @param return_index 放置完成后跳转的状态索引
 */
void robot_arm_start_place_return_sequence(uint8_t return_index) {
    if (return_index > 11) {
        return;
    }
    g_place_return_sequence_active = 1;
    g_place_return_target_index = return_index;
    g_arm_target_index = 6;
    robot_arm_apply_target(6);
}

/**
 * @brief 应用/下发预设目标点位
 * @param index 状态索引
 */
void robot_arm_apply_target(uint8_t index) {
    arm_status_t prev_status = g_robot_arm.status;

    /* 放置层级循环递增 */
    if (prev_status == ARM_STATE_PLACE && index != ARM_STATE_PLACE) {
        g_place_target_index = (g_place_target_index + 1) % 3;
    }

    g_pump_wait_state = PUMP_WAIT_NONE;
    g_robot_arm.status = arm_status_from_index(index);
    g_robot_arm.last_status = prev_status;

    /* 离开抓取状态时清除重试偏移 */
    if (prev_status == ARM_STATE_CATCH &&
        g_robot_arm.status != ARM_STATE_CATCH) {
        g_pump_retry_offset_y = 0.0f;
    }

    if (g_robot_arm.status == ARM_STATE_CLOSE_PUMP) {
        xTaskNotifyGive(arm_feedback_task_handle);
        return;
    }

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
        target_pitch =
            g_arm_wait_takeout_points[g_wait_takeout_target_index].pitch;
        g_robot_arm.takeout_layer = g_wait_takeout_target_index;
    }

    /**
     * TAKEOUT_1/TAKEOUT_2 阶段式顺序步进（防碰撞）：
     *   1. 大臂先抬起（小臂、吸盘锁定）
     *   2. 小臂运动（大臂、吸盘锁定）
     *   3. 三关节协同到达最终目标
     * 从 READY 态切换时不执行避障序列，直接运动到位。
     */
    if (arm_is_takeout_state(g_robot_arm.status) &&
        !arm_is_ready_state(prev_status)) {

        /* 设定取出层数（独立于 place_layer），并逐层递减 */
        g_robot_arm.takeout_layer = g_wait_takeout_target_index;
        g_wait_takeout_target_index = (g_wait_takeout_target_index == 0)
                                          ? 2
                                          : (g_wait_takeout_target_index - 1);

        float wait_takeout_suction_angle = g_arm_reach_target_joint[2];
        robot_arm_start_takeout_sequence(&g_robot_arm, target_y, target_z,
                                         target_pitch,
                                         wait_takeout_suction_angle);
        robot_arm_mark_reach_target(target_y, target_z, target_pitch);
        g_last_target_index = index;
        xTaskNotifyGive(arm_feedback_task_handle);
        return;
    }

    robot_arm_mark_reach_target(target_y, target_z, target_pitch);
    robot_arm_set_target(&g_robot_arm, target_y, target_z, target_pitch);
    g_last_target_index = index;

    xTaskNotifyGive(arm_feedback_task_handle);
}

/** @} */
#pragma endregion

/* ======================== 【底层运算与状态判定模块】 ============================ */
#pragma region Math_And_State
/** @defgroup Math_And_State 角度归一化、运动学计算记录与电机物理到位判定 */
/** @{ */

/**
 * @brief 将角度限制到 [-PI, PI]
 * @param angle 输入角度 (rad)
 * @return 归一化角度 (rad)
 */
static float arm_ctrl_wrap_pi(float angle) {
    while (angle > PI) {
        angle -= 2.0f * PI;
    }
    while (angle < -PI) {
        angle += 2.0f * PI;
    }
    return angle;
}

/**
 * @brief 将角度限制到 [-2PI, 2PI]
 * @param angle 输入角度 (rad)
 * @return 归一化角度 (rad)
 */
static float arm_ctrl_wrap_2pi(float angle) {
    while (angle > 2.0f * PI) {
        angle -= 2.0f * PI;
    }
    while (angle < -2.0f * PI) {
        angle += 2.0f * PI;
    }
    return angle;
}

/**
 * @brief 记录本次目标点对应的关节角，用于到位判定
 * @param y 目标 Y 坐标 (mm)
 * @param z 目标 Z 坐标 (mm)
 * @param pitch 末端姿态角 (rad)
 */
static void robot_arm_mark_reach_target(float y, float z, float pitch) {
    float joint_angles[3];
    /* 逆解得到关节角目标值 */
    arm_pos_angle(y, z, pitch, joint_angles);

    g_arm_reach_target_joint[0] = joint_angles[0];
    g_arm_reach_target_joint[1] = joint_angles[1];
    g_arm_reach_target_joint[2] = joint_angles[2];
    // g_arm_reach_pending = 1;
}

/**
 * @brief 判定电机是否已到达目标角度
 * @return true: 已到位 / false: 运动中
 */
static bool arm_is_motor_reached(void) {
    if (robot_arm_is_takeout_sequence_active(&g_robot_arm)) {
        return false;
    }

    float err_j1 = arm_ctrl_wrap_pi(g_robot_arm.damiao_1.position -
                                    g_arm_reach_target_joint[0]);
    float err_j3 = arm_ctrl_wrap_2pi(g_robot_arm.damiao_3.position -
                                     g_arm_reach_target_joint[1]);
    float err_j4 = arm_ctrl_wrap_pi(g_robot_arm.damiao_4.position -
                                    g_arm_reach_target_joint[2]);

    return (fabsf(err_j1) <= ARM_REACH_JOINT_TOL_RAD) &&
           (fabsf(err_j3) <= ARM_REACH_JOINT_TOL_RAD) &&
           (fabsf(err_j4) <= ARM_REACH_JOINT_TOL_RAD);
}

/** @} */
#pragma endregion

/* ======================== 【末端执行器与气压模块】 ============================ */
#pragma region End_Effector
/** @defgroup End_Effector 气泵硬件驱动、ADC气压读取与业务反馈校验 */
/** @{ */

/**
 * @brief 执行抓取状态下的气压检测与重试逻辑
 */
static void arm_pump_catch_check(void) {
    pump_set_state(1); // 打开气泵
    uint32_t wait_start_tick = HAL_GetTick();
    uint8_t retry_count = 0;
    float retry_offset_y = 0.0f;
    float retry_offset_z = 0.0f;

    /* 只要状态没被外部打断，就一直循环检测 */
    while (g_robot_arm.status == ARM_STATE_CATCH ||
           g_robot_arm.status == ARM_STATE_WAIT_TAKEOUT) {
        uint16_t adc_val = 0;

#if ARM_USE_PUMP_ADC_CHECK
        if (pump_read_adc_filtered(&adc_val)) {
            if (adc_val < PUMP_ADC_READY_LOW) {
                log_message(LOG_INFO, "ARM_CATCH Success, adc=%d", adc_val);
                if (arm_return_enabel) {
                    control_dispatch_publish(1); // 成功抓取
                }
                return;
            }
        }
#else
        // 未开启 ADC 检测，延时默认成功
        if (HAL_GetTick() - wait_start_tick >= 2000U) {
            if (arm_return_enabel) {
                control_dispatch_publish(1); // 成功抓取
            }
            return;
        }
#endif

        /* 超时与推进重试逻辑 */
        if (HAL_GetTick() - wait_start_tick >= 1000U) {
#if ARM_USE_PUMP_ADC_CHECK

            if (retry_count < 10) {
                if (g_wait_takeout_target_index != 0) {
                    if (g_robot_arm.status == ARM_STATE_WAIT_TAKEOUT) {
                        retry_offset_y -= 10.0f;
                    } else if (g_robot_arm.status == ARM_STATE_CATCH) {
                        retry_offset_y += 10.0f;
                    }
                    float new_y = g_robot_arm.final_target_y + retry_offset_y;

                    // 重新设定推进目标
                    robot_arm_mark_reach_target(new_y,
                                                g_robot_arm.final_target_z,
                                                g_robot_arm.final_target_pitch);
                    g_robot_arm.arm_target_y = new_y;
                } else if (g_wait_takeout_target_index == 0) {
                    retry_offset_z -= 10.0f;
                    float new_z = g_robot_arm.final_target_z + retry_offset_z;

                    // 重新设定推进目标
                    robot_arm_mark_reach_target(g_robot_arm.final_target_y,
                                                new_z,
                                                g_robot_arm.final_target_pitch);
                    g_robot_arm.arm_target_z = new_z;
                }

                g_robot_arm.motion_state = ARM_MOTION_STATE_DIRECT_MOVE;

                // 等待这次微调推进到位
                while (!arm_is_motor_reached() &&
                       g_robot_arm.status == ARM_STATE_CATCH) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }

                wait_start_tick = HAL_GetTick(); // 重置 2 秒计时器
                retry_count++;
                continue;
            }
#endif
            // 重试耗尽，抓取失败
            if (arm_return_enabel) {
                control_dispatch_publish(1); // 成功抓取
            }
            return;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief 执行放置状态下的压力释放检测
 */
static void arm_pump_place_check(bool publish_result) {
    if (g_place_target_index == 2 && g_robot_arm.status == ARM_STATE_PLACE) {
        if (publish_result) {
            control_dispatch_publish(1); // 第三层不关气泵
        }
        return;
    }
    pump_set_state(0); // 关闭气泵
    uint32_t wait_start_tick = HAL_GetTick();

    while (1) {
        uint16_t adc_val = 0;

#if ARM_USE_PUMP_ADC_CHECK
        if (pump_read_adc_filtered(&adc_val)) {
            if (adc_val > PUMP_ADC_READY_HIGH) {
                log_message(LOG_INFO, "ARM_PLACE Success, adc=%d", adc_val);
                if (publish_result) {
                    control_dispatch_publish(1);
                }
                return;
            }
        }
#endif
        if (HAL_GetTick() - wait_start_tick >= 2000U) {
            if (publish_result) {
                control_dispatch_publish(1); // 超时也认为放置完成
            }
            return;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief 设置气泵开关状态
 * @param on 1: 打开气泵, 0: 关闭气泵
 */
void pump_set_state(uint8_t on) {
    HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief 读取气泵压力 ADC 值
 * @param out_value 输出 ADC 结果指针
 * @return 1: 读取成功, 0: 读取失败
 */
static uint8_t pump_read_adc(uint16_t *out_value) {
    if (out_value == NULL) {
        return 0;
    }

    if (HAL_ADC_Start(&hadc1) != HAL_OK) {
        return 0;
    }

    if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK) {
        HAL_ADC_Stop(&hadc1);
        return 0;
    }

    *out_value = (uint16_t)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    return 1;
}

#define ADC_SAMPLE_COUNT 50

static uint8_t pump_read_adc_filtered(uint16_t *out_value) {
    uint32_t sum = 0;
    uint16_t samples[ADC_SAMPLE_COUNT];

    for (int i = 0; i < ADC_SAMPLE_COUNT; i++) {
        if (!pump_read_adc(&samples[i])) {
            return 0;
        }
        sum += samples[i];
    }
    *out_value = (uint16_t)(sum / ADC_SAMPLE_COUNT);
    return 1;
}

/** @} */
#pragma endregion

#if ARM_USE_REMOTE_KEY
/* ======================== 【遥控器交互模块】 ============================ */
#pragma region Remote_Control
/** @defgroup Remote_Control 物理遥控器按键状态机调度 */
/** @{ */

/* 按键 → 状态索引 映射表，按需增删改 */
static const uint8_t g_arm_key_index_map[][2] = {
    {10, 0},  /* INIT        */
    {11, 4},  /* READY_1     */
    {12, 5},  /* CATCH       */
    {13, 6},  /* PLACE       */
    {14, 7},  /* WAIT_TAKEOUT*/
    {15, 8},  /* TAKEOUT_1   */
    {16, 11},  /* TAKEOUT_2   */
};
#define ARM_KEY_MAP_COUNT \
    (sizeof(g_arm_key_index_map) / sizeof(g_arm_key_index_map[0]))

/**
 * @brief 遥控器按键切换状态回调
 * @param key 按键值
 * @param event 按键事件类型
 */
static void arm_remote_state_switch(uint8_t key, remote_key_event_t event) {
    UNUSED(event);
    for (uint8_t i = 0; i < ARM_KEY_MAP_COUNT; i++) {
        if (g_arm_key_index_map[i][0] == key) {
            if (key == g_last_switch_key) {
                return;
            }
            g_last_switch_key = key;
            robot_arm_set_state_index(g_arm_key_index_map[i][1]);
            return;
        }
    }
}

/** @} */
#pragma endregion
#endif
