/**
 * @file    chassis.c
 * @author  Dominate0017
 * @brief   底盘控制任务
 * @version 1.0
 * @date    2026-05-02
 * ********************************************************************************
 *    Date    | Version |   Author    | Version Info
 * -----------+---------+-------------+----------------------------------------
 * 2026-05-02 |   1.0   | Dominate0017 | 代码重构,删除多余任务,自锁按键直接清空PID
 */
#include "includes.h"
#include "chassis_calculations/chassis_calculations.h"
#include "omni_wheels/omni_wheels.h"
#include "microros_ctrl.h"
#include "lift.h"

// #define SWITCH_WORLD_KEY   2 /* 切换世界/自身坐标系按键 */
#define SWITCH_AUTO_KEY    3 /* 切换自动/手动按键 */
#define SET_HALT_KEY       4 /* 切换自锁按键 */

#define CHASSIS_CAN_SELECT can1_selected

typedef enum {
    CHASSIS_MODE_MANUAL = 0,
    CHASSIS_MODE_AUTO,
} chassis_mode_t;

typedef struct {
    chassis_mode_t mode;
    chassis_speed_t chassis_speed;
    float g_current_rotor_degree[4];
    bool world_cordinate;
    bool halt;
} chassis_handle_t;

static chassis_handle_t chassis_handle = {
    .mode = CHASSIS_MODE_MANUAL,
    .chassis_speed =
        {
            .target_rpm = {0.0f, 0.0f, 0.0f, 0.0f},
        },
    .world_cordinate = false,
    .halt = false,
};

static dji_motor_handle_t dji_3508_handle[4];
static pid_t dji_3508_speed_pid[4];
static pid_t dji_3508_pos_pid[4];

static void chassis_bottom_init(void);
static void chassis_tasks_init(void);
static void chassis_switch_mode(uint8_t key, remote_key_event_t event);
static void chassis_set_halt(bool enable);
static void chassis_halt_degree_update(void);
static void chassis_pid_clear_state(pid_t *pid);
static bool chassis_3508_feedback_ready(void);
static void chassis_update_target_rpm(float vx, float vy, float vw);
static void chassis_mode_manual_update(void);
static void chassis_mode_auto_update(void);

typedef void (*chassis_mode_handler_t)(void);

static TaskHandle_t chassis_mode_task_handle;
void chassis_mode_task(void *pvParameters);
static TaskHandle_t chassis_driver_task_handle;
void chassis_driver_task(void *pvParameters);

/**
 * @brief 底盘模式切换：手动/自动
 */
void chassis_mode_task(void *pvParameters) {
    (void)pvParameters;
    static const chassis_mode_handler_t mode_handlers[] = {
        [CHASSIS_MODE_MANUAL] = chassis_mode_manual_update,
        [CHASSIS_MODE_AUTO] = chassis_mode_auto_update,
    };
    if(chassis_handle.mode == CHASSIS_MODE_AUTO) {
        LED1_ON();
    } else {
        LED1_OFF();
    }
    while (1) {
        if ((uint8_t)chassis_handle.mode <
                (sizeof(mode_handlers) / sizeof(mode_handlers[0])) &&
            mode_handlers[chassis_handle.mode] != NULL) {
            mode_handlers[chassis_handle.mode]();
        }
        vTaskDelay(5);
    }
}

/**
 * @brief 底盘驱动任务：根据当前模式计算电机输出
 */
void chassis_driver_task(void *pvParameters) {
    (void)pvParameters;
    int16_t motor_out_current[4] = {0};
    bool feedback_ready_last = false;

    while (1) {
        bool feedback_ready = chassis_3508_feedback_ready();

        if (!feedback_ready) {
            for (int i = 0; i < 4; i++) {
                motor_out_current[i] = 0;
                chassis_pid_clear_state(&dji_3508_speed_pid[i]);
                chassis_pid_clear_state(&dji_3508_pos_pid[i]);
            }

            if (feedback_ready_last) {
                log_message(LOG_ERROR, "3508 feedback lost, output disabled.");
            }
            feedback_ready_last = false;

            dji_motor_set_current(CHASSIS_CAN_SELECT, DJI_MOTOR_GROUP1,
                                  motor_out_current[0], motor_out_current[1],
                                  motor_out_current[2], motor_out_current[3]);
            vTaskDelay(3);
            continue;
        }

        if (!feedback_ready_last) {
            log_message(LOG_INFO,
                        "3508 feedback ready, enable chassis output.");
        }
        feedback_ready_last = true;

        if (!chassis_handle.halt) {
            float ff_gain[4] = {1.80f, 1.80f, 1.65f, 1.75f};
            for (int i = 0; i < 4; i++) {
                float real_rpm = dji_3508_handle[i].speed_rpm;
                float target_rpm = chassis_handle.chassis_speed.target_rpm[i];
                float calc_current =
                    pid_calc(&dji_3508_speed_pid[i], target_rpm, real_rpm);
                float forwardfeed = target_rpm * ff_gain[i];
                motor_out_current[i] = (int16_t)(calc_current + forwardfeed);
            }
        } else {
            for (int i = 0; i < 4; i++) {
                float target_degree = chassis_handle.g_current_rotor_degree[i];
                float measure_degree = dji_3508_handle[i].rotor_degree;
                float target_rpm = pid_calc(&dji_3508_pos_pid[i], target_degree,
                                            measure_degree);
                float measure_rpm = dji_3508_handle[i].speed_rpm;
                float calc_current =
                    pid_calc(&dji_3508_speed_pid[i], target_rpm, measure_rpm);
                motor_out_current[i] = (int16_t)calc_current;
            }
        }

        dji_motor_set_current(CHASSIS_CAN_SELECT, DJI_MOTOR_GROUP1,
                              motor_out_current[0], motor_out_current[1],
                              motor_out_current[2], motor_out_current[3]);
        vTaskDelay(3);
    }
}

static void chassis_switch_mode(uint8_t key, remote_key_event_t event) {
    UNUSED(event);

    /* 若抬升处于动作序列，禁止SET_HALT_KEY以外的按键 */
    if (lift_is_sequence_running() && key != SET_HALT_KEY) {
        return;
    }

    switch (key) {
        case SET_HALT_KEY:
            chassis_set_halt(!chassis_handle.halt);
            break;

        case SWITCH_AUTO_KEY:
            chassis_handle.mode = (chassis_handle.mode == CHASSIS_MODE_AUTO)
                                      ? CHASSIS_MODE_MANUAL
                                      : CHASSIS_MODE_AUTO;
            /* 更新LED1：亮表示自动模式，灭表示手动模式 */
            if (chassis_handle.mode == CHASSIS_MODE_AUTO) {
                LED1_ON();
            } else {
                LED1_OFF();
            }
            lift_set_chassis_mode(chassis_handle.mode == CHASSIS_MODE_AUTO);
            log_message(LOG_INFO, (chassis_handle.mode == CHASSIS_MODE_AUTO)
                                      ? "Switch to auto mode. "
                                      : "Switch to manual mode. ");
            break;

        // case SWITCH_WORLD_KEY:
        //     chassis_handle.world_cordinate = !chassis_handle.world_cordinate;
        //     log_message(LOG_INFO, chassis_handle.world_cordinate
        //                               ? "Set chassis to world coordinate. "
        //                               : "Set chassis to self coordinate. ");
        //     break;

        default:
            break;
    }
}

/**
 * @brief 底盘总初始化：电机、PID、任务、按键回调
 */
void chassis_init(void) {
    chassis_bottom_init();
    lift_init();
    chassis_tasks_init();

    // remote_register_key_callback(SWITCH_WORLD_KEY, REMOTE_KEY_PRESS_UP,
    //                              chassis_switch_mode);
    remote_register_key_callback(SWITCH_AUTO_KEY, REMOTE_KEY_PRESS_UP,
                                 chassis_switch_mode);
    remote_register_key_callback(SET_HALT_KEY, REMOTE_KEY_PRESS_UP,
                                 chassis_switch_mode);
}

/**
 * @brief 底盘底层初始化：配置电机、PID和运动规划参数
 */
static void chassis_bottom_init(void) {
    for (int i = 0; i < 4; i++) {
        if (dji_motor_init(&dji_3508_handle[i], DJI_M3508, CAN_Motor1_ID + i,
                           CHASSIS_CAN_SELECT) != 0) {
            log_message(LOG_ERROR, "3508 motor init failed: idx=%d", i);
            return;
        }
    }

    pid_init(&dji_3508_speed_pid[0], 16384.0f, 800.0f, 0.0f, 16384.0f,
             DELTA_PID, 7.50f, 0.035f, 0.35f);
    pid_init(&dji_3508_speed_pid[1], 16384.0f, 800.0f, 0.0f, 16384.0f,
             DELTA_PID, 7.50f, 0.043f, 0.35f);
    pid_init(&dji_3508_speed_pid[2], 16384.0f, 800.0f, 0.0f, 16384.0f,
             DELTA_PID, 7.20f, 0.033f, 0.35f);
    pid_init(&dji_3508_speed_pid[3], 16384.0f, 800.0f, 0.0f, 16384.0f,
             DELTA_PID, 7.40f, 0.043f, 0.35f);

    pid_init(&dji_3508_pos_pid[0], 5000.0f, 0.0f, 5.5f, 360.0f, POSITION_PID,
             55.0f, 0.0f, 0.0f);
    pid_init(&dji_3508_pos_pid[1], 5000.0f, 0.0f, 5.5f, 360.0f, POSITION_PID,
             55.0f, 0.0f, 0.0f);
    pid_init(&dji_3508_pos_pid[2], 5000.0f, 0.0f, 5.5f, 360.0f, POSITION_PID,
             55.0f, 0.0f, 0.0f);
    pid_init(&dji_3508_pos_pid[3], 5000.0f, 0.0f, 5.5f, 360.0f, POSITION_PID,
             55.0f, 0.0f, 0.0f);

    chassis_plan_config_t plan_cfg;
    chassis_plan_config_default(&plan_cfg);

    plan_cfg.max_speed_xy = MAX_SPEED_XY;
    plan_cfg.max_speed_w = MAX_SPEED_W;
    plan_cfg.max_accel_xy = MAX_ACCEL_XY;
    plan_cfg.max_accel_w = MAX_ACCEL_W;

    chassis_plan_init(&plan_cfg, CHASSIS_SPEED_PLAN_TRAPEZOID);
}

/**
 * @brief 底盘任务初始化：创建模式切换任务和驱动任务
 */
static void chassis_tasks_init(void) {
    if (xTaskCreate(chassis_mode_task, "chassis_mode_task", 256, NULL, 4,
                    &chassis_mode_task_handle) != pdPASS) {
        log_message(LOG_ERROR, "Failed to create chassis_mode_task.");
        return;
    }

    if (xTaskCreate(chassis_driver_task, "chassis_driver_task", 256, NULL, 5,
                    &chassis_driver_task_handle) != pdPASS) {
        log_message(LOG_ERROR, "Failed to create chassis_driver_task.");
    }
}

/**
 * @brief 设置底盘急停状态
 */
static void chassis_set_halt(bool enable) {
    if (chassis_handle.halt == enable) {
        return;
    }

    chassis_handle.halt = enable;

    /* 更新LED3：亮表示自锁，灭表示非自锁 */
    // if (enable) {
    //     LED3_ON();
    // } else {
    //     LED3_OFF();
    // }

    for (int i = 0; i < 4; i++) {
        chassis_pid_clear_state(&dji_3508_speed_pid[i]);
        chassis_pid_clear_state(&dji_3508_pos_pid[i]);
    }

    if (enable) {
        for (int i = 0; i < 4; i++) {
            chassis_handle.chassis_speed.target_rpm[i] = 0.0f;
        }
        chassis_halt_degree_update();
        log_message(LOG_INFO, "Set chassis halt. ");
    } else {
        log_message(LOG_INFO, "Release chassis halt. ");
    }
}

/**
 * @brief 底盘自锁时更新目标位置为当前转子位置，保持位置不变
 */
static void chassis_halt_degree_update(void) {
    if (chassis_handle.halt) {
        for (int i = 0; i < 4; i++) {
            chassis_handle.g_current_rotor_degree[i] =
                dji_3508_handle[i].rotor_degree;
        }
    }
}

/**
 * @brief 清除PID状态
 */
static void chassis_pid_clear_state(pid_t *pid) {
    pid->iout = 0.0f;
    pid->pos_out = 0.0f;

#if PID_USE_DELTA_PID
    pid->err[0] = 0.0f;
    pid->err[1] = 0.0f;
    pid->err[2] = 0.0f;
    pid->delta_u = 0.0f;
    pid->delta_out = 0.0f;
    pid->delta_lastout = 0.0f;
#else
    pid->err[0] = 0.0f;
    pid->err[1] = 0.0f;
#endif
}

/**
 * @brief 检查3508电机反馈是否准备好：所有电机都收到过反馈数据
 */
static bool chassis_3508_feedback_ready(void) {
    for (int i = 0; i < 4; i++) {
        if (!dji_3508_handle[i].got_offset) {
            return false;
        }
    }
    return true;
}

/**
 * @brief 更新底盘目标转速：根据输入的线速度和角速度计算每个轮子的目标转速
 */
static void chassis_update_target_rpm(float vx, float vy, float vw) {
    chassis_slope_t target_speed = {
        .vx = vx,
        .vy = vy,
        .vw = vw,
    };

    /* 当前导航消息中未提供稳定可用的航向角，暂不进行世界系变换。 */
    (void)chassis_handle.world_cordinate;
    omni_wheels_resolve(&target_speed, chassis_handle.chassis_speed.target_rpm);
}

/**
 * @brief 底盘手动模式更新：根据遥控器输入计算目标转速
 */
static void chassis_mode_manual_update(void) {
    float target_y = -g_remote_ctrl_data.rs[0] / 10.0f;
    float target_x = g_remote_ctrl_data.rs[1] / 10.0f;
    float target_yaw = -g_remote_ctrl_data.rs[2] / 5.0f;

    if (fabs(target_x) < 0.1f) {
        target_x = 0.0f;
    }
    if (fabs(target_y) < 0.1f) {
        target_y = 0.0f;
    }
    if (fabs(target_yaw) < 0.1f) {
        target_yaw = 0.0f;
    }

    chassis_update_target_rpm(target_x, target_y, target_yaw);
}

/**
 * @brief 底盘自动模式更新：根据导航参数计算目标转速
 */
static void chassis_mode_auto_update(void) {
    chassis_update_target_rpm(nav_sub_pram.linear_x, nav_sub_pram.linear_y,
                              nav_sub_pram.angular_z);

}
