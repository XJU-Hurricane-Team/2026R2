/**
 * @file catch.c
 * @author xinglu
 * @brief 矛头夹取任务
 * @version 1.3
 * @date 2026-04-15
 */

#include "includes.h"
#include "catch.h"
#include "microros_ctrl.h"
#include "vl53l1/vl53l1_apply.h"

#define CATCH_TASK_PERIOD_MS      5U
#define CATCH_STATE_INIT_KEY      11U
#define CATCH_STATE_READY_KEY     12U
#define CATCH_STATE_GRAB_KEY      13U
#define CATCH_STATE_CHECK_KEY     14U
#define CATCH_STATE_RECOGNIZE_KEY 15U
#define CATCH_STATE_DONE_KEY      16U

/**
 * @brief 电机目标状态定义
 * 
 */
typedef struct {
    uint8_t servo_target;
    uint8_t dm_target;
    uint8_t dji_target;
} catch_motor_target_t;

static catch_state_t catch_state = CATCH_STATE_INIT;
static bool recognize_published = false;

static TaskHandle_t catch_task_handle;
TaskHandle_t catch_feedback_handle;

static void catch_task(void *pvParameters);
static void catch_tasks_init(void);
static void catch_update(void);
void catch_set_state(catch_state_t state);

static void catch_remote_state_switch(uint8_t key, remote_key_event_t event);
static void catch_apply_state(catch_state_t state);

/**
 * @brief 夹爪整体各阶段下电机状态
 * 
 */
static const catch_motor_target_t g_catch_motor_targets[CATCH_STATE_COUNT] = {
    [CATCH_STATE_INIT] =
        {
            .servo_target = CATCH_HEAD_SERVO_TARGET_OPEN,
            .dm_target = CATCH_HEAD_DM_TARGET_RETRACT,
        },
    [CATCH_STATE_READY] =
        {
            .servo_target = CATCH_HEAD_SERVO_TARGET_OPEN,
            .dm_target = CATCH_HEAD_DM_TARGET_EXTEND,
        },
    [CATCH_STATE_GRAB] =
        {
            .servo_target = CATCH_HEAD_SERVO_TARGET_CLOSE,
            .dm_target = CATCH_HEAD_DM_TARGET_EXTEND,
        },
    [CATCH_STATE_RECOGNIZE] =
        {
            .servo_target = CATCH_HEAD_SERVO_TARGET_OPEN,
            .dm_target = CATCH_HEAD_DM_TARGET_EXTEND,
        },
    [CATCH_STATE_CHECK] =
        {
            .servo_target = CATCH_HEAD_SERVO_TARGET_CLOSE,
            .dm_target = CATCH_HEAD_DM_TARGET_CHECK,
        },
    [CATCH_STATE_DONE] =
        {
            .servo_target = CATCH_HEAD_SERVO_TARGET_OPEN,
            .dm_target = CATCH_HEAD_DM_TARGET_CHECK,
        },
};

/*
 * @brief 统一状态切换接口。
 * @param state 目标状态。
 * @note 会自动处理状态切换的流程更新和电机目标更新，无需外部重复调用相关函数。
 */
void catch_set_state(catch_state_t state) {
    if (state >= CATCH_STATE_COUNT) {
        return;
    }

    catch_state = state;
    if (catch_state == CATCH_STATE_RECOGNIZE) {
        recognize_published = false;
    }
    catch_apply_state(catch_state);

    if (catch_feedback_handle != NULL) {
        xTaskNotifyGive(catch_feedback_handle);
    }
}

/**
 * @brief 更新电机状态
 * 
 * @param state 
 */
static void catch_apply_state(catch_state_t state) {
    if (state >= CATCH_STATE_COUNT) {
        return;
    }
    catch_head_set_servo_target(g_catch_motor_targets[state].servo_target);
    catch_head_set_dm_target(g_catch_motor_targets[state].dm_target);
}

/**
 * @brief 夹取状态更新，更新传感器状态，处理自动流程，更新电机状态
 * 
 */
static void catch_update(void) {

    catch_head();
}

/**
 * @brief 初始化夹爪状态
 * 
 */
void catch_init(void) {
    catch_head_init();
    vl53l1_apply_init();

    catch_state = CATCH_STATE_INIT;

    catch_apply_state(catch_state);

    remote_register_key_callback(CATCH_STATE_INIT_KEY, REMOTE_KEY_PRESS_UP,
                                 catch_remote_state_switch);
    remote_register_key_callback(CATCH_STATE_READY_KEY, REMOTE_KEY_PRESS_UP,
                                 catch_remote_state_switch);
    remote_register_key_callback(CATCH_STATE_GRAB_KEY, REMOTE_KEY_PRESS_UP,
                                 catch_remote_state_switch);
    remote_register_key_callback(CATCH_STATE_RECOGNIZE_KEY, REMOTE_KEY_PRESS_UP,
                                 catch_remote_state_switch);
    remote_register_key_callback(CATCH_STATE_CHECK_KEY, REMOTE_KEY_PRESS_UP,
                                 catch_remote_state_switch);
    remote_register_key_callback(CATCH_STATE_DONE_KEY, REMOTE_KEY_PRESS_UP,
                                 catch_remote_state_switch);
    catch_tasks_init();
}

/**
 * @brief 按键回调
 * 
 * @param key 
 * @param event 
 */
static void catch_remote_state_switch(uint8_t key, remote_key_event_t event) {
    UNUSED(event);

    switch (key) {
        case CATCH_STATE_INIT_KEY: {
            catch_set_state(CATCH_STATE_INIT);
        } break;

        case CATCH_STATE_READY_KEY: {
            catch_set_state(CATCH_STATE_READY);
        } break;

        case CATCH_STATE_GRAB_KEY: {
            catch_set_state(CATCH_STATE_GRAB);
        } break;

        case CATCH_STATE_CHECK_KEY: {
            catch_set_state(CATCH_STATE_CHECK);
        } break;

        case CATCH_STATE_RECOGNIZE_KEY: {
            catch_set_state(CATCH_STATE_RECOGNIZE);
        } break;

        case CATCH_STATE_DONE_KEY: {
            catch_set_state(CATCH_STATE_DONE);
        } break;
        default: {
        } break;
    }
}

/**
 * @brief 夹取任务函数
 * 
 * @param pvParameters 
 */
static void catch_task(void *pvParameters) {
    UNUSED(pvParameters);

    while (1) {
        catch_update();
        vTaskDelay(pdMS_TO_TICKS(CATCH_TASK_PERIOD_MS));
    }
}

/**
 * @brief 夹取反馈任务函数
 * @note 该任务等待通知触发，触发后检测动作是否完成， 发布成功到 ROS2.
 * 
 * @param pvParameters 
 */
static void catch_feedback_task(void *pvParameters) {
    UNUSED(pvParameters);

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        catch_state_t current_state = catch_state;
        while (!catch_head_is_target_reached()) {
            // 防止状态突然改变
            if (catch_state != current_state) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        log_message(LOG_INFO, "State %d reached target", current_state);

        if (catch_state != current_state) {
            continue; // 状态已被打断，重新等待新通知
        }

        // 电机到位后，根据不同任务执行不同逻辑
        switch (current_state) {

            case CATCH_STATE_INIT:
            case CATCH_STATE_READY:
            case CATCH_STATE_GRAB:
            case CATCH_STATE_DONE:
                control_dispatch_publish(1);
                break;

            case CATCH_STATE_RECOGNIZE:
                VL53L1_StartMeasurement(g_vl53l1_handle2);
                while (catch_state == CATCH_STATE_RECOGNIZE) {
                    uint16_t dist = 0;
                    if (vl53l1_apply_get_distance_mm(&dist, g_vl53l1_handle2)) {
                        if (dist < VL53L1_APPLY_DISTANCE_THRESHOLD_MM &&
                            dist > 0) {
                            nav_publish(0);

                            // 发现物体，触发下一步抓取
                            catch_set_state(CATCH_STATE_GRAB);
                            log_message(LOG_INFO, "Recognized! dist = %d",
                                        dist);

                            break;
                        }
                    }
                    vTaskDelay(pdMS_TO_TICKS(30)); // 给 I2C 留出刷新时间
                }
                VL53L1_StopMeasurement(g_vl53l1_handle2);
                break;

            case CATCH_STATE_CHECK:
                // 检测抓取是否成功。
                {
                    uint16_t dist = 0;
                    uint32_t total_dist = 0;
                    uint8_t valid_count = 0;
                    VL53L1_StartMeasurement(g_vl53l1_handle);
                    for (int i = 0; i < 3; i++) {
                        if (vl53l1_apply_get_distance_mm(&dist,
                                                         g_vl53l1_handle)) {
                            total_dist += dist;
                            valid_count++;
                        }
                        vTaskDelay(pdMS_TO_TICKS(30));
                    }
                    dist = total_dist / valid_count;
                    if (total_dist > 0 && valid_count > 0 && dist <= 200) {
                        control_dispatch_publish(1); // 成功抓取
                    } else {
                        control_dispatch_publish(1); // 抓取失败
                    }
                     VL53L1_StopMeasurement(g_vl53l1_handle);
                    log_message(LOG_INFO, "Check! dist = %d", dist);
                }
                break;

            default:
                break;
        }
    }
}

/**
 * @brief 夹取任务初始化
 * 
 */
static void catch_tasks_init(void) {
    xTaskCreate(catch_task, "catch_task", 256, NULL, 3, &catch_task_handle);
    xTaskCreate(catch_feedback_task, "catch_feedback_task", 256, NULL, 3,
                &catch_feedback_handle);
}
