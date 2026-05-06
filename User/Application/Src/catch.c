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
#include "npn_switch/npn_switch.h"

#define CATCH_AUTO_FLOW_ENABLE   0U
#define CATCH_TASK_PERIOD_MS     5U
#define CATCH_SENSOR_COUNT       6U

#define CATCH_STATE_INIT_KEY     11U
#define CATCH_STATE_READY_KEY    12U
#define CATCH_STATE_GRAB_KEY     13U
#define CATCH_STATE_CHECK_KEY    14U
#define CATCH_STATE_ASSEMBLY_KEY 15U
#define CATCH_STATE_DONE_KEY     16U

/**
 * @brief 状态内流程定义
 * 
 */
typedef enum {
    CATCH_FLOW_WAIT_FIRST_DETECT = 0, /* 在 READY 阶段等待首次检测到物体 */
    CATCH_FLOW_WAIT_SECOND_DETECT,    /* 二次检测，确认下一阶段条件成立 */
    CATCH_FLOW_DONE,                  /* 检测完成，进入拼接态*/
} catch_flow_t;

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
static catch_flow_t catch_flow = CATCH_FLOW_WAIT_FIRST_DETECT;

/* 传感器稳定计数：active 连续达到阈值才认为状态有效 */
static uint8_t sensor_active_cnt = 0;
static TaskHandle_t catch_task_handle;
TaskHandle_t catch_feedback_handle;

static void catch_task(void *pvParameters);
static void catch_tasks_init(void);
static void catch_update(void);
void catch_set_state(catch_state_t state);
static void catch_update_flow_for_state(catch_state_t state);
static void catch_remote_state_switch(uint8_t key, remote_key_event_t event);
static void catch_apply_state(catch_state_t state);
static uint8_t catch_is_sensor_active(void);
static void catch_update_sensor_counter(void);
static void catch_process_auto_flow(void);

bool check_sensor_active(void);

/**
 * @brief 夹爪整体各阶段下电机状态
 * 
 */
static const catch_motor_target_t g_catch_motor_targets[CATCH_STATE_COUNT] = {
    [CATCH_STATE_INIT] =
        {
            .servo_target = CATCH_HEAD_SERVO_TARGET_CLOSE,
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
    [CATCH_STATE_CHECK] =
        {
            .servo_target = CATCH_HEAD_SERVO_TARGET_CLOSE,
            .dm_target = CATCH_HEAD_DM_TARGET_CHECK,
        },
    [CATCH_STATE_ASSEMBLY] =
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

    if (state == catch_state) {
        return;
    }

    catch_state = state;
    catch_apply_state(catch_state);
    catch_update_flow_for_state(catch_state);
}

/**
 * @brief 更新状态对应的流程
 * 
 * @param state 
 */
static void catch_update_flow_for_state(catch_state_t state) {
    switch (state) {
        case CATCH_STATE_INIT:
        case CATCH_STATE_READY: {
            catch_flow = CATCH_FLOW_WAIT_FIRST_DETECT;
        } break;

        case CATCH_STATE_GRAB: {
            catch_flow = CATCH_FLOW_WAIT_SECOND_DETECT;
        } break;

        case CATCH_STATE_CHECK:
        case CATCH_STATE_ASSEMBLY:
        default: {
            catch_flow = CATCH_FLOW_DONE;
        } break;
    }

    sensor_active_cnt = 0;
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
 * @brief 判断传感器是否被触发
 * 
 * @return uint8_t 
 */
static uint8_t catch_is_sensor_active(void) {
    return (uint8_t)(npn_switch_read_level() == GPIO_PIN_RESET);
}

/**
 * @brief 更新传感器计数器
 * 
 */
static void catch_update_sensor_counter(void) {
    if (catch_is_sensor_active()) {
        if (sensor_active_cnt < CATCH_SENSOR_COUNT) {
            sensor_active_cnt++;
        }
    } else {
        sensor_active_cnt = 0;
    }
}

/**
 * @brief 夹取状态更新，更新传感器状态，处理自动流程，更新电机状态
 * 
 */
static void catch_update(void) {
    catch_update_sensor_counter();
    catch_process_auto_flow();
    catch_head();
}

/**
 * @brief 初始化夹爪状态
 * 
 */
void catch_init(void) {
    catch_head_init();

    catch_state = CATCH_STATE_INIT;
    catch_update_flow_for_state(catch_state);
    catch_apply_state(catch_state);

    remote_register_key_callback(CATCH_STATE_INIT_KEY, REMOTE_KEY_PRESS_UP,
                                 catch_remote_state_switch);
    remote_register_key_callback(CATCH_STATE_READY_KEY, REMOTE_KEY_PRESS_UP,
                                 catch_remote_state_switch);
    remote_register_key_callback(CATCH_STATE_GRAB_KEY, REMOTE_KEY_PRESS_UP,
                                 catch_remote_state_switch);
    remote_register_key_callback(CATCH_STATE_ASSEMBLY_KEY, REMOTE_KEY_PRESS_UP,
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

        case CATCH_STATE_ASSEMBLY_KEY: {
            catch_set_state(CATCH_STATE_ASSEMBLY);
        } break;

        case CATCH_STATE_DONE_KEY: {
            catch_set_state(CATCH_STATE_DONE);
        } break;
        default: {
        } break;
    }
}

/**
 * @brief 夹取自动流程
 * 
 */
static void catch_process_auto_flow(void) {
#if CATCH_AUTO_FLOW_ENABLE
    switch (catch_flow) {
        case CATCH_FLOW_WAIT_FIRST_DETECT: {
            if (catch_state == CATCH_STATE_READY &&
                sensor_active_cnt >= CATCH_SENSOR_COUNT) {
                catch_set_state(CATCH_STATE_GRAB);
            }
        } break;

        case CATCH_FLOW_WAIT_SECOND_DETECT: {
            if (catch_state == CATCH_STATE_GRAB &&
                sensor_active_cnt >= CATCH_SENSOR_COUNT) {
                catch_set_state(CATCH_STATE_ASSEMBLY);
            }
        } break;

        case CATCH_FLOW_DONE:
        default: {
        } break;
    }
#else
    UNUSED(catch_flow);
#endif
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
        check_sensor_active();
        vTaskDelay(pdMS_TO_TICKS(CATCH_TASK_PERIOD_MS));
    }
}

/**
 * @brief 夹取反馈任务函数
 * @note 该任务等待通知触发，触发后调用 grab_microros_publish(),检测动作是否完成， 发布成功失败到 ROS2.
 * 
 * @param pvParameters 
 */
static void catch_feedback_task(void *pvParameters) {
    UNUSED(pvParameters);

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

      
       while(!catch_head_is_target_reached()){
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        control_dispatch_publish(1);
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

bool check_sensor_active(void) {
    // 读取 PE8 引脚的状态
    static int8_t count = 0;
    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_7) == GPIO_PIN_SET && count == 0 ) {
        // log_message(LOG_INFO, "Target Detected!");
        count = 1;
        return true;
    } else {
        count = 0;
        return false;
    }
}
