/**
 * @file    lift.c
 * @author  Dominate0017
 * @brief   抬升与2006控制模块
 * @version 1.2
 * @date    2026-05-17
 * ********************************************************************************
 *    Date    | Version |   Author    | Version Info
 * -----------+---------+-------------+----------------------------------------
 * 2026-04-28 |   1.0   | Dominate0017 | 改用上升/下降沿式判断光电状态
 * 2026-05-02 |   1.1   | Dominate0017 | 修改光电状态残留问题，代码重构
 * 2026-05-17 |   1.2   | Dominate0017 | 增加前后光电个数，优化上升/下降沿判定逻辑
 */
#include "includes.h"
#include "microros_ctrl.h"
#include "lift.h"
#include "vl53l1/vl53l1_apply.h"

#define CHASSIS_CAN_SELECT       can1_selected
#define LIFT_CAN_SELECT          can2_selected

#define LIFT_CATCH_DEGREE_KEY    1 /* 夹爪高度：5.049rad*/
#define LIFT_UP_R1_KEY           2 /* 抬升到R1位置：5.42rad */
#define LIFT_SEQ_UP_KEY          5 /* 上台阶按键 */
#define LIFT_SEQ_DOWN_KEY        6 /* 下台阶按键 */
#define LIFT_UP_KEY              7 /* 抬升升起按键 */
#define LIFT_DOWN_KEY            8 /* 抬升下降按键 */
#define LIFT_STOP_KEY            9 /* 中断序列并复位按键 */

#define SENSOR_GPIO_PORT_0       GPIOE
#define SENSOR_GPIO_PORT_1       GPIOF
#define PROXIMITY_SENSOR_PORT    GPIOC
#define FRONT_SENSOR_PIN_0       GPIO_PIN_2
#define FRONT_SENSOR_PIN_1       GPIO_PIN_2
#define MIDDLE_SENSOR_PIN_0      GPIO_PIN_4
#define MIDDLE_SENSOR_PIN_1      GPIO_PIN_9
#define REAR_SENSOR_PIN_0        GPIO_PIN_5
#define REAR_SENSOR_PIN_1        GPIO_PIN_10
#define PROXIMITY_SENSOR_PIN_0   GPIO_PIN_2
#define PROXIMITY_SENSOR_PIN_1   GPIO_PIN_3
#define LIFT_CATCH_ENABLE_PORT   GPIOB
#define LIFT_CATCH_ENABLE_PIN    GPIO_PIN_2
//2.7498
#define LIFT_TARGET_CATCH_DEG    2.40f      //2.6049 - 2.50 = 0.
#define LIFT_TARGET_DEG_UP_MAX   2.0f      //最初5.42
#define LIFT_TARGET_DEG_DOWN_MAX 12.275f
#define LIFT_TARGET_DEG_UP_SEQ   0.0f
#define LIFT_TARGET_DEG_DOWN_SEQ 12.275f
#define LIFT_TARGET_DEG_STEP     0.025f
#define LIFT_TARGET_SPEED_UP_R1  5.0f  
#define LIFT_TARGET_SPEED_HIGH   21.0f      //抬升快速
#define LIFT_TARGET_SPEED_LOW    7.5f       //抬升慢速
#define LIFT_TARGET_SPEED        10.0f

#define LIFT_2006_HIGH_SPEED     4850.0f
#define LIFT_2006_LOW_SPEED      3550.0f
#define TIME_DURATION_FRONT      0.4f
#define TIME_DURATION_REAR       1.70f   

typedef enum {
    LIFT_STATE_NORMAL = 0,
    LIFT_STATE_DOWN,
    LIFT_STATE_UP,
} lift_state_t;

typedef struct {
    uint8_t action; /* 1=抬升, 2=下降 */
    uint8_t step;
} lift_fsm_t;

typedef struct {
    bool current_state;
    bool last_stable_state;
    bool prev_stable_state; // 用于边缘检测
    uint32_t change_tick;
    uint32_t debounce_ms;
} photoelectric_debounce_t;

typedef struct {
    bool is_auto_mode;
    float target_2006_rpm;
    lift_state_t lift_state;
    float lift_target_degree;
    lift_fsm_t lift_fsm;
    uint32_t step3_start_tick;    /* tick when step 3 (drive 2006) started */
    float step3_last_duration;    /* last measured duration from step3 start to step4 (seconds) */
    bool catch_up_until_proximity_active;
} lift_handle_t;

static bool g_auto_lift_target_pending = false;
static float g_auto_lift_target_degree = 0.0f;
bool up_R1_flag = false;

static lift_handle_t g_lift_handle = {
    .is_auto_mode = false,
    .target_2006_rpm = 0.0f,
    .lift_state = LIFT_STATE_NORMAL,
    .lift_target_degree = 0.0f,
    .lift_fsm = {0, 0},
    .step3_start_tick = 0,
    .step3_last_duration = 0.0f,
    .catch_up_until_proximity_active = false,
};

static photoelectric_debounce_t g_rear_photoelectric_debounce = {
    .current_state = false,
    .last_stable_state = false,
    .prev_stable_state = false,
    .change_tick = 0,
    .debounce_ms = 6,
};

static photoelectric_debounce_t g_front_photoelectric_debounce = {
    .current_state = false,
    .last_stable_state = false,
    .prev_stable_state = false,
    .change_tick = 0,
    .debounce_ms = 6,
};

static photoelectric_debounce_t g_middle_photoelectric_debounce = {
    .current_state = false,
    .last_stable_state = false,
    .prev_stable_state = false,
    .change_tick = 0,
    .debounce_ms = 6,
};

/* VL53L1 距离跳变检测——用于下降序列 step3 辅助触发 */
static uint16_t s_last_vl53l1_dist_mm = 0;   /* 上一次有效测距值 */
static bool     s_vl53l1_has_last = false;   /* 是否已有有效历史值 */
#define VL53L1_DELTA_TRIGGER_MIN_MM  150      /* 距离跳变触发下限 */
#define VL53L1_DELTA_TRIGGER_MAX_MM  350      /* 距离跳变触发上限 */

typedef void (*lift_seq_handler_t)(void);
typedef void (*lift_step_handler_t)(void);

static dji_motor_handle_t dji_2006_handle[2];
static dm_handle_t dm_motor_handle[2];
static pid_t dji_2006_pid[2];

static TaskHandle_t lift_state_task_handle;
static void lift_state_task(void *pvParameters);
static TaskHandle_t lift_sequence_task_handle;
static void lift_sequence_task(void *pvParameters);
static TaskHandle_t chassis_proximity_switch_task_handle;
static void chassis_proximity_switch_task(void *pvParameters);

static void lift_bottom_init(void);
static void lift_tasks_init(void);
static void lift_seq_update(void);
static void lift_seq_start(uint8_t action);
static void lift_seq_up_update(void);
static void lift_seq_down_update(void);

static bool get_rear_photoelectric(void);
static bool get_front_photoelectric(void);
static bool get_middle_photoelectric(void);
static bool get_front_photoelectric_rising_edge(void);
static bool get_front_photoelectric_falling_edge(void);
static bool get_rear_photoelectric_rising_edge(void);
static bool get_rear_photoelectric_falling_edge(void);
static bool get_middle_photoelectric_rising_edge(void);
static bool get_middle_photoelectric_falling_edge(void);
static bool photoelectric_get_stable_state(photoelectric_debounce_t *debounce,
                                           GPIO_TypeDef *port_0,
                                           uint16_t pin_0,
                                           GPIO_TypeDef *port_1,
                                           uint16_t pin_1);
static void photoelectric_sync_state(photoelectric_debounce_t *debounce,
                                     bool raw_state);
static void lift_sync_photoelectric_state(void);

static bool dm_position_check(lift_state_t state);
static float lift_limit_target(float target_degree);
void auto_lift_set_target(float target_degree);
static float lift_select_target_speed(float target_degree, float real_degree);
void lift_set_target(float target_degree);
static void lift_publish_if_auto(uint8_t code);
static void lift_finish_sequence(void);
static void lift_seq_emergency_stop(void);

static void lift_up_step_wait_front_trigger(void);
static void lift_up_step_set_down_target(void);
static void lift_up_step_wait_down_arrived(void);
static void lift_up_step_drive_2006_forward(void);
static void lift_up_step_wait_up_arrived_and_finish(void);
static void lift_down_step_wait_rear_release(void);
static void lift_down_step_set_down_target(void);
static void lift_down_step_wait_down_arrived(void);
static void lift_down_step_drive_2006_backward(void);
static void lift_down_step_wait_up_arrived_and_finish(void);

// 抬升状态任务：控制DM电机和2006电机
static void lift_state_task(void *pvParameters) {
    (void)pvParameters;
    int16_t motor_2006_out[2] = {0};

    vTaskDelay(pdMS_TO_TICKS(10));
    dm_motor_enable(&dm_motor_handle[0]);
    vTaskDelay(pdMS_TO_TICKS(10));
    dm_motor_enable(&dm_motor_handle[1]);

    while (1) {
        if (g_lift_handle.lift_fsm.action == 0) {
            if (g_lift_handle.is_auto_mode) {
                g_lift_handle.target_2006_rpm = 0.0f;
            } else {
                g_lift_handle.target_2006_rpm =
                    g_remote_ctrl_data.rs[3] * 250.0f;
            }

            switch (g_lift_handle.lift_state) {
                case LIFT_STATE_UP:
                     lift_set_target(g_lift_handle.lift_target_degree +
                                    LIFT_TARGET_DEG_STEP);
                    break;
                case LIFT_STATE_DOWN:
                    lift_set_target(g_lift_handle.lift_target_degree -
                                    LIFT_TARGET_DEG_STEP);
                    break;
                case LIFT_STATE_NORMAL:
                default:
                    break;
            }
        }

        for (int i = 0; i < 2; i++) {
            float real_2006_rpm = dji_2006_handle[i].speed_rpm;
            float target_rpm = (i == 0) ? g_lift_handle.target_2006_rpm
                                        : -g_lift_handle.target_2006_rpm;

            motor_2006_out[i] =
                (int16_t)pid_calc(&dji_2006_pid[i], target_rpm, real_2006_rpm);
        }

        dji_motor_set_current(CHASSIS_CAN_SELECT, DJI_MOTOR_GROUP2,
                              motor_2006_out[0], motor_2006_out[1], 0, 0);

        dm_pos_speed_ctrl(&dm_motor_handle[0], g_lift_handle.lift_target_degree,
                  lift_select_target_speed(g_lift_handle.lift_target_degree,
                               dm_motor_handle[1].position));
        dm_pos_speed_ctrl(&dm_motor_handle[1],
                  -g_lift_handle.lift_target_degree,
                  lift_select_target_speed(-g_lift_handle.lift_target_degree,
                               dm_motor_handle[1].position));

        if (g_auto_lift_target_pending &&
            g_lift_handle.lift_fsm.action == 0 &&
            (fabs(fabs(dm_motor_handle[1].position) -
                  fabs(g_auto_lift_target_degree)) < 0.1f)) {
            g_auto_lift_target_pending = false;
            control_dispatch_publish(1);
        }

        vTaskDelay(10);
    }
}

// 序列任务：处理上下台阶的动作序列
static void lift_sequence_task(void *pvParameters) {
    (void)pvParameters;
    while (1) {
        if (g_lift_handle.lift_fsm.action == 0) {
            (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            continue;
        }
        lift_seq_update();
        vTaskDelay(3);
    }
}

static bool g_chassis_proximity_check_active = false;

static void chassis_proximity_switch_task(void *pvParameters) {
    (void)pvParameters;
    while (1) {
        if (!g_chassis_proximity_check_active) {
            (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            continue;
        }
        bool proximity_1_low = (HAL_GPIO_ReadPin(PROXIMITY_SENSOR_PORT, PROXIMITY_SENSOR_PIN_0) == GPIO_PIN_RESET);
        bool proximity_2_low = (HAL_GPIO_ReadPin(PROXIMITY_SENSOR_PORT, PROXIMITY_SENSOR_PIN_1) == GPIO_PIN_RESET);
        if (proximity_1_low || proximity_2_low) {
            nav_publish(0);
            g_chassis_proximity_check_active = false; 
        }
        vTaskDelay(5); 
    }
}

// 切换模式：遥控按键回调处理
void lift_switch_mode(uint8_t key, remote_key_event_t event) {
    /* 抬升处于动作序列，禁止LIFT_STOP_KEY以外的按键 */
    if (g_lift_handle.lift_fsm.action != 0 && key != LIFT_STOP_KEY) {
        return;
    }

    switch (key) {
        case LIFT_STOP_KEY:
            lift_seq_emergency_stop();
            break;

        case LIFT_CATCH_DEGREE_KEY:
            // lift_begin_catch_up_until_proximity();
            lift_set_target(LIFT_TARGET_UP_RAMP_DEG);
            break;
        
        case LIFT_UP_R1_KEY:
            lift_set_target(-LIFT_TARGET_UP_R1_DEG);
            break;

        case LIFT_SEQ_UP_KEY:
            log_message(LOG_INFO, "Trigger Lift UP sequence.");
            lift_seq_start(1);
            // lift_publish_if_auto(1);
            nav_publish(1);
            break;

        case LIFT_SEQ_DOWN_KEY:
            log_message(LOG_INFO, "Trigger Lift DOWN sequence.");
            lift_seq_start(2);
            nav_publish(1);
            //lift_publish_if_auto(2);
            break;

        case LIFT_UP_KEY:
            switch (event) {
                case REMOTE_KEY_PRESS_DOWN:
                case REMOTE_KEY_PRESSING:
                    g_lift_handle.lift_state = LIFT_STATE_UP;
                    break;
                case REMOTE_KEY_PRESS_UP:
                    g_lift_handle.lift_state = LIFT_STATE_NORMAL;
                    break;
                default:
                    break;
            }
            break;

        case LIFT_DOWN_KEY:
            switch (event) {
                case REMOTE_KEY_PRESS_DOWN:
                case REMOTE_KEY_PRESSING:
                    g_lift_handle.lift_state = LIFT_STATE_DOWN;
                    break;
                case REMOTE_KEY_PRESS_UP:
                    g_lift_handle.lift_state = LIFT_STATE_NORMAL;
                    break;
                default:
                    break;
            }
            break;

        default:
            break;
    }
}

// 设置阶梯模式：自动控制上下台阶
void lift_set_stair_mode(uint8_t mode) {
    if (!g_lift_handle.is_auto_mode) {
        return;
    }

    switch (mode) {
        case 0:
            g_lift_handle.lift_fsm.action = 0;
            g_lift_handle.lift_fsm.step = 0;
            g_lift_handle.target_2006_rpm = 0.0f;
            break;
        case 1:
            lift_seq_start(1);
            break;
        case 2:
            lift_seq_start(2);
            break;
        default:
            break;
    }
}

// 设置底盘模式：切换自动/手动控制
void lift_set_chassis_mode(bool is_auto_mode) {
    g_lift_handle.is_auto_mode = is_auto_mode;
    if (is_auto_mode && g_lift_handle.lift_fsm.action == 0) {
        g_lift_handle.target_2006_rpm = 0.0f;
    } else if (!is_auto_mode) {
        g_auto_lift_target_pending = false;
    }
}

// 检查序列是否运行中
bool lift_is_sequence_running(void) {
    return (g_lift_handle.lift_fsm.action != 0);
}

void chassis_proximity_switch(void) {
    g_chassis_proximity_check_active = true;
    if (chassis_proximity_switch_task_handle != NULL) {
        xTaskNotifyGive(chassis_proximity_switch_task_handle); // 唤醒接近开关任务
    }
}

// 初始化抬升模块：电机、任务、按键回调
void lift_init(void) {
    lift_bottom_init();
    lift_tasks_init();

    remote_register_key_callback(LIFT_CATCH_DEGREE_KEY, REMOTE_KEY_PRESS_UP,
                                 lift_switch_mode);
    remote_register_key_callback(LIFT_SEQ_UP_KEY, REMOTE_KEY_PRESS_UP,
                                 lift_switch_mode);
    remote_register_key_callback(LIFT_SEQ_DOWN_KEY, REMOTE_KEY_PRESS_UP,
                                 lift_switch_mode);
    remote_register_key_callback(LIFT_UP_KEY, REMOTE_KEY_PRESS_DOWN,
                                 lift_switch_mode);
    remote_register_key_callback(LIFT_UP_KEY, REMOTE_KEY_PRESSING,
                                 lift_switch_mode);
    remote_register_key_callback(LIFT_UP_KEY, REMOTE_KEY_PRESS_UP,
                                 lift_switch_mode);
    remote_register_key_callback(LIFT_DOWN_KEY, REMOTE_KEY_PRESS_DOWN,
                                 lift_switch_mode);
    remote_register_key_callback(LIFT_DOWN_KEY, REMOTE_KEY_PRESSING,
                                 lift_switch_mode);
    remote_register_key_callback(LIFT_DOWN_KEY, REMOTE_KEY_PRESS_UP,
                                 lift_switch_mode);
    remote_register_key_callback(LIFT_STOP_KEY, REMOTE_KEY_PRESS_UP,
                                 lift_switch_mode);
}

// 底层初始化：配置2006电机、DM电机和PID
static void lift_bottom_init(void) {
    for (int i = 0; i < 2; i++) {
        if (dji_motor_init(&dji_2006_handle[i], DJI_M2006, CAN_Motor5_ID + i,
                           CHASSIS_CAN_SELECT) != 0) {
            log_message(LOG_ERROR, "2006 motor init failed: idx=%d", i);
            return;
        }
    }

    for (int i = 0; i < 2; i++) {
        dm_motor_init(&dm_motor_handle[i], 0x11 + i, 0x01 + i,
                      DM_MODE_POS_SPEED, DM_J4310, 12.5f, 30.0f, 10.0f,
                      LIFT_CAN_SELECT);  
    }

    for (int i = 0; i < 2; i++) {
        pid_init(&dji_2006_pid[i], 16384.0f, 500.0f, 2.0f, 15000.0f, POSITION_PID,
                 2.30f, 0.001f, 0.00f);
    }
}

// 创建两个FreeRTOS任务：状态任务和序列任务
static void lift_tasks_init(void) {
    if (xTaskCreate(lift_state_task, "lift_state_task", 256, NULL, 4,
                    &lift_state_task_handle) != pdPASS) {
        log_message(LOG_ERROR, "Failed to create lift_state_task.");
        return;
    }

    if (xTaskCreate(lift_sequence_task, "lift_sequence_task", 256, NULL, 4,
                    &lift_sequence_task_handle) != pdPASS) {
        log_message(LOG_ERROR, "Failed to create lift_sequence_task.");
    }

    if (xTaskCreate(chassis_proximity_switch_task, "chassis_proximity_switch_task", 256, NULL, 4,
                    &chassis_proximity_switch_task_handle) != pdPASS) {
        log_message(LOG_ERROR, "Failed to create chassis_proximity_switch_task.");
    }
}

// 序列更新：根据当前动作调用对应处理函数
static void lift_seq_update(void) {
    static const lift_seq_handler_t seq_handlers[] = {
        NULL,
        lift_seq_up_update,
        lift_seq_down_update,
    };

    if (g_lift_handle.lift_fsm.action <
            (sizeof(seq_handlers) / sizeof(seq_handlers[0])) &&
        seq_handlers[g_lift_handle.lift_fsm.action] != NULL) {
        seq_handlers[g_lift_handle.lift_fsm.action]();
    }
}

// 启动序列：初始化动作状态，发送任务通知
static void lift_seq_start(uint8_t action) {
    if (action != 1 && action != 2) {
        return;
    }

    /* 序列启动时清除边沿残留，避免上一次状态直接触发新序列。 */
    lift_sync_photoelectric_state();

    g_lift_handle.lift_fsm.action = action;
    g_lift_handle.lift_fsm.step = 0;

    /* 下降序列启动时，启动 VL53L1 测距并重置距离跟踪 */
    if (action == 2) {
        s_last_vl53l1_dist_mm = 0;
        s_vl53l1_has_last = false;
        vl53l1_apply_start_measurement(g_vl53l1_handle);
        vl53l1_apply_start_measurement(g_vl53l1_handle3);
    }

    if (lift_sequence_task_handle != NULL) {
        xTaskNotifyGive(lift_sequence_task_handle);
    }
}

// 上升序列：逐步执行5个步骤
static void lift_seq_up_update(void) {
    static const lift_step_handler_t step_handlers[] = {
        lift_up_step_wait_front_trigger,
        lift_up_step_set_down_target,
        lift_up_step_wait_down_arrived,
        lift_up_step_drive_2006_forward,
        lift_up_step_wait_up_arrived_and_finish,
    };

    if (g_lift_handle.lift_fsm.step <
            (sizeof(step_handlers) / sizeof(step_handlers[0])) &&
        step_handlers[g_lift_handle.lift_fsm.step] != NULL) {
        step_handlers[g_lift_handle.lift_fsm.step]();
    }
}

// 下降序列：逐步执行5个步骤
static void lift_seq_down_update(void) {
    static const lift_step_handler_t step_handlers[] = {
        lift_down_step_wait_rear_release,
        lift_down_step_set_down_target,
        lift_down_step_wait_down_arrived,
        lift_down_step_drive_2006_backward,
        lift_down_step_wait_up_arrived_and_finish,
    };
    // log_message(LOG_INFO, "2006_speed:%d", (int)dji_2006_handle[0].speed_rpm);
    if (g_lift_handle.lift_fsm.step <
            (sizeof(step_handlers) / sizeof(step_handlers[0])) &&
        step_handlers[g_lift_handle.lift_fsm.step] != NULL) {
        step_handlers[g_lift_handle.lift_fsm.step]();
    }
}

// 获取后光电状态（带防抖，只有全高/全低才更新稳定态）
static bool get_rear_photoelectric(void) {
    return photoelectric_get_stable_state(&g_rear_photoelectric_debounce,
                                          SENSOR_GPIO_PORT_0,
                                          REAR_SENSOR_PIN_0,
                                          SENSOR_GPIO_PORT_1,
                                          REAR_SENSOR_PIN_1);
}

// 获取前光电状态（带防抖，只有全高/全低才更新稳定态）
static bool get_front_photoelectric(void) {
    return photoelectric_get_stable_state(&g_front_photoelectric_debounce,
                                          SENSOR_GPIO_PORT_0,
                                          FRONT_SENSOR_PIN_0,
                                          SENSOR_GPIO_PORT_1,
                                          FRONT_SENSOR_PIN_1);
}

// 获取中光电状态（带防抖，只有全高/全低才更新稳定态）
static bool get_middle_photoelectric(void) {
    return photoelectric_get_stable_state(&g_middle_photoelectric_debounce, 
                                          SENSOR_GPIO_PORT_0,
                                          MIDDLE_SENSOR_PIN_0, 
                                          SENSOR_GPIO_PORT_1, 
                                          MIDDLE_SENSOR_PIN_1);
}

// 检测前光电上升沿（false -> true）
static bool get_front_photoelectric_rising_edge(void) {
    get_front_photoelectric(); // 更新状态
    bool rising = (!g_front_photoelectric_debounce.prev_stable_state &&
                   g_front_photoelectric_debounce.last_stable_state);
    if (rising) {
        g_front_photoelectric_debounce.prev_stable_state =
            g_front_photoelectric_debounce.last_stable_state;
    }
    return rising;
}

// 检测前光电下降沿（true -> false）
static bool get_front_photoelectric_falling_edge(void) {
    get_front_photoelectric(); // 更新状态
    bool falling = (g_front_photoelectric_debounce.prev_stable_state &&
                    !g_front_photoelectric_debounce.last_stable_state);
    if (falling) {
        g_front_photoelectric_debounce.prev_stable_state =
            g_front_photoelectric_debounce.last_stable_state;
    }
    return falling;
}

// 检测后光电上升沿（false -> true）
static bool get_rear_photoelectric_rising_edge(void) {
    get_rear_photoelectric(); // 更新状态
    bool rising = (!g_rear_photoelectric_debounce.prev_stable_state &&
                   g_rear_photoelectric_debounce.last_stable_state);
    if (rising) {
        g_rear_photoelectric_debounce.prev_stable_state =
            g_rear_photoelectric_debounce.last_stable_state;
    }
    return rising;
}

// 检测后光电下降沿（true -> false）
static bool get_rear_photoelectric_falling_edge(void) {
    get_rear_photoelectric(); // 更新状态
    bool falling = (g_rear_photoelectric_debounce.prev_stable_state &&
                    !g_rear_photoelectric_debounce.last_stable_state);
    if (falling) {
        g_rear_photoelectric_debounce.prev_stable_state =
            g_rear_photoelectric_debounce.last_stable_state;
    }
    return falling;
}

// 检测中光电上升沿（false -> true）
static bool get_middle_photoelectric_rising_edge(void) {
    get_middle_photoelectric(); // 更新状态
    bool rising = (!g_middle_photoelectric_debounce.prev_stable_state &&
                   g_middle_photoelectric_debounce.last_stable_state);
    if (rising) {
        g_middle_photoelectric_debounce.prev_stable_state =
            g_middle_photoelectric_debounce.last_stable_state;
    }
    return rising;
}

// 检测中光电下降沿（true -> false）
static bool get_middle_photoelectric_falling_edge(void) {
    get_middle_photoelectric(); // 更新状态
    bool falling = (g_middle_photoelectric_debounce.prev_stable_state &&
                    !g_middle_photoelectric_debounce.last_stable_state);
    if (falling) {
        g_middle_photoelectric_debounce.prev_stable_state =
            g_middle_photoelectric_debounce.last_stable_state;
    }
    return falling;
}

// 读取光电状态并更新防抖结构体
static bool photoelectric_get_stable_state(photoelectric_debounce_t *debounce,
                                           GPIO_TypeDef *port_0,
                                           uint16_t pin_0,
                                           GPIO_TypeDef *port_1,
                                           uint16_t pin_1) {
    bool pin_0_high = HAL_GPIO_ReadPin(port_0, pin_0);
    bool pin_1_high = HAL_GPIO_ReadPin(port_1, pin_1);
    bool both_high = pin_0_high && pin_1_high;
    bool both_low = !pin_0_high && !pin_1_high;

    if (!both_high && !both_low) {
        return debounce->last_stable_state;
    }

    bool raw_state = both_high;

    if (raw_state != debounce->current_state) {
        debounce->current_state = raw_state;
        debounce->change_tick = xTaskGetTickCount();
    }

    uint32_t elapsed_ms =
        (xTaskGetTickCount() - debounce->change_tick) * portTICK_PERIOD_MS;
    if (elapsed_ms >= debounce->debounce_ms) {
        // 保存上一次的稳定状态，然后更新当前稳定状态
        debounce->prev_stable_state = debounce->last_stable_state;
        debounce->last_stable_state = raw_state;
    }

    return debounce->last_stable_state;
}

static void photoelectric_sync_state(photoelectric_debounce_t *debounce,
                                     bool raw_state) {
    debounce->current_state = raw_state;
    debounce->last_stable_state = raw_state;
    debounce->prev_stable_state = raw_state;
    debounce->change_tick = xTaskGetTickCount();
}

// 同步所有光电状态（序列启动时使用）
static void lift_sync_photoelectric_state(void) {
    bool front_raw = 
        (HAL_GPIO_ReadPin(SENSOR_GPIO_PORT_0, FRONT_SENSOR_PIN_0) && 
         HAL_GPIO_ReadPin(SENSOR_GPIO_PORT_1, FRONT_SENSOR_PIN_1));
    bool middle_raw =
        (HAL_GPIO_ReadPin(SENSOR_GPIO_PORT_0, MIDDLE_SENSOR_PIN_0) &&
         HAL_GPIO_ReadPin(SENSOR_GPIO_PORT_1, MIDDLE_SENSOR_PIN_1));
    bool rear_raw =         
        (HAL_GPIO_ReadPin(SENSOR_GPIO_PORT_0, REAR_SENSOR_PIN_0) && 
         HAL_GPIO_ReadPin(SENSOR_GPIO_PORT_1, REAR_SENSOR_PIN_1));


    photoelectric_sync_state(&g_front_photoelectric_debounce, front_raw);
    photoelectric_sync_state(&g_rear_photoelectric_debounce, rear_raw);
    photoelectric_sync_state(&g_middle_photoelectric_debounce, middle_raw);
}

// 检查DM电机是否到达目标位置
static bool dm_position_check(lift_state_t state) {
    bool motor0_ready = false;
    bool motor1_ready = false;

    if (state == LIFT_STATE_DOWN) {
        // motor0_ready = (fabs(fabs(dm_motor_handle[0].position) -
        //                      LIFT_TARGET_DEG_DOWN_SEQ) < 0.1f);
        motor1_ready = (fabs(fabs(dm_motor_handle[1].position) -
                             LIFT_TARGET_DEG_DOWN_SEQ) < 0.1f);
        // return motor0_ready && motor1_ready;
        return motor1_ready;
    }

    if (state == LIFT_STATE_UP) {
        // motor0_ready = (fabs(fabs(dm_motor_handle[0].position) -
        //                      LIFT_TARGET_DEG_UP_SEQ) < 0.1f);
        motor1_ready = (fabs(fabs(dm_motor_handle[1].position) -
                             LIFT_TARGET_DEG_UP_SEQ) < 0.1f);
        //return motor0_ready && motor1_ready;
        return motor1_ready;
    }

    return false;
}

// 限制目标角度在最大范围内
static float lift_limit_target(float target_degree) {
    /* 正向限位为 +6.0f，负向限位为 -LIFT_TARGET_DEG_DOWN_MAX（-16.0f） */
    if (target_degree > LIFT_TARGET_DEG_UP_MAX) {
        return LIFT_TARGET_DEG_UP_MAX;
    }
    if (target_degree < -LIFT_TARGET_DEG_DOWN_MAX) {
        return -LIFT_TARGET_DEG_DOWN_MAX;
    }
    return target_degree;
}

// 设置DM电机目标角度
void lift_set_target(float target_degree) {
    g_lift_handle.lift_target_degree = lift_limit_target(target_degree);
}

// 自动模式设置DM电机目标角度(完成后反馈)
void auto_lift_set_target(float target_degree){
    g_auto_lift_target_degree = lift_limit_target(target_degree);
    g_lift_handle.lift_target_degree = g_auto_lift_target_degree;
    g_auto_lift_target_pending = true;
}

// 自动模式下发布微ROS消息
static void lift_publish_if_auto(uint8_t code) {
    if (g_lift_handle.is_auto_mode) {
        if (code == 0) {
            nav_publish((int8_t)code);
        } else {
            control_dispatch_publish((int8_t)code);
        }
    }
}

// 完成序列：恢复正常状态
static void lift_finish_sequence(void) {
    g_lift_handle.lift_state = LIFT_STATE_NORMAL;
    g_lift_handle.lift_fsm.action = 0;
    g_lift_handle.catch_up_until_proximity_active = false;
    up_R1_flag = false;

    /* 序列结束，停止 VL53L1 测距 */
    vl53l1_apply_stop_measurement(g_vl53l1_handle);
    vl53l1_apply_stop_measurement(g_vl53l1_handle3);
    s_vl53l1_has_last = false;
}

// 紧急停止：清除所有动作
static void lift_seq_emergency_stop(void) {
    g_lift_handle.lift_state = LIFT_STATE_NORMAL;
    g_lift_handle.lift_target_degree = 0.0f;
    g_lift_handle.target_2006_rpm = 0.0f;
    g_lift_handle.lift_fsm.action = 0;
    g_lift_handle.lift_fsm.step = 0;
    g_lift_handle.catch_up_until_proximity_active = false;
    g_auto_lift_target_pending = false;
    up_R1_flag = false;

    /* 紧急停止也关 VL53L1 */
    vl53l1_apply_stop_measurement(g_vl53l1_handle);
    s_vl53l1_has_last = false;

    log_message(LOG_INFO,
                "Lift sequence emergency stop: action=0, degree=0, 2006=0");
}

void lift_begin_catch_up_until_proximity(void) {
    if (g_lift_handle.lift_fsm.action != 0) {
        return;
    }

    if (HAL_GPIO_ReadPin(LIFT_CATCH_ENABLE_PORT, LIFT_CATCH_ENABLE_PIN) ==
        GPIO_PIN_RESET) {
        g_lift_handle.catch_up_until_proximity_active = false;
        g_lift_handle.lift_state = LIFT_STATE_NORMAL;
        g_lift_handle.target_2006_rpm = 0.0f;
        return;
    }

    g_lift_handle.catch_up_until_proximity_active = true;
    g_lift_handle.lift_state = LIFT_STATE_UP;
}

void lift_on_catch_proximity_falling_edge(void) {
    if (!g_lift_handle.catch_up_until_proximity_active) {
        return;
    }

    g_lift_handle.catch_up_until_proximity_active = false;
    g_lift_handle.lift_state = LIFT_STATE_NORMAL;
    g_lift_handle.target_2006_rpm = 0.0f;
    control_dispatch_publish(1);
}

static float lift_select_target_speed(float target_degree, float real_degree) {
    if (up_R1_flag) {
        return LIFT_TARGET_SPEED_UP_R1;
    }
    /* 非序列模式：恒定速度 */
    if (g_lift_handle.lift_fsm.action == 0) {
        return LIFT_TARGET_SPEED;
    }
    /* 序列模式：三段变速，基于总行程 LIFT_TARGET_DEG_DOWN_SEQ(12.275f)
       前 1/5(~2.455) 低速起步，中间 3/5 高速，后 1/5(~2.455) 低速收尾 */
    float target_error = fabsf(fabsf(target_degree) - fabsf(real_degree));
    float one_fifth = LIFT_TARGET_DEG_DOWN_SEQ / 5.0f;
    if (target_error <= one_fifth || target_error >= 4.0f * one_fifth) {
        return LIFT_TARGET_SPEED_LOW;
    }
    return LIFT_TARGET_SPEED_HIGH;
}

// 上升步骤1：等待前光电上升沿
static void lift_up_step_wait_front_trigger(void) {
    if (get_front_photoelectric_rising_edge()) {
        log_message(LOG_INFO, "chassis up");
        lift_publish_if_auto(0);
        g_lift_handle.lift_fsm.step = 1;
    }
}

// 上升步骤2：设置DM电机下降到初始位置
static void lift_up_step_set_down_target(void) {
    g_lift_handle.lift_state = LIFT_STATE_DOWN;
    lift_set_target(-LIFT_TARGET_DEG_DOWN_SEQ);
    g_lift_handle.lift_fsm.step = 2;
}

// 上升步骤3：等待DM电机下降到位
static void lift_up_step_wait_down_arrived(void) {
    if (dm_position_check(LIFT_STATE_DOWN)) {
        g_lift_handle.lift_fsm.step = 3;
    }
}

// 上升步骤4：2006电机正转，等待后光电上升沿
static void lift_up_step_drive_2006_forward(void) {
    static uint32_t step4_start_tick = 0;

    /* 记录 step4 开始 tick（只在首次进入时记录） */
    if (step4_start_tick == 0) {
        step4_start_tick = xTaskGetTickCount();
    }

    /* 计算自 step4 开始经过的秒数 */
    uint32_t now_tick = xTaskGetTickCount();
    float elapsed_sec = (now_tick - step4_start_tick) *
                        (portTICK_PERIOD_MS * 0.001f);

    /* 2006 转速：up_R1_flag 为 true 时恒定 1500rpm，否则三段变速：
       0 ~ TIME_DURATION_FRONT: 低速, TIME_DURATION_FRONT ~ TIME_DURATION_REAR: 高速, > TIME_DURATION_REAR: 低速 */
    if (up_R1_flag) {
        g_lift_handle.target_2006_rpm = 1500.0f;
    } else if (elapsed_sec < TIME_DURATION_FRONT) {
        g_lift_handle.target_2006_rpm = LIFT_2006_LOW_SPEED;
    } else if (elapsed_sec < TIME_DURATION_REAR) {
        g_lift_handle.target_2006_rpm = LIFT_2006_HIGH_SPEED;
    } else {
        g_lift_handle.target_2006_rpm = LIFT_2006_LOW_SPEED;
    }

    // 检查后光电的上升沿（false -> true）
    if (get_rear_photoelectric_rising_edge()) {
        step4_start_tick = 0;
        g_lift_handle.target_2006_rpm = 0.0f;
        g_lift_handle.lift_state = LIFT_STATE_UP;
        lift_set_target(LIFT_TARGET_DEG_UP_SEQ);
        g_lift_handle.lift_fsm.step = 4;
    }
}

// 上升步骤5：等待DM电机上升到位并完成序列
static void lift_up_step_wait_up_arrived_and_finish(void) {
    if (dm_position_check(LIFT_STATE_UP)) {
        log_message(LOG_INFO, "dm arrived");
        lift_publish_if_auto(1);
        lift_finish_sequence();
    }
}

// 下降步骤1：等待后光电下降沿 或 VL53L1(handle3) 距离 >= 150mm
static void lift_down_step_wait_rear_release(void) {
    bool vl53l1_triggered = false;
    uint16_t cur_dist_mm = 0;
    if (vl53l1_apply_get_distance_mm(&cur_dist_mm, g_vl53l1_handle3)) {
        if (cur_dist_mm >= 150) {
            vl53l1_triggered = true;
        }
    }

    if (get_rear_photoelectric_falling_edge() || vl53l1_triggered) {
        log_message(LOG_INFO, "chassis down");
        lift_publish_if_auto(0);
        g_lift_handle.lift_fsm.step = 1;
    }
}

// 下降步骤2：设置DM电机下降到初始位置
static void lift_down_step_set_down_target(void) {
    g_lift_handle.lift_state = LIFT_STATE_DOWN;
    lift_set_target(-LIFT_TARGET_DEG_DOWN_SEQ);
    g_lift_handle.lift_fsm.step = 2;
}

// 下降步骤3：等待DM电机下降到位
static void lift_down_step_wait_down_arrived(void) {
    if (dm_position_check(LIFT_STATE_DOWN)) {
        g_lift_handle.lift_fsm.step = 3;
    }
}

// 下降步骤4：2006电机反转，等待中光电下降沿 或 VL53L1 距离跳变
static void lift_down_step_drive_2006_backward(void) {
    /* 记录 step3 开始 tick（只在首次进入时记录） */
    if (g_lift_handle.step3_start_tick == 0) {
        g_lift_handle.step3_start_tick = xTaskGetTickCount();
    }

    /* 计算自 step3 开始经过的秒数 */
    uint32_t now_tick = xTaskGetTickCount();
    float elapsed_sec = (now_tick - g_lift_handle.step3_start_tick) *
                        (portTICK_PERIOD_MS * 0.001f);

    /* 2006 转速：up_R1_flag 为 true 时恒定 -1500rpm，否则三段变速：
       0 ~ TIME_DURATION_FRONT: 低速, TIME_DURATION_FRONT ~ TIME_DURATION_REAR: 高速, > TIME_DURATION_REAR: 低速 */
    if (up_R1_flag) {
        g_lift_handle.target_2006_rpm = -1500.0f;
    } else if (elapsed_sec < TIME_DURATION_FRONT) {
        g_lift_handle.target_2006_rpm = -LIFT_2006_LOW_SPEED;
    } else if (elapsed_sec < TIME_DURATION_REAR) {
        g_lift_handle.target_2006_rpm = -LIFT_2006_HIGH_SPEED;
    } else {
        g_lift_handle.target_2006_rpm = -LIFT_2006_LOW_SPEED;
    }

    /* ---- VL53L1 距离检测：距离 >= 150mm 时触发 ---- */
    bool vl53l1_triggered = false;
    uint16_t cur_dist_mm = 0;
    if (vl53l1_apply_get_distance_mm(&cur_dist_mm, g_vl53l1_handle)) {
        if (cur_dist_mm >= 150) {
            vl53l1_triggered = true;
        }
    }

    /* 检查中光电的下降沿 或 VL53L1 距离 >= 150mm，完成时清除开始 tick 并前进步骤 */
    if (get_middle_photoelectric_falling_edge() || vl53l1_triggered) {
        /* 记录本次耗时供调试（可选） */
        float total_time = elapsed_sec;
        g_lift_handle.step3_last_duration = total_time;
        g_lift_handle.step3_start_tick = 0;

        // log_message(LOG_INFO, "lift up, step duration=%.3fs, vl53l1_trig=%d",
        //             total_time, vl53l1_triggered);
        g_lift_handle.target_2006_rpm = 0.0f;
        g_lift_handle.lift_state = LIFT_STATE_UP;
        lift_set_target(LIFT_TARGET_DEG_UP_SEQ);
        g_lift_handle.lift_fsm.step = 4;
    }
}

// 下降步骤5：等待DM电机上升到位并完成序列
static void lift_down_step_wait_up_arrived_and_finish(void) {
    if (dm_position_check(LIFT_STATE_UP)) {
        lift_publish_if_auto(1);
        lift_finish_sequence();
    }
}
