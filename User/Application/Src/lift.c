/**
 * @file    lift.c
 * @author  Dominate0017
 * @brief   抬升与2006控制模块
 * @version 1.0
 * @date    2026-04-28
 */

#include "includes.h"
#include "microros_ctrl.h"
#include "lift.h"

#define CHASSIS_CAN_SELECT       can1_selected
#define LIFT_CAN_SELECT          can2_selected

#define LIFT_SEQ_UP_KEY          5 /* 上台阶按键 */
#define LIFT_SEQ_DOWN_KEY        6 /* 下台阶按键 */
#define LIFT_UP_KEY              7 /* 抬升升起按键 */
#define LIFT_DOWN_KEY            8 /* 抬升下降按键 */
#define LIFT_STOP_KEY            9 /* 中断序列并复位按键 */

#define SENSOR_GPIO_PORT         GPIOE
#define MIDDLE_SENSOR_PIN_1      GPIO_PIN_5
#define MIDDLE_SENSOR_PIN_0      GPIO_PIN_6
#define FRONT_SENSOR_PIN         GPIO_PIN_7
#define REAR_SENSOR_PIN          GPIO_PIN_8

#define LIFT_TARGET_DEG_MAX      7.93f
#define LIFT_TARGET_DEG_UP_SEQ   0.0f
#define LIFT_TARGET_DEG_DOWN_SEQ 7.93f
#define LIFT_TARGET_DEG_STEP     0.025f
#define LIFT_TARGET_SPEED        5.0f

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
    uint32_t change_tick;
    uint32_t debounce_ms;
} photoelectric_debounce_t;

typedef struct {
    bool is_auto_mode;
    float target_2006_rpm;

    lift_state_t lift_state;
    float lift_target_degree;
    lift_fsm_t lift_fsm;
} lift_handle_t;

static lift_handle_t g_lift_handle = {
    .is_auto_mode = false,
    .target_2006_rpm = 0.0f,
    .lift_state = LIFT_STATE_NORMAL,
    .lift_target_degree = 0.0f,
    .lift_fsm = {0, 0},
};

static photoelectric_debounce_t g_rear_photoelectric_debounce = {
    .current_state = false,
    .last_stable_state = false,
    .change_tick = 0,
    .debounce_ms = 10,
};

static photoelectric_debounce_t g_front_photoelectric_debounce = {
    .current_state = false,
    .last_stable_state = false,
    .change_tick = 0,
    .debounce_ms = 10,
};

static photoelectric_debounce_t g_middle_photoelectric_debounce = {
    .current_state = false,
    .last_stable_state = false,
    .change_tick = 0,
    .debounce_ms = 10,
};

static dji_motor_handle_t dji_2006_handle[2];
static dm_handle_t dm_motor_handle[2];
static pid_t dji_2006_pid[2];

static TaskHandle_t lift_state_task_handle;
static TaskHandle_t lift_sequence_task_handle;

static void lift_bottom_init(void);
static void lift_tasks_init(void);
static void lift_seq_start(uint8_t action);
static void lift_seq_update(void);
static void lift_seq_up_update(void);
static void lift_seq_down_update(void);

static bool get_rear_photoelectric(void);
static bool get_front_photoelectric(void);
static bool get_middle_photoelectric(void);
static bool dm_position_check(lift_state_t state);

static float lift_limit_target(float target_degree);
static void lift_set_target(float target_degree);
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

static void lift_state_task(void *pvParameters);
static void lift_sequence_task(void *pvParameters);

void lift_init(void) {
    lift_bottom_init();
    lift_tasks_init();

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
                g_lift_handle.target_2006_rpm = g_remote_ctrl_data.rs[3] * 250.0f;
            }

            switch (g_lift_handle.lift_state) {
                case LIFT_STATE_UP:
                    lift_set_target(g_lift_handle.lift_target_degree + LIFT_TARGET_DEG_STEP);
                    break;
                case LIFT_STATE_DOWN:
                    lift_set_target(g_lift_handle.lift_target_degree - LIFT_TARGET_DEG_STEP);
                    break;
                case LIFT_STATE_NORMAL:
                default:
                    break;
            }
        }

        for (int i = 0; i < 2; i++) {
            float real_2006_rpm = dji_2006_handle[i].speed_rpm;
            float target_rpm = (i == 0) ? g_lift_handle.target_2006_rpm : -g_lift_handle.target_2006_rpm;

            motor_2006_out[i] = (int16_t)pid_calc(&dji_2006_pid[i], target_rpm,
                                                  real_2006_rpm);
        }

        dji_motor_set_current(CHASSIS_CAN_SELECT, DJI_MOTOR_GROUP2,
                              motor_2006_out[0], motor_2006_out[1], 0, 0);

        dm_pos_speed_ctrl(&dm_motor_handle[0],
                          g_lift_handle.lift_target_degree,
                          LIFT_TARGET_SPEED);
        dm_pos_speed_ctrl(&dm_motor_handle[1],
                          -g_lift_handle.lift_target_degree,
                          LIFT_TARGET_SPEED);

        vTaskDelay(5);
    }
}

static void lift_sequence_task(void *pvParameters) {
    (void)pvParameters;
    while (1) {
        if (g_lift_handle.lift_fsm.action == 0) {
            (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            continue;
        }

        lift_seq_update();
        vTaskDelay(5);
    }
}

void lift_switch_mode(uint8_t key, remote_key_event_t event) {
    if (g_lift_handle.lift_fsm.action != 0 && key != LIFT_STOP_KEY) {
        return;
    }

    switch (key) {
        case LIFT_STOP_KEY:
            lift_seq_emergency_stop();
            break;

        case LIFT_SEQ_UP_KEY:
            log_message(LOG_INFO, "Trigger Lift UP sequence.");
            lift_seq_start(1);
            lift_publish_if_auto(1);
            break;

        case LIFT_SEQ_DOWN_KEY:
            log_message(LOG_INFO, "Trigger Lift DOWN sequence.");
            lift_seq_start(2);
            lift_publish_if_auto(2);
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

void lift_set_chassis_mode(bool is_auto_mode) {
    g_lift_handle.is_auto_mode = is_auto_mode;
    if (is_auto_mode && g_lift_handle.lift_fsm.action == 0) {
        g_lift_handle.target_2006_rpm = 0.0f;
    }
}

bool lift_is_sequence_running(void) {
    return (g_lift_handle.lift_fsm.action != 0);
}

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
        pid_init(&dji_2006_pid[i], 10000.0f, 500.0f, 0.0f, 15000.0f,
                 DELTA_PID, 1.80f, 0.01f, 0.00f);
    }
}

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
}

static void lift_seq_start(uint8_t action) {
    if (action != 1 && action != 2) {
        return;
    }

    g_lift_handle.lift_fsm.action = action;
    g_lift_handle.lift_fsm.step = 0;

    if (lift_sequence_task_handle != NULL) {
        xTaskNotifyGive(lift_sequence_task_handle);
    }
}

static float lift_limit_target(float target_degree) {
    if (fabs(target_degree) > LIFT_TARGET_DEG_MAX) {
        if (target_degree < 0) {
            return -LIFT_TARGET_DEG_MAX;
        }
        return LIFT_TARGET_DEG_MAX;
    }
    return target_degree;
}

static void lift_set_target(float target_degree) {
    g_lift_handle.lift_target_degree = lift_limit_target(target_degree);
}

static void lift_publish_if_auto(uint8_t code) {
    if (g_lift_handle.is_auto_mode) {
        stair_microros_publish(code);
    }
}

static void lift_finish_sequence(void) {
    g_lift_handle.lift_state = LIFT_STATE_NORMAL;
    g_lift_handle.lift_fsm.action = 0;
}

static void lift_seq_emergency_stop(void) {
    g_lift_handle.lift_state = LIFT_STATE_NORMAL;
    g_lift_handle.lift_target_degree = 0.0f;
    g_lift_handle.target_2006_rpm = 0.0f;
    g_lift_handle.lift_fsm.action = 0;
    g_lift_handle.lift_fsm.step = 0;

    log_message(LOG_INFO, "Lift sequence emergency stop: action=0, degree=0, 2006=0");
}

static void lift_up_step_wait_front_trigger(void) {
    if (get_front_photoelectric()) {
        log_message(LOG_INFO, "chassis up");
        lift_publish_if_auto(0);
        g_lift_handle.lift_fsm.step = 1;
    }
}

static void lift_up_step_set_down_target(void) {
    g_lift_handle.lift_state = LIFT_STATE_DOWN;
    lift_set_target(-LIFT_TARGET_DEG_DOWN_SEQ);
    g_lift_handle.lift_fsm.step = 2;
}

static void lift_up_step_wait_down_arrived(void) {
    if (dm_position_check(LIFT_STATE_DOWN)) {
        g_lift_handle.lift_fsm.step = 3;
    }
}

static void lift_up_step_drive_2006_forward(void) {
    g_lift_handle.target_2006_rpm = 1000.0f;
    vTaskDelay(500);
    if (get_rear_photoelectric()) {
        log_message(LOG_INFO, "2006 move");
        g_lift_handle.target_2006_rpm = 0.0f;
        g_lift_handle.lift_state = LIFT_STATE_UP;
        lift_set_target(LIFT_TARGET_DEG_UP_SEQ);
        g_lift_handle.lift_fsm.step = 4;
    }
}

static void lift_up_step_wait_up_arrived_and_finish(void) {
    if (dm_position_check(LIFT_STATE_UP)) {
        log_message(LOG_INFO, "dm arrived");
        lift_finish_sequence();
    }
}

static void lift_down_step_wait_rear_release(void) {
    if (!get_rear_photoelectric()) {
        log_message(LOG_INFO, "chassis down");
        lift_publish_if_auto(0);
        g_lift_handle.lift_fsm.step = 1;
    }
}

static void lift_down_step_set_down_target(void) {
    g_lift_handle.lift_state = LIFT_STATE_DOWN;
    lift_set_target(-LIFT_TARGET_DEG_DOWN_SEQ);
    g_lift_handle.lift_fsm.step = 2;
}

static void lift_down_step_wait_down_arrived(void) {
    if (dm_position_check(LIFT_STATE_DOWN)) {
        g_lift_handle.lift_fsm.step = 3;
    }
}

static void lift_down_step_drive_2006_backward(void) {
    g_lift_handle.target_2006_rpm = -1000.0f;
    if (!get_middle_photoelectric()) {
        vTaskDelay(500);
        g_lift_handle.target_2006_rpm = 0.0f;
        g_lift_handle.lift_state = LIFT_STATE_UP;
        lift_set_target(LIFT_TARGET_DEG_UP_SEQ);
        g_lift_handle.lift_fsm.step = 4;
    }
}

static void lift_down_step_wait_up_arrived_and_finish(void) {
    if (dm_position_check(LIFT_STATE_UP)) {
        lift_finish_sequence();
    }
}

typedef void (*lift_seq_handler_t)(void);
typedef void (*lift_step_handler_t)(void);

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

static void lift_seq_down_update(void) {
    static const lift_step_handler_t step_handlers[] = {
        lift_down_step_wait_rear_release,
        lift_down_step_set_down_target,
        lift_down_step_wait_down_arrived,
        lift_down_step_drive_2006_backward,
        lift_down_step_wait_up_arrived_and_finish,
    };

    if (g_lift_handle.lift_fsm.step <
            (sizeof(step_handlers) / sizeof(step_handlers[0])) &&
        step_handlers[g_lift_handle.lift_fsm.step] != NULL) {
        step_handlers[g_lift_handle.lift_fsm.step]();
    }
}

static bool dm_position_check(lift_state_t state) {
    bool motor0_ready = false;
    bool motor1_ready = false;

    if (state == LIFT_STATE_DOWN) {
        motor0_ready =
            (fabs(fabs(dm_motor_handle[0].position) - LIFT_TARGET_DEG_DOWN_SEQ) <
             0.015f);
        motor1_ready =
            (fabs(fabs(dm_motor_handle[1].position) - LIFT_TARGET_DEG_DOWN_SEQ) <
             0.015f);
        return motor0_ready && motor1_ready;
    }

    if (state == LIFT_STATE_UP) {
        motor0_ready =
            (fabs(fabs(dm_motor_handle[0].position) - LIFT_TARGET_DEG_UP_SEQ) <
             0.015f);
        motor1_ready =
            (fabs(fabs(dm_motor_handle[1].position) - LIFT_TARGET_DEG_UP_SEQ) <
             0.015f);
        return motor0_ready && motor1_ready;
    }

    return false;
}

static bool get_front_photoelectric(void) {
    bool raw_state = HAL_GPIO_ReadPin(SENSOR_GPIO_PORT, FRONT_SENSOR_PIN);

    if (raw_state != g_front_photoelectric_debounce.current_state) {
        g_front_photoelectric_debounce.current_state = raw_state;
        g_front_photoelectric_debounce.change_tick = xTaskGetTickCount();
    }

    uint32_t elapsed_ms =
        (xTaskGetTickCount() - g_front_photoelectric_debounce.change_tick) *
        portTICK_PERIOD_MS;
    if (elapsed_ms >= g_front_photoelectric_debounce.debounce_ms) {
        g_front_photoelectric_debounce.last_stable_state = raw_state;
    }

    return g_front_photoelectric_debounce.last_stable_state;
}

static bool get_rear_photoelectric(void) {
    bool raw_state = HAL_GPIO_ReadPin(SENSOR_GPIO_PORT, REAR_SENSOR_PIN);

    if (raw_state != g_rear_photoelectric_debounce.current_state) {
        g_rear_photoelectric_debounce.current_state = raw_state;
        g_rear_photoelectric_debounce.change_tick = xTaskGetTickCount();
    }

    uint32_t elapsed_ms =
        (xTaskGetTickCount() - g_rear_photoelectric_debounce.change_tick) *
        portTICK_PERIOD_MS;
    if (elapsed_ms >= g_rear_photoelectric_debounce.debounce_ms) {
        g_rear_photoelectric_debounce.last_stable_state = raw_state;
    }

    return g_rear_photoelectric_debounce.last_stable_state;
}

static bool get_middle_photoelectric(void) {
    bool raw_state = (HAL_GPIO_ReadPin(SENSOR_GPIO_PORT, MIDDLE_SENSOR_PIN_0) ||
                      HAL_GPIO_ReadPin(SENSOR_GPIO_PORT, MIDDLE_SENSOR_PIN_1));

    if (raw_state != g_middle_photoelectric_debounce.current_state) {
        g_middle_photoelectric_debounce.current_state = raw_state;
        g_middle_photoelectric_debounce.change_tick = xTaskGetTickCount();
    }

    uint32_t elapsed_ms =
        (xTaskGetTickCount() - g_middle_photoelectric_debounce.change_tick) *
        portTICK_PERIOD_MS;
    if (elapsed_ms >= g_middle_photoelectric_debounce.debounce_ms) {
        g_middle_photoelectric_debounce.last_stable_state = raw_state;
    }

    return g_middle_photoelectric_debounce.last_stable_state;
}
