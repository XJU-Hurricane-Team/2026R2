/**
 * @file    rtos_tasks.c
 * @author  Deadline039
 * @brief   RTOS tasks.
 * @version 1.0
 * @date    2024-01-31
 */

#include "includes.h"
#include "vl53l1/vl53l1_apply.h"
#include "arm_ctrl.h"
#include "ws2812/ws2812.h"
static void log_task_stack_usage(TaskHandle_t task_handle,
                                 const char *task_name,
                                 configSTACK_DEPTH_TYPE stack_words);
static void log_system_heap_summary(void);

static TaskHandle_t start_task_handle;
void start_task(void *pvParameters);

static TaskHandle_t task1_handle;
void task1(void *pvParameters);

static TaskHandle_t task2_handle;
void task2(void *pvParameters);
                                  
static TaskHandle_t microros_task_handle;
void microros_task(void *pvParameters);

/*****************************************************************************/

/**
 * @brief FreeRTOS start up.
 *
 */
void freertos_start(void) {
    xTaskCreate(start_task, "start_task", 128 * 10, NULL, 2,
                &start_task_handle);
    vTaskStartScheduler();
}

/**
 * @brief Start up task.
 *
 * @param pvParameters Start parameters.
 */
void start_task(void *pvParameters) {
    UNUSED(pvParameters);

    log_init(LOG_DEBUG);
    can_list_add_can(can1_selected, 4, 4);
    can_list_add_can(can2_selected, 4, 4);
    can_list_add_can(can3_selected, 4, 4);

    if (microros_init() != 0) {
        log_message(LOG_ERROR, "start_task: microros_init failed");
        Error_Handler();
    }

    logger_module_init();

    if (xTaskCreate(microros_task, "microros_task", 128 * 8, NULL, 3,
                    &microros_task_handle) != pdPASS) {
        log_message(LOG_ERROR, "start_task: microros_task create failed");
        Error_Handler();
    }

    chassis_init();
    catch_init();
    msg_process_init();
    robot_arm_init();
    

    if (xTaskCreate(task1, "task1", 256, NULL, 2, &task1_handle) != pdPASS) {
        log_message(LOG_ERROR, "start_task: task1 create failed");
        Error_Handler();
    }

    vTaskDelete(NULL);
}
// uint16_t dist1 = 0;
// uint16_t dist2 = 0;

/* VL53L1 距离跳变测试——统计 15~30cm 跳变次数 */
// static uint16_t s_test_last_dist_mm = 0;
// static bool     s_test_has_last = false;
// static uint32_t s_test_delta_trigger_count = 0;  /* 触发次数计数器 */
// uint16_t cur_dist_mm2 = 0;
// uint16_t cur_dist_mm = 0;
/**
 * @brief Task1: PA1 层数切换 + WS2812 层数反馈
 */
void task1(void *pvParameters) {
    UNUSED(pvParameters);
    LED0_OFF();

    uint8_t last_layer = 0xFF; /* 上一次显示的层数，用于减少WS2812刷新 */
    uint8_t pa1_last = 1;      /* PA1 上一次电平（上拉默认高） */

    while (1) {
        LED0_TOGGLE();

        /* PA1 下降沿检测：层数加1 */
        uint8_t pa1_now = (uint8_t)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
        if (pa1_last == 1 && pa1_now == 0) {
            uint8_t layer = robot_arm_get_layer_count();
            layer = (layer + 1) % 4;
            robot_arm_set_layer_count(layer);
        }
        pa1_last = pa1_now;

        /* WS2812 灯带反馈当前已放置数量 */
        uint8_t layer = robot_arm_get_layer_count();
        if (layer != last_layer) {
            last_layer = layer;
            uint8_t led_count = layer * 7; /* 0→0, 1→7, 2→14, 3→21 */
            uint32_t color;
            if (layer == 0) {
                color = COLOR_OFF;
            } else if (layer == 1) {
                color = COLOR_GREEN;
            } else if (layer == 2) {
                color = COLOR_YELLOW;
            } else {
                color = COLOR_RED;
            }
            for (int i = 0; i < NUM_LEDS; i++) {
                WS2812_SetColor(i, (i < led_count) ? color : COLOR_OFF);
            }
            WS2812_Send();
        }

        vTaskDelay(50);
    }
}

static void log_task_stack_usage(TaskHandle_t task_handle,
                                 const char *task_name,
                                 configSTACK_DEPTH_TYPE stack_words) {
    if ((task_handle == NULL) || (task_name == NULL) || (stack_words == 0U)) {
        return;
    }

    UBaseType_t free_words = uxTaskGetStackHighWaterMark(task_handle);
    if (free_words > (UBaseType_t)stack_words) {
        free_words = (UBaseType_t)stack_words;
    }

    UBaseType_t used_words = (UBaseType_t)stack_words - free_words;
    UBaseType_t free_bytes = free_words * sizeof(StackType_t);

    log_message(LOG_INFO, "[Stack] %s min_free=%lu bytes used=%lu bytes",
                task_name, (unsigned long)free_bytes, (unsigned long)(used_words * sizeof(StackType_t)));
}

static void log_system_heap_summary(void) {
    // 获取当前剩余堆内存（字节）
    size_t current_free_heap = xPortGetFreeHeapSize();
    
    // 获取历史最低剩余堆内存（字节）
    size_t min_ever_free_heap = xPortGetMinimumEverFreeHeapSize();

    log_message(LOG_INFO, 
                "[Heap Summary] Current Free: %u bytes | Min Free: %u bytes", 
                (unsigned int)current_free_heap, 
                (unsigned int)min_ever_free_heap);
}

#ifdef configASSERT
/**
 * @brief FreeRTOS assert failed function. 
 * 
 * @param pcFile File name
 * @param ulLine File line
 */
void vAssertCalled(const char *pcFile, unsigned int ulLine) {
    fprintf(stderr, "FreeRTOS assert failed. File: %s, line: %u. \n", pcFile,
            ulLine);
}
#endif /* configASSERT */

#if configCHECK_FOR_STACK_OVERFLOW
/**
 * @brief The application stack overflow hook is called when a stack overflow is detected for a task.
 *
 * @param xTask the task that just exceeded its stack boundaries.
 * @param pcTaskName A character string containing the name of the offending task.
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    UNUSED(xTask);
    fprintf(stderr, "Stack overflow! Taskname: %s. \n", pcTaskName);
}
#endif /* configCHECK_FOR_STACK_OVERFLOW */

#if configUSE_MALLOC_FAILED_HOOK
/**
 * @brief This hook function is called when allocation failed.
 * 
 */
void vApplicationMallocFailedHook(void) {
    fprintf(stderr, "FreeRTOS malloc failed! \n");
}
#endif /* configUSE_MALLOC_FAILED_HOOK */