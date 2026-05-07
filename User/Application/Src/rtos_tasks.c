/**
 * @file    rtos_tasks.c
 * @author  Deadline039
 * @brief   RTOS tasks.
 * @version 1.0
 * @date    2024-01-31
 */

#include "includes.h"

static void log_task_stack_usage(TaskHandle_t task_handle,
                                 const char *task_name,
                                 configSTACK_DEPTH_TYPE stack_words);
static void log_system_heap_summary(void);

static TaskHandle_t start_task_handle;
void start_task(void *pvParameters);

static TaskHandle_t task1_handle;
void task1(void *pvParameters);

static TaskHandle_t nav_task_handle;
void nav_task(void *pvParameters);

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

    if (xTaskCreate(nav_task, "nav_task", 128 * 10, NULL, 3,
                    &nav_task_handle) != pdPASS) {
        log_message(LOG_ERROR, "start_task: nav_task create failed");
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

/**
 * @brief Task1: Blink.
 *
 * @param pvParameters Start parameters.
 */
void task1(void *pvParameters) {
    UNUSED(pvParameters);
    LED0_OFF();
    LED1_ON();

    // uint8_t count = 0;

    while (1) {
        LED0_TOGGLE();
        LED1_TOGGLE();

        /* Print stack usage every 5 seconds to avoid flooding the log output. */
        // if (++count >= 5) {
        //     count = 0;
        //     // log_task_stack_usage(task1_handle, "task1", 256);
        //     log_task_stack_usage(nav_task_handle, "nav_task", 128*10);
        //     log_system_heap_summary();
        // }

        vTaskDelay(1000);
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