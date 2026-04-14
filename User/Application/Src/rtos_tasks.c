/**
 * @file    rtos_tasks.c
 * @author  Deadline039
 * @brief   RTOS tasks.
 * @version 1.0
 * @date    2024-01-31
 */

#include "includes.h"

static TaskHandle_t start_task_handle;
void start_task(void *pvParameters);

static TaskHandle_t task1_handle;
void task1(void *pvParameters);

static TaskHandle_t arm_ctrl_task_handle;
void arm_ctrl_task(void *pvParameters);

static TaskHandle_t nav_task_handle;
void nav_task(void *pvParameters);

RobotArm g_robot_arm;

/* Simple target point type used by the test sequence */
typedef struct {
    float y;
    float z;
    float pitch;
} target_point_t;

/* Four test points: origin + three targets */
static const target_point_t target_points[4] = {
    {139.95 + 20.0, 102.70, 0.9155f},   /* zero point */
    {450.0f,  356.0f, -PI/2.0},  
    {500.0f,  300.0f, 0.0f},   
    {-450.0f,  356.0f, -PI/2.0},   
};

/*****************************************************************************/

/**
 * @brief FreeRTOS start up.
 *
 */
void freertos_start(void) {
    xTaskCreate(start_task, "start_task", 256, NULL, 2, &start_task_handle);
    vTaskStartScheduler();
}

/**
 * @brief Start up task.
 *
 * @param pvParameters Start parameters.
 */
void start_task(void *pvParameters) {
    UNUSED(pvParameters);
    taskENTER_CRITICAL();

    microros_init();
    log_init(LOG_DEBUG);
    chassis_init();
    catch_init();
    msg_process_init();
    // robot_arm_system_init(&g_robot_arm);

    xTaskCreate(task1, "task1", 128, NULL, 2, &task1_handle);
    xTaskCreate(arm_ctrl_task, "arm_ctrl_task", 512, NULL, 2,
                &arm_ctrl_task_handle);

    if (xTaskCreate(nav_task, "nav_task", 128*10, NULL, 3, &nav_task_handle) !=
        pdPASS) {
        Error_Handler();
    }

    vTaskDelete(NULL);
    taskEXIT_CRITICAL();
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

    while (1) {
        LED0_TOGGLE();
        LED1_TOGGLE();
        vTaskDelay(1000);
    }
}

/**
 * @brief arm_ctrl_task: Control the robot arm to move to target points in sequence.
 *
 * @param pvParameters Start parameters.
 */
void arm_ctrl_task(void *pvParameters) {
    UNUSED(pvParameters);

    key_press_t key = KEY_NO_PRESS;
    uint8_t index = 0;

    robot_arm_set_target(&g_robot_arm, target_points[index].y,
                          target_points[index].z,
                          target_points[index].pitch);
    g_robot_arm.arm_motion_active = 0;

    while (1) {
        key = key_scan(0);
        switch (key) {
            case WKUP_PRESS: {
                index = (index + 1) % 4;
                robot_arm_set_target(&g_robot_arm, target_points[index].y,
                                     target_points[index].z,
                                     target_points[index].pitch);

                joint_target[0] = target_points[index].y;
                joint_target[1] = target_points[index].z;
                joint_target[2] = target_points[index].pitch;

                g_robot_arm.arm_motion_active = 0;
            } break;

            default:
                break;
        }

        robot_arm_update(&g_robot_arm);
        vTaskDelay(10);
    }
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