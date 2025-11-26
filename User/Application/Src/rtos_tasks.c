/**
 * @file    rtos_tasks.c
 * @author  Deadline039
 * @brief   RTOS tasks.
 * @version 1.0
 * @date    2024-01-31
 */

#include "includes.h"

#define RAD_DEG 57.324

#define damiao_debug 0      //达妙电机调试
#define unitree1_debug 0    //unitree1电机调试
#define unitree2_debug 0    //unitree2电机调试

static TaskHandle_t start_task_handle;
void start_task(void *pvParameters);

static TaskHandle_t task1_handle;
void task1(void *pvParameters);

static TaskHandle_t task2_handle;
void task2(void *pvParameters);

static TaskHandle_t arm_ctrl_handle;
void arm_ctr_task(void *pvParameters);

static TaskHandle_t msg_rec_handle;
void msg_rec_task(void *pvParameters);

#if damiao_debug
static TaskHandle_t damiao_debug_handle;
void damiao_debug_task(void *pvParameters);
#endif

#if unitree1_debug
static TaskHandle_t unitree1_debug_handle;
void unitree1_debug_task(void *pvParameters);
#endif

#if unitree2_debug
static TaskHandle_t unitree2_debug_handle;
void unitree2_debug_task(void *pvParameters);
#endif

unitree_motor_handle_t motor1 = {0};
unitree_motor_handle_t motor2 = {0};

dm_handle_t damiao = {0};
float dm_pos = 0.0;
float dm_pos_update = 0.0;

ctrl_param_t param = {0, 1, 0, 0, 0, 3, 0.2};
ctrl_param_t param2 = {1, 1, 0, 0, 0, 3, 0.2};

Trajectory t1 = {.state = FINISHED};
Trajectory t2 = {.state = FINISHED};

float des_debug[3] = {0.0, 0.0, 0.0};
float end_pos[3] = {0.0, 0.0, 0.0};
float angle[3] = {0.0, 0.0, 0.0};
/*****************************************************************************/

/**
 * @brief FreeRTOS start up.
 *
 */
void freertos_start(void) {
    xTaskCreate(start_task, "start_task", 128, NULL, 2, &start_task_handle);
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

    xTaskCreate(task1, "task1", 128, NULL, 2, &task1_handle);
    xTaskCreate(task2, "task2", 128, NULL, 3, &task2_handle);
    xTaskCreate(arm_ctr_task, "arm_ctr_task", 128, NULL, 3, &arm_ctrl_handle);
    xTaskCreate(msg_rec_task, "msg_rec_task", 128, NULL, 3, &msg_rec_handle);

#if damiao_debug
    xTaskCreate(damiao_debug_task, "damiao_debug_task", 128, NULL, 3, &damiao_debug_handle);
#endif

#if unitree1_debug
    xTaskCreate(unitree1_debug_task, "unitree1_debug_task", 128, NULL, 3, &unitree1_debug_handle);
#endif

#if unitree2_debug
    xTaskCreate(unitree2_debug_task, "unitree2_debug_task", 128, NULL, 3, &unitree2_debug_handle);
#endif

    can1_init(1000, 350);
    can2_init(1000, 350);
    can_list_add_can(can1_selected, 4, 4);
    can_list_add_can(can2_selected, 4, 4);
    rs_list_init(2);
    unitree_motor_init(&motor1, 0, 1);
    unitree_motor_init(&motor2, 1, 1);
    dm_motor_init(&damiao, 0x11, 0x01, DM_MODE_MIT, DM_G6220, 12.5, 45, 10,
                  can1_selected);
    dm_motor_enable(&damiao);

    // vTaskSuspend(arm_ctrl_handle);
    // vTaskSuspend(msg_rec_handle);
    
    vTaskDelete(start_task_handle);
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
        vTaskDelay(2000);
    }
}

/**
 * @brief Task2: print running time.
 *
 * @param pvParameters Start parameters.
 */
void task2(void *pvParameters) {
    UNUSED(pvParameters);

    while (1) {
        // current_angle = motor1.Pos;
        // difference = param.Pos - current_angle;
        // for (i = 1; i <= 500; i++) {
        //     param1 = param;
        //     param1.Pos = current_angle + difference/500.0*i;
        //     unitree_send_data(&usart1_handle, &motor1, param1);
        //     vTaskDelay(2);
        //     unitree_receive_data(&usart1_handle);
        // }
        // vTaskDelay(500);

        t_trajectory_update(&t1, &param.Pos, &param.W);
        unitree_send_data(&usart1_handle, &motor1, param);
        vTaskDelay(2);
        t_trajectory_update(&t2, &param2.Pos, &param2.W);
        unitree_send_data(&usart1_handle, &motor2, param2);
        vTaskDelay(3);

        // uint8_t ret = arm_get_end_position(&damiao, &motor1, &motor2, end_pos);
        arm_inverse_solution(angle, &end_pos[0], &end_pos[1], &end_pos[2]);
        // dm_mit_ctrl(&damiao, angle, 0.0f, 1.8f, 0.1f, 0.0f);
        }
}

void arm_ctr_task(void *pvParameters) {
    UNUSED(pvParameters);

    // float angle[3];
    float delta = 0.0f;
    float step = 0.0f;
    key_press_t key = KEY_NO_PRESS;

    while (1) {
        key = key_scan(0);
        switch (key){
            // 回到原点
            case WKUP_PRESS: {
                param.Pos = 0.0;
                param.W = 0;
                param.K_P = 3;
                param.K_W = 0.2;
                param.T = 0.0;

                param2.Pos = 0.0;

                dm_pos = 0.0;

                des_debug[0] = 0.0;
                des_debug[1] = 0.0;
                des_debug[2] = 0.0;

                t_trajectory_init(&t1, motor1.Pos, param.Pos, 10, 20, 0.005);
                t_trajectory_init(&t2, motor2.Pos, param2.Pos, 10, 20, 0.005);
            } break;

            // 两臂垂直，就绪态
            // case KEY1_PRESS: {
            //     param.Pos = -8.1347;
            //     param.W = 0;
            //     param.K_P = 3;
            //     param.K_W = 0.2;
            //     param.T = 0.0;

            //     param2.Pos = 0.0;

            //     dm_pos = 0.0;

            //     t_trajectory_init(&t1, motor1.Pos, param.Pos, 10, 20, 0.005);
            //     t_trajectory_init(&t2, motor2.Pos, param2.Pos, 10, 20, 0.005);
            //     dm_mit_ctrl(&damiao, dm_pos, 0.0, 15.0, 0.01, 0.0);
            // } break;

            case KEY0_PRESS: {
                des_debug[0] -= 100;
                arm_ctrl(des_debug[0], des_debug[1], des_debug[2], angle);
                param.Pos = -angle[2] * 6.33;
                param2.Pos = -angle[1] * 6.33;

                param.W = 0;
                param.K_P = 3;
                param.K_W = 0.2;
                param.T = 0.0;

                dm_pos = angle[0];

                t_trajectory_init(&t1, motor1.Pos, param.Pos, 10, 20, 0.005);
                t_trajectory_init(&t2, motor2.Pos, param2.Pos, 10, 20, 0.005);
                // dm_mit_ctrl(&damiao, dm_pos, 0.0, 15.0, 0.01, 0.0);
            } break;

            // 机械臂运动到相应位置
            case KEY1_PRESS: {
                des_debug[1] -= 100;
                arm_ctrl(des_debug[0], des_debug[1], des_debug[2], angle);
                param.Pos = -angle[2];
                param2.Pos = -angle[1];

                param.W = 0;
                param.K_P = 3;
                param.K_W = 0.2;
                param.T = 0.0;

                param2.W = 0;
                param2.K_P = 3;
                param2.K_W = 0.2;
                param2.T = 0.0;

                dm_pos = angle[0];

                t_trajectory_init(&t1, motor1.Pos, param.Pos, 10, 20, 0.005);
                t_trajectory_init(&t2, motor2.Pos, param2.Pos, 10, 20, 0.005);
                // dm_mit_ctrl(&damiao, dm_pos, 0.0, 15.0, 0.01, 0.0);
            } break;

            case KEY2_PRESS: {
                des_debug[2] -= 100;
                arm_ctrl(des_debug[0], des_debug[1], des_debug[2], angle);
                param.Pos = -angle[2] * 6.33;
                param2.Pos = -angle[1] * 6.33;

                param.W = 0;
                param.K_P = 3;
                param.K_W = 0.2;
                param.T = 0.0;

                param2.W = 0;
                param2.K_P = 3;
                param2.K_W = 0.2;
                param2.T = 0.0;

                dm_pos = angle[0];

                t_trajectory_init(&t1, motor1.Pos, param.Pos, 10, 20, 0.005);
                t_trajectory_init(&t2, motor2.Pos, param2.Pos, 10, 20, 0.005);
                // dm_mit_ctrl(&damiao, dm_pos, 0.0, 15.0, 0.01, 0.0);
            } break;

            default: {
            } break;
        }

        // t_trajectory_init(&t1, motor1.Pos, param.Pos, 10, 20, 0.005);
        // t_trajectory_init(&t2, motor2.Pos, param2.Pos, 10, 20, 0.005);
        delta = dm_pos - dm_pos_update; 
        
        if (fabs(delta) > 0.01) { 
            step = (fabs(delta) > 0.01) ? 0.01 : fabs(delta);
            dm_pos_update += (delta > 0) ? step : -step;
        }

        dm_mit_ctrl(&damiao, dm_pos_update, 0.0, 60.0, 0.01, 0.0);

        vTaskDelay(5);
    }
}

void msg_rec_task(void *pvParameters) {
    UNUSED(pvParameters);

    while (1) {

        unitree_receive_data(&usart1_handle);
        // t_trajectory_init(&t1, motor1.Pos, param.Pos, 10, 20, 0.005);
        vTaskDelay(2);
    }
}

#if damiao_debug 
void damiao_debug_task(void *pvParameters) {
    UNUSED(pvParameters);

    key_press_t key = KEY_NO_PRESS;

    float delta = 0.0f;
    float step = 0.0f;

    while (1) {
        key = key_scan(0);

        switch (key)
        {
            case KEY1_PRESS: 
                dm_pos += 1.2f;
                break;
            
            case KEY2_PRESS: 
                dm_pos -= 1.2f;
                break;
            
            default:
                break;
        }

        delta = dm_pos - dm_pos_update; 
        
        if (fabs(delta) > 0.01) { 
            step = (fabs(delta) > 0.01) ? 0.01 : fabs(delta);
            dm_pos_update += (delta > 0) ? step : -step;
        }

        dm_mit_ctrl(&damiao, dm_pos_update, 0.0, 60.0, 0.01, 0.0);
        vTaskDelay(10);
    }
}
#endif

#if unitree1_debug 
void unitree1_debug_task(void *pvParameters) {
    UNUSED(pvParameters);

    key_press_t key = KEY_NO_PRESS;

    while (1) {
        key = key_scan(0);
        switch (key)
        {
            case KEY1_PRESS: {
                param.Pos += 0.5;
                param.W = 0;
                param.K_P = 3;
                param.K_W = 0.2;
                param.T = 0.0;
                t_trajectory_init(&t1, motor1.Pos, param.Pos, 10, 20, 0.005);
            } break;
            
            case KEY2_PRESS: {
                param.Pos -= 0.5;
                param.W = 0;
                param.K_P = 1;
                param.K_W = 0.2;
                param.T = 0.0;
                t_trajectory_init(&t1, motor1.Pos, param.Pos, 10, 20, 0.005);
            } break;
            
            default:
                break;
        }
        vTaskDelay(10);
    }
}
#endif

#if unitree2_debug 
void unitree2_debug_task(void *pvParameters) {
    UNUSED(pvParameters);
    key_press_t key = KEY_NO_PRESS;

    while (1) {
        key = key_scan(0);
        switch (key)
        {
            case WKUP_PRESS: {
                param2.Pos += 0.5;
                param2.W = 0;
                param2.K_P = 3;
                param2.K_W = 0.2;
                param2.T = 0.0;
                t_trajectory_init(&t2, motor2.Pos, param2.Pos, 10, 20, 0.005);
            } break;
            
            case KEY0_PRESS: {
                param2.Pos -= 0.5;
                param2.W = 0;
                param2.K_P = 3;
                param2.K_W = 0.2;
                param2.T = 0.0;
                t_trajectory_init(&t2, motor2.Pos, param2.Pos, 10, 20, 0.005);
            } break;
            
            default:
                break;
        }
        vTaskDelay(10);
    }
}
#endif

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