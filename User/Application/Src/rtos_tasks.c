/**
 * @file    rtos_tasks.c
 * @author  Deadline039
 * @brief   RTOS tasks.
 * @version 1.0
 * @date    2024-01-31
 */

#include "includes.h"

#define RAD_DEG 57.324

static TaskHandle_t start_task_handle;
void start_task(void *pvParameters);

static TaskHandle_t task1_handle;
void task1(void *pvParameters);

static TaskHandle_t task2_handle;
void motor_ctr_task(void *pvParameters);

static TaskHandle_t arm_ctrl_handle;
void arm_ctr_task(void *pvParameters);

static TaskHandle_t msg_rec_handle;
void msg_rec_task(void *pvParameters);

static TaskHandle_t arm_sequence_handle;
void arm_sequence_task(void *pvParameters);

EventGroupHandle_t arm_event_group;

unitree_motor_handle_t motor1 = {0};
unitree_motor_handle_t motor2 = {0};
dm_handle_t damiao = {0};

ctrl_param_t param = {0, 1, 0, 0, 0, 3, 0.2};
ctrl_param_t param2 = {1, 1, 0, 0, 0, 3, 0.2};
float dm_pos = 0.0;
float dm_w = 0.0;

// 三电机的速度规划曲线结构体
Trajectory_Handler_t traj[3] = {[0 ... 2] = {.state = FINISHED}};
geometry_msgs__msg__Point pos;
float angle[3] = {0.0, 0.0, 0.0};

arm_handle_t myarm = {0};

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

    arm_event_group = xEventGroupCreate();
    xTaskCreate(task1, "task1", 128, NULL, 2, &task1_handle);
    xTaskCreate(motor_ctr_task, "motor_ctr_task", 128, NULL, 3, &task2_handle);
    xTaskCreate(arm_ctr_task, "arm_ctr_task", 3000, NULL, 3, &arm_ctrl_handle);
    xTaskCreate(msg_rec_task, "msg_rec_task", 128, NULL, 3, &msg_rec_handle);
    xTaskCreate(arm_sequence_task, "arm_sequence_task", 256, NULL, 4,
                &arm_sequence_handle);

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

    geometry_msgs__msg__Point__init(&pos);

    myarm.motor1 = &motor1;
    myarm.motor2 = &motor2;
    myarm.motor3 = &damiao;
    myarm.status = DEFAULT;

    vTaskDelete(start_task_handle);
    taskEXIT_CRITICAL();
}

void subscription_callback(const void *msgin) {
    // Cast received message to used type
    const geometry_msgs__msg__Point *msg =
        (const geometry_msgs__msg__Point *)msgin;

    pos.x = msg->x;
    pos.y = msg->y;
    pos.z = msg->z;
}

void service_callback(const void *request_msg, void *response_msg) {
    // Cast messages to expected types
    std_srvs__srv__SetBool_Request *req_in =
        (std_srvs__srv__SetBool_Request *)request_msg;
    std_srvs__srv__SetBool_Response *res_in =
        (std_srvs__srv__SetBool_Response *)response_msg;

    res_in->success = false;

    if (req_in->data == false) {
        switch (myarm.status) {
            case DEFAULT:
                myarm.status = MOVING_TO_READY;
                xEventGroupSetBits(arm_event_group, EVENT_READY);
                res_in->success = false;
                break;
            case READY:
                res_in->success = true;
                break;
            default:
                res_in->success = false;
                break;
        }
    } else if (req_in->data == true) {
        if (myarm.status == READY) {
            // arm_pos_angle(pos.x, pos.y, pos.z, angle);
            myarm.status = CATCH;
            xEventGroupSetBits(arm_event_group, EVENT_CATCH);
            res_in->success = true;
        } else {
            res_in->success = false;
        }
    }

    // Handle request message and set the response message values
}

void arm_sequence_task(void *pvParameters) {
    UNUSED(pvParameters);
    EventBits_t uxBits;

    while (1) {

        // 等待事件标志位
        uxBits = xEventGroupWaitBits(
            arm_event_group, EVENT_READY | EVENT_CATCH | EVENT_TRAJ_FINISHED,
            pdTRUE,       // 接收到后清除标志位
            pdFALSE,      // 等待任意一个标志位即可
            portMAX_DELAY // 永久阻塞，直到接收到事件
        );

        // 恢复准备态
        if (uxBits & EVENT_READY) {
            arm_pos_angle(0.0, 0.0, 0.0, angle);
            param.Pos = -angle[2] * 6.33;
            param2.Pos = -angle[1] * 6.33;
            dm_pos = -angle[0];
            arm_angle_drive(traj, &myarm, param.Pos, param2.Pos, dm_pos);
        }

        // 准备态到抓取态
        else if (uxBits & EVENT_CATCH) {
            arm_pos_angle(pos.x, pos.y, pos.z, angle);
            param.Pos = -angle[2] * 6.33;
            param2.Pos = -angle[1] * 6.33;
            dm_pos = -angle[0];
            arm_angle_drive(traj, &myarm, param.Pos, param2.Pos, dm_pos);

        }
        
        // 抓取到放置
        else if (uxBits & EVENT_PLACE) {
            param.Pos = 0.0;
            param2.Pos = 0.0;
            dm_pos = 0.0;
            arm_angle_drive(traj, &myarm, param.Pos, param2.Pos, dm_pos);

        }

        // 轨迹完成后动作规划
        else if (uxBits & EVENT_TRAJ_FINISHED) {

            if (myarm.status == MOVING_TO_READY) {
                myarm.status = READY;
            } else if (myarm.status == CATCH) {
                // 抓取到放置
                myarm.status = PLACE;
                xEventGroupSetBits(arm_event_group, EVENT_PLACE);
            } else if (myarm.status == PLACE) {
                // 放置到准备
                myarm.status = MOVING_TO_READY;
                xEventGroupSetBits(arm_event_group, EVENT_READY);
            }
        }
    }
}

/**
 * @brief 
 * 
 * @param pvParameters 
 */
void arm_ctr_task(void *pvParameters) {
    UNUSED(pvParameters);
    int res = 0;

    rmw_uros_set_custom_transport(
        true, (void *)&usart3_handle, cubemx_transport_open,
        cubemx_transport_close, cubemx_transport_write, cubemx_transport_read);

    rcl_allocator_t freeRTOS_allocator =
        rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate = microros_allocate;
    freeRTOS_allocator.deallocate = microros_deallocate;
    freeRTOS_allocator.reallocate = microros_reallocate;
    freeRTOS_allocator.zero_allocate = microros_zero_allocate;

    if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
        printf("Error on default allocators (line %d)\n", __LINE__);
    }

    // micro-ROS app

    rcl_service_t service;
    rcl_subscription_t subscriber;
    rclc_executor_t executor; //执行器

    std_srvs__srv__SetBool_Request request_msg;
    std_srvs__srv__SetBool_Response response_msg;
    rclc_support_t support;
    rcl_allocator_t allocator;
    rcl_node_t node;

    allocator = rcl_get_default_allocator();

    //create init_options
    res |= rclc_support_init(&support, 0, NULL, &allocator);

    //create node
    res |= rclc_node_init_default(&node, "cubemx_node", "", &support);

    res |= rclc_executor_init(&executor, &support.context, 2, &allocator);

    res = rclc_service_init_default(
        &service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
        "arm_ctr_srv");
    if (res != RCL_RET_OK) {
        printf("rclc_init_default failed: %d\n", res);
        vTaskDelete(NULL);
        return;
    }

    // Initialize a reliable subscriber
    rclc_subscription_init_default(
        &subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point), "pos_sub");

    std_srvs__srv__SetBool_Request__init(&request_msg);
    std_srvs__srv__SetBool_Response__init(&response_msg);

    rclc_executor_add_subscription(&executor, &subscriber, &pos,
                                   &subscription_callback, ON_NEW_DATA);
    rclc_executor_add_service(&executor, &service, &request_msg, &response_msg,
                              service_callback);
    while (1) {
        rclc_executor_spin(&executor);
    }
}

/**
 * @brief Task2: print running time.
 *
 * @param pvParameters Start parameters.
 */
void motor_ctr_task(void *pvParameters) {
    UNUSED(pvParameters);

    while (1) {

        t_trajectory_update(&traj[0], &param.Pos, &param.W);
        unitree_send_data(&usart1_handle, &motor1, param);
        vTaskDelay(2);
        t_trajectory_update(&traj[1], &param2.Pos, &param2.W);
        unitree_send_data(&usart1_handle, &motor2, param2);
        vTaskDelay(3);
        t_trajectory_update(&traj[2], &dm_pos, &dm_w);
        dm_mit_ctrl(&damiao, dm_pos, 0.0, 30.0, 0.01, 0.0);
        if ((traj[0].state == FINISHED) && (traj[1].state == FINISHED) &&
            (traj[2].state == FINISHED) && (myarm.status != READY)) {
            xEventGroupSetBits(arm_event_group, EVENT_TRAJ_FINISHED);
        }
    }
}

void msg_rec_task(void *pvParameters) {
    UNUSED(pvParameters);

    while (1) {

        unitree_receive_data(&usart1_handle);
        vTaskDelay(2);
    }
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