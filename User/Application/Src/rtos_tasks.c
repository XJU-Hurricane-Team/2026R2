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

    if (req_in->data == false) {
        switch (myarm.status) {
            case DEFAULT:
                arm_pos_angle(0.0, 0.0, 0.0, angle);
                param.Pos = -angle[2] * 6.33;
                param2.Pos = -angle[1] * 6.33;
                dm_pos = -angle[0];
                arm_angle_drive(traj, &myarm, param.Pos, param2.Pos, dm_pos);
                myarm.status = READY;
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
        arm_pos_angle(pos.x, pos.y, pos.z, angle);
        param.Pos = -angle[2] * 6.33;
        param2.Pos = -angle[1] * 6.33;
        dm_pos = -angle[0];
        myarm.status = CATCH;
        arm_angle_drive(traj, &myarm, param.Pos, param2.Pos, dm_pos);

        param.Pos = 0.0;
        param2.Pos = 0.0;
        dm_pos = 0.0;
        myarm.status = PLACE;
        arm_angle_drive(traj, &myarm, param.Pos, param2.Pos, dm_pos);

        arm_pos_angle(0.0, 0.0, 0.0, angle);
        param.Pos = -angle[2] * 6.33;
        param2.Pos = -angle[1] * 6.33;
        dm_pos = -angle[0];
        myarm.status = READY;
        arm_angle_drive(traj, &myarm, param.Pos, param2.Pos, dm_pos);

        res_in->success = true;
    }

    // Handle request message and set the response message values
}

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
    xTaskCreate(motor_ctr_task, "motor_ctr_task", 128, NULL, 3, &task2_handle);
    xTaskCreate(arm_ctr_task, "arm_ctr_task", 3000, NULL, 3, &arm_ctrl_handle);
    xTaskCreate(msg_rec_task, "msg_rec_task", 128, NULL, 3, &msg_rec_handle);

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

void msg_rec_task(void *pvParameters) {
    UNUSED(pvParameters);

    while (1) {

        unitree_receive_data(&usart1_handle);
        vTaskDelay(2);
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