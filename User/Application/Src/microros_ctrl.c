/**
 * @file    microros_ctrl.c
 * @author  whyyy
 * @brief   MicroROS 控制模块.
 * @version 1.0
 * @date    2026-04-7
 */

#include "cubemx.h"
#include "./usart_ex/usart_ex.h"
#include <string.h>

#include "logger/logger.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "MicroROSConfig.h"
#include "microros_ctrl.h"
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/string.h>

custom_nav_msgs__msg__SpeedHeading nav_pram;

static rclc_executor_t executor;
static rcl_node_t node;
static rclc_support_t support;
static rcl_allocator_t allocator;
static SemaphoreHandle_t microros_rcl_mutex;

// 导航模块句柄
static rcl_subscription_t subscriber;

// 日志模块句柄
static rcl_publisher_t log_msg_publisher;
static rcl_publisher_t log_data_publisher;
std_msgs__msg__String ros_log_msg;
std_msgs__msg__Float32MultiArray ros_log_data;
static char ros_string_buffer[LOG_MSG_BUFFER_SIZE];
static float ros_data_buffer[LOG_DATA_COUNT + 1];
log_msg_packet_t log_msg_packet = {0};
log_data_packet_t log_data_packet = {0};

void nav_module_init(void);
void nav_module_callback(const void *msgin);
void logger_module_init(void);
void microros_log_msg_cb(const char *data, uint16_t len);
void microros_log_data_cb(const log_data_packet_t *packet);

/**
 * @brief 初始化MicroROS
 * 
 * @param pvParameters 
 */

int microros_init(void) {
    rcl_ret_t ret;

    rmw_uros_set_custom_transport(
        true, (void *)&huart4, cubemx_transport_open, cubemx_transport_close,
        cubemx_transport_write, cubemx_transport_read);

    rcl_allocator_t freeRTOS_allocator =
        rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate = microros_allocate;
    freeRTOS_allocator.deallocate = microros_deallocate;
    freeRTOS_allocator.reallocate = microros_reallocate;
    freeRTOS_allocator.zero_allocate = microros_zero_allocate;

    if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
        printf("Error on default allocators (line %d)\n", __LINE__);
        return (int)RCL_RET_ERROR;
    }

    allocator = rcl_get_default_allocator();
    microros_rcl_mutex = xSemaphoreCreateMutex();
    if (microros_rcl_mutex == NULL) {
        printf("microros_init: create mutex failed\n");
        return (int)RCL_RET_BAD_ALLOC;
    }

    // rmw_options 设置 client key，确保每次启动 client key 不同以避免与之前的实例冲突
    __attribute__((section(".noinit"))) static uint32_t boot_index;
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    ret = rcl_init_options_init(&init_options, allocator);
    if (ret != RCL_RET_OK) {
        printf("microros_init: rcl_init_options_init failed, ret=%d\n",
               (int)ret);
        return (int)ret;
    }

    rmw_init_options_t *rmw_options =
        rcl_init_options_get_rmw_init_options(&init_options);
    if (rmw_options == NULL) {
        printf("microros_init: get rmw init options failed\n");
        return (int)RCL_RET_ERROR;
    }

    uint32_t max_ids = 10;
    if (boot_index >= max_ids) {
        boot_index = 0;
    } else {
        boot_index++;
        if (boot_index >= max_ids) {
            boot_index = 0;
        }
    }
    uint32_t base_key = 0x5851F420;
    ret = (rcl_ret_t)rmw_uros_options_set_client_key(base_key + boot_index,
                                                     rmw_options);
    if (ret != RCL_RET_OK) {
        printf("microros_init: set client key failed, ret=%d\n", (int)ret);
        return (int)ret;
    }

    // init
    ret = rclc_support_init_with_options(&support, 0, NULL, &init_options,
                                         &allocator);
    if (ret != RCL_RET_OK) {
        printf("microros_init: support init failed, ret=%d\n", (int)ret);
        return (int)ret;
    }

    ret = rclc_node_init_default(&node, "chassis_node", "", &support);
    if (ret != RCL_RET_OK) {
        printf("microros_init: node init failed, ret=%d\n", (int)ret);
        return (int)ret;
    }

    ret = rclc_executor_init(&executor, &support.context, 2, &allocator);
    if (ret != RCL_RET_OK) {
        printf("microros_init: executor init failed, ret=%d\n", (int)ret);
        return (int)ret;
    }

    logger_module_init();

    /* 注册日志的发送接收函数 */
    log_register_output_callback(microros_log_msg_cb, microros_log_data_cb);

    return (int)RCL_RET_OK;
}

void nav_task(void *pvParameters) {
    UNUSED(pvParameters);
    if (microros_init() != (int)RCL_RET_OK) {
        printf("nav_task: microros_init failed, task halted\n");
        vTaskDelete(NULL);
        return;
    }
    nav_module_init();

    while (1) {
        // EXECUTE_EVERY_MS(50, t_nav, {
        //     if (microros_rcl_mutex != NULL &&
        //         xSemaphoreTake(microros_rcl_mutex, pdMS_TO_TICKS(10)) ==
        //             pdTRUE) {
        //         rclc_executor_spin_some(&executor, 0);
        //         xSemaphoreGive(microros_rcl_mutex);
        //     }
        // });

        if (microros_rcl_mutex != NULL &&
            xSemaphoreTake(microros_rcl_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            rclc_executor_spin_some(&executor, 5000000); /* 5ms */
            xSemaphoreGive(microros_rcl_mutex);
        }
        vTaskDelay(10);
    }
}

// 初始化导航接收模块
void nav_module_init(void) {
    rcl_ret_t ret = rclc_subscription_init_default(
        &subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(custom_nav_msgs, msg, SpeedHeading),
        "/nav_speed_heading_data");
    if (ret != RCL_RET_OK) {
        printf("nav_module_init: subscription init failed, ret=%d\n", (int)ret);
        return;
    }

    ret = rclc_executor_add_subscription(&executor, &subscriber, &nav_pram,
                                         &nav_module_callback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) {
        printf("nav_module_init: add subscription failed, ret=%d\n", (int)ret);
    }
}

void nav_module_callback(const void *msgin) {
    // Cast received message to used type
    const custom_nav_msgs__msg__SpeedHeading *msg =
        (const custom_nav_msgs__msg__SpeedHeading *)msgin;

    nav_pram.linear_x = msg->linear_x;
    nav_pram.linear_y = msg->linear_y;
    nav_pram.angular_z = msg->angular_z;
}

// 初始化日志模块
void logger_module_init(void) {
    rclc_publisher_init_default(
        &log_msg_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/log/msg");

    rclc_publisher_init_default(
        &log_data_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/log/data");

    ros_log_msg.data.data = ros_string_buffer;
    ros_log_msg.data.capacity = sizeof(ros_string_buffer) / sizeof(char);
    ros_log_msg.data.size = 0;

    ros_log_data.data.data = ros_data_buffer;
    ros_log_data.data.capacity = sizeof(ros_data_buffer) / sizeof(float);
    ros_log_data.data.size = 0;
}

/**
 * @brief MicroROS 字符串日志发送回调
 */
void microros_log_msg_cb(const char *data, uint16_t len) {
    size_t copy_len = len;
    if (copy_len >= ros_log_msg.data.capacity) {
        copy_len = ros_log_msg.data.capacity - 1;
    }

    memcpy(ros_log_msg.data.data, data, copy_len * sizeof(char));
    ros_log_msg.data.data[copy_len] = '\0';
    ros_log_msg.data.size = copy_len;

    if (microros_rcl_mutex != NULL &&
        xSemaphoreTake(microros_rcl_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        rcl_ret_t pub_ret = rcl_publish(&log_msg_publisher, &ros_log_msg, NULL);
        (void)pub_ret;
        xSemaphoreGive(microros_rcl_mutex);
    }
}

/**
 * @brief MicroROS 数据日志发送回调
 */
void microros_log_data_cb(const log_data_packet_t *packet) {
    /* 1. 总发送长度 = 1个ID头 + 真实数据个数 */
    size_t copy_len = 1 + packet->count;

    if (copy_len > ros_log_data.data.capacity) {
        copy_len = ros_log_data.data.capacity;
    }

    if (copy_len > 1) {
        ros_log_data.data.data[0] = (float)packet->id;
        memcpy(&ros_log_data.data.data[1], packet->data,
               (copy_len - 1) * sizeof(float));

        ros_log_data.data.size = copy_len;

        if (microros_rcl_mutex != NULL &&
            xSemaphoreTake(microros_rcl_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {

            rcl_ret_t pub_ret =
                rcl_publish(&log_data_publisher, &ros_log_data, NULL);
            (void)pub_ret;

            xSemaphoreGive(microros_rcl_mutex);
        }
    }
}
