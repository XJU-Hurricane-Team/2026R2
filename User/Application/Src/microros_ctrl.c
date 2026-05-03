/**
 * @file    microros_ctrl.c
 * @author  whyyy
 * @brief   MicroROS 控制模块.
 * @version 1.0
 * @date    2026-04-7
 */

#include "includes.h"

#include "MicroROSConfig.h"
#include "microros_ctrl.h"
#include "catch.h"
#include "lift.h"
#include "arm_ctrl.h"

#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/bool.h>
#include <std_srvs/srv/set_bool.h>
#include "custom_msg/srv/grab.h"
#include "geometry_msgs/msg/point.h"

static rclc_executor_t executor = {0};
static rcl_node_t node = {0};
static rclc_support_t support = {0};
static rcl_allocator_t allocator = {0};
static SemaphoreHandle_t microros_rcl_mutex = NULL;

// 导航，抓取模块
static rcl_subscription_t nav_subscriber = {0};
static rcl_service_t grab_service = {0};
static rcl_publisher_t grab_publisher = {0};
static rcl_publisher_t stair_publisher = {0};
custom_msg__msg__SpeedHeading nav_pram = {0};
static custom_msg__srv__Grab_Request grab_request = {0};
static custom_msg__srv__Grab_Response grab_response = {0};
static std_msgs__msg__Int8 grab_pub_pram = {0};
static std_msgs__msg__Int8 stair_pub_pram = {0};

// 机械臂模块
static arm_status_t arm_microros_state = ARM_STATE_INIT;
static geometry_msgs__msg__Point arm_target_cache = {0};
static bool arm_target_cache_valid = false;
static rcl_subscription_t arm_target_subscriber = {0};
static geometry_msgs__msg__Point arm_target_msg = {0};
static rcl_service_t arm_service = {0};
static rcl_publisher_t arm_ready_publisher = {0};
static std_msgs__msg__Bool arm_ready_msg = {0};
static std_srvs__srv__SetBool_Request arm_request = {0};
static std_srvs__srv__SetBool_Response arm_response = {0};

// 日志模块句柄
static rcl_publisher_t log_msg_publisher = {0};
static rcl_publisher_t log_data_publisher = {0};
std_msgs__msg__String ros_log_msg = {0};
std_msgs__msg__Float32MultiArray ros_log_data = {0};
static char ros_string_buffer[LOG_MSG_BUFFER_SIZE] = {0};
static float ros_data_buffer[LOG_DATA_COUNT + 1] = {
    0}; /* 预留第一个元素存放数据个数 */
log_msg_packet_t log_msg_packet = {0};
log_data_packet_t log_data_packet = {0};

void nav_module_init(void);
void nav_sub_callback(const void *msgin);

void grab_microros_init(void);
void grab_microros_callback(const void *request_msg, void *response_msg);
void arm_microros_init(void);
void arm_microros_callback(const void *request_msg, void *response_msg);
void arm_target_sub_callback(const void *msgin);

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
        log_message(LOG_ERROR, "Error on default allocators (line %d)\n",
                    __LINE__);
        return (int)RCL_RET_ERROR;
    }

    allocator = rcl_get_default_allocator();
    microros_rcl_mutex = xSemaphoreCreateMutex();
    if (microros_rcl_mutex == NULL) {
        log_message(LOG_ERROR, "microros_init: create mutex failed\n");
        return (int)RCL_RET_BAD_ALLOC;
    }

    // rmw_options 设置 client key，确保每次启动 client key 不同以避免与之前的实例冲突
    __attribute__((section(".noinit"))) static uint32_t boot_index;
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    ret = rcl_init_options_init(&init_options, allocator);
    if (ret != RCL_RET_OK) {
        log_message(LOG_ERROR,
                    "microros_init: rcl_init_options_init failed, ret=%d\n",
                    (int)ret);
        return (int)ret;
    }

    rmw_init_options_t *rmw_options =
        rcl_init_options_get_rmw_init_options(&init_options);
    if (rmw_options == NULL) {
        log_message(LOG_ERROR, "microros_init: get rmw init options failed\n");
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
        log_message(LOG_ERROR, "microros_init: set client key failed, ret=%d\n",
                    (int)ret);
        return (int)ret;
    }

    // init
    ret = rclc_support_init_with_options(&support, 0, NULL, &init_options,
                                         &allocator);
    if (ret != RCL_RET_OK) {
        log_message(LOG_ERROR, "microros_init: support init failed, ret=%d\n",
                    (int)ret);
        return (int)ret;
    }

    ret = rclc_node_init_default(&node, "chassis", "", &support);
    if (ret != RCL_RET_OK) {
        log_message(LOG_ERROR, "microros_init: node init failed, ret=%d\n",
                    (int)ret);
        return (int)ret;
    }

    ret = rclc_executor_init(&executor, &support.context, 12, &allocator);
    if (ret != RCL_RET_OK) {
        log_message(LOG_ERROR, "microros_init: executor init failed, ret=%d\n",
                    (int)ret);
        return (int)ret;
    }

    return (int)RCL_RET_OK;
}

/**
 * @brief 导航任务
 * 
 * @param pvParameters 
 */
void nav_task(void *pvParameters) {
    UNUSED(pvParameters);

    nav_module_init();
    vTaskDelay(1000); // 确保导航模块先于抓取模块初始化
    grab_microros_init();
    vTaskDelay(1000); // 确保抓取模块先于台阶模块初始化
    stair_microros_init();
    arm_microros_init();

    while (1) {
        if (microros_rcl_mutex != NULL &&
            xSemaphoreTake(microros_rcl_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            rclc_executor_spin_some(&executor, 5000000); /* 5ms */
            xSemaphoreGive(microros_rcl_mutex);
        }
        vTaskDelay(5);
    }
}

/**
 * @brief 初始化导航模块
 * 
 */
void nav_module_init(void) {
    rcl_ret_t ret = rclc_subscription_init_default(
        &nav_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(custom_msg, msg, SpeedHeading),
        "/nav_speed_heading_data");
    if (ret != RCL_RET_OK) {
        log_message(LOG_ERROR,
                    "nav_module_init: subscription init failed, ret=%d\n",
                    (int)ret);
        return;
    } else {
        log_message(LOG_INFO, "nav_module_init: subscription init success\n");
    }

    ret = rclc_executor_add_subscription(&executor, &nav_subscriber, &nav_pram,
                                         &nav_sub_callback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) {
        log_message(LOG_ERROR, "nav_module_init: add subscription failed");
    }
}

/**
 * @brief 导航订阅回调函数
 * @note 获取导航发布的底盘速度
 * @param msgin 
 */
void nav_sub_callback(const void *msgin) {
    nav_pram = *(const custom_msg__msg__SpeedHeading *)msgin;
}

/**
 * @brief 初始化抓取模块
 * 
 */
void grab_microros_init(void) {
    rcl_ret_t ret = rclc_service_init_default(
        &grab_service, &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(custom_msg, srv, Grab), "/grab_service");
    if (ret != RCL_RET_OK) {
        log_message(LOG_ERROR,
                    "grab_microros_init: service init failed, ret=%d\n",
                    (int)ret);
    }else{
        log_message(LOG_INFO, "grab_microros_init: service init successed");
    }
    
    ret = rclc_publisher_init_default(
        &grab_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "/grab_topic");
    if (ret != RCL_RET_OK) {
        log_message(LOG_ERROR,
                    "grab_microros_init: publisher init failed, ret=%d\n",
                    (int)ret);
    }
    
    ret = rclc_executor_add_service(&executor, &grab_service, &grab_request,
                                    &grab_response, &grab_microros_callback);
    if (ret != RCL_RET_OK) {
        log_message(LOG_ERROR, "grab_microros_init: add service failed");
    }else{
        log_message(LOG_INFO, "grab_microros_init: add service successed");
    }
}

/**
 * @brief 台阶MicroROS初始化
 */
void stair_microros_init(void) {

    rcl_ret_t ret = rclc_publisher_init_default(
        &stair_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "/stair_topic");
    if (ret != RCL_RET_OK) {
        log_message(LOG_ERROR, "stair_microros_init: publisher init failed, ret=%d\n", (int)ret);
    }
    else{
        log_message(LOG_INFO,"stair_microros_init: publisher init success");
    }
    
}

// /**
//  * @brief 夹爪动作状态发布函数
//  */
// void grab_microros_publish(void) {
//     bool success = false;

//     while (1) {
//         success = catch_head_is_target_reached();
//         if (success) {
//             grab_pub_pram.data = 1;
//             if (microros_rcl_mutex != NULL &&
//                 xSemaphoreTake(microros_rcl_mutex, pdMS_TO_TICKS(10)) ==
//                     pdTRUE) {
//                 rcl_ret_t pub_ret =
//                     rcl_publish(&grab_publisher, &grab_pub_pram, NULL);
//                 if (pub_ret != RCL_RET_OK) {
//                     log_message(LOG_ERROR,
//                                 "grab_microros_publish: publish failed\n");
//                 }
//                 xSemaphoreGive(microros_rcl_mutex);
//             }
//             grab_pub_pram.data = 0;
//             break;
//         }
//         vTaskDelay(10);
//     }
// }
/**
 * @brief 台阶动作状态发布函数
 */
void stair_microros_publish(int8_t status) {
    stair_pub_pram.data = status;
    if (microros_rcl_mutex != NULL &&
        xSemaphoreTake(microros_rcl_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        rcl_ret_t pub_ret = rcl_publish(&stair_publisher, &stair_pub_pram, NULL);
        if (pub_ret != RCL_RET_OK) {
            log_message(LOG_ERROR, "stair_microros_publish: publish failed\n");
        }
        xSemaphoreGive(microros_rcl_mutex);
    }
}

/**
 * @brief 夹爪动作状态发布函数
 */
void grab_microros_publish(int8_t status) {
    grab_pub_pram.data = status;
    if (microros_rcl_mutex != NULL &&
        xSemaphoreTake(microros_rcl_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        rcl_ret_t pub_ret = rcl_publish(&grab_publisher, &grab_pub_pram, NULL);
        if (pub_ret != RCL_RET_OK) {
            log_message(LOG_ERROR, "grab_microros_publish: publish failed\n");
        }
        xSemaphoreGive(microros_rcl_mutex);
    }
}

/**
 * @brief 抓取服务回调函数
 * 
 * @param request_msg 
 * @param response_msg 
 * @note 后续可扩展为多动作的控制指令发布，如上下台阶，机械臂控制等
 */
void grab_microros_callback(const void *request_msg, void *response_msg) {
    custom_msg__srv__Grab_Request *req_in =
        (custom_msg__srv__Grab_Request *)request_msg;
    log_message(LOG_INFO, "Received request, event = %d", req_in->event);
    if (req_in->event == 0) {
        switch (req_in->command_mode) {
            case 0:
                catch_set_state(CATCH_STATE_INIT);
                break;
            case 1:
                catch_set_state(CATCH_STATE_READY);
                break;
            case 2:
                catch_set_state(CATCH_STATE_GRAB);
                break;
            case 3:
                catch_set_state(CATCH_STATE_CHECK);
                break;
            case 4:
                catch_set_state(CATCH_STATE_ASSEMBLY);
                break;
            case 5:
                catch_set_state(CATCH_STATE_DONE);
                break;
            default:
                break;
        }

        if (catch_feedback_handle != NULL) {
            xTaskNotifyGive(catch_feedback_handle);
        }
    } else if (req_in->event == 2) {
        log_message(LOG_INFO, "Received request, command = %d", req_in->command_mode);
        switch (req_in->command_mode) {
            case 0:
                lift_set_stair_mode(1);
                break;
            case 1:
                lift_set_stair_mode(2);
                break;
            default:
                break;
        }
    }

    custom_msg__srv__Grab_Response *res_in =
        (custom_msg__srv__Grab_Response *)response_msg;
    res_in->success = true;
}

/**
 * @brief 初始化日志模块
 * 
 */
void logger_module_init(void) {
    int ret = 0;
    ret |= rclc_publisher_init_default(
        &log_msg_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/log/msg");

    ret |= rclc_publisher_init_default(
        &log_data_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/log/data");

    /* 注册日志的发送接收函数 */
    log_register_output_callback(microros_log_msg_cb, microros_log_data_cb);

    ros_log_msg.data.data = ros_string_buffer;
    ros_log_msg.data.capacity = sizeof(ros_string_buffer) / sizeof(char);
    ros_log_msg.data.size = 0;

    ros_log_data.data.data = ros_data_buffer;
    ros_log_data.data.capacity = sizeof(ros_data_buffer) / sizeof(float);
    ros_log_data.data.size = 0;

    if (ret) {
        log_message(LOG_ERROR, "logger_module_init: publisher init failed\n");
    }
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
        rcl_publish(&log_msg_publisher, &ros_log_msg, NULL);
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
            rcl_publish(&log_data_publisher, &ros_log_data, NULL);
            xSemaphoreGive(microros_rcl_mutex);
        }
    }
}

/**
 * @brief 机械臂目标坐标订阅回调函数
 */
void arm_target_sub_callback(const void *msgin) {
    const geometry_msgs__msg__Point *msg = (const geometry_msgs__msg__Point *)msgin;
    
    // 仅在 READY 态缓存有效坐标，非 READY 态直接忽略
    robot_arm_set_dynamic_catch_target((float)msg->x, (float)msg->y, (float)msg->z);
    
    log_message(LOG_INFO, "arm target received: y=%.2f z=%.2f pitch=%.2f\n",
                (float)msg->x, (float)msg->y, (float)msg->z);
}

void arm_microros_set_state(arm_status_t state) {
    arm_microros_state = state;
    if (state != ARM_STATE_READY) {
        arm_microros_clear_target();
        return;
    }

    arm_ready_msg.data = true;
    if (microros_rcl_mutex != NULL &&
        xSemaphoreTake(microros_rcl_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        rcl_ret_t ret = rcl_publish(&arm_ready_publisher, &arm_ready_msg, NULL);
        if (ret != RCL_RET_OK) {
            log_message(LOG_ERROR, "arm_microros_set_state: publish ready failed\n");
        }
        xSemaphoreGive(microros_rcl_mutex);
    }
}

void arm_microros_clear_target(void) {
    arm_target_cache_valid = false;
    arm_target_cache.x = 0.0f;
    arm_target_cache.y = 0.0f;
    arm_target_cache.z = 0.0f;
}

/**
 * @brief 初始化机械臂服务与目标订阅
 */
void arm_microros_init(void) {
    rcl_ret_t ret;

    // 初始化机械臂目标坐标订阅 
    ret = rclc_subscription_init_default(
        &arm_target_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point), "pos_sub");
    if (ret != RCL_RET_OK) {
        log_message(LOG_ERROR, "arm_microros_init: arm target sub init failed\n");
    }

    geometry_msgs__msg__Point__init(&arm_target_msg);

    ret = rclc_executor_add_subscription(&executor, &arm_target_subscriber,
                                         &arm_target_msg,
                                         &arm_target_sub_callback,
                                         ON_NEW_DATA);
    if (ret != RCL_RET_OK) {
        log_message(LOG_ERROR, "arm_microros_init: add arm target sub failed\n");
    }

    // 初始化机械臂控制服务
    ret = rclc_service_init_default(
        &arm_service, &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool), "arm_ctr_srv");
    if (ret != RCL_RET_OK) {
        log_message(LOG_ERROR, "arm_microros_init: service init failed, ret=%d\n", (int)ret);
    }

    std_srvs__srv__SetBool_Request__init(&arm_request);
    std_srvs__srv__SetBool_Response__init(&arm_response);

    ret = rclc_executor_add_service(&executor, &arm_service, &arm_request,
                                    &arm_response, &arm_microros_callback);
    if (ret != RCL_RET_OK) {
        log_message(LOG_ERROR, "arm_microros_init: add service failed");
    }
}

/**
 * @brief 机械臂服务回调函数
 */
void arm_microros_callback(const void *request_msg, void *response_msg) {
    std_srvs__srv__SetBool_Request *req_in =
        (std_srvs__srv__SetBool_Request *)request_msg;
    std_srvs__srv__SetBool_Response *res_in =
        (std_srvs__srv__SetBool_Response *)response_msg;

    res_in->success = false;

    if (req_in->data == false) {
        arm_microros_set_state(ARM_STATE_READY);
        res_in->success = true;
    } else if (arm_microros_state == ARM_STATE_READY) {
        arm_microros_set_state(ARM_STATE_CATCH);
        if (arm_target_cache_valid) {
            log_message(LOG_INFO,
                        "arm target consumed: y=%.2f z=%.2f pitch=%.2f\n",
                        (float)arm_target_cache.x,
                        (float)arm_target_cache.y,
                        (float)arm_target_cache.z);
            arm_microros_clear_target();
        } else {
            log_message(LOG_WARNING,
                        "arm service: no cached target available\n");
        }
        res_in->success = true;
    }
}