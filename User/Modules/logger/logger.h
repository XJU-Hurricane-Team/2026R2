/**
 * @file    logger.h
 * @author  Deadline039,whyyy
 * @brief   日志记录系统
 * @version 2.0
 * @date    2026-03-27
 * @see     https://www.bilibili.com/video/av1250963900/?p=89
 *
 ******************************************************************************
 *    Date    | Version |   Author    | Version Info
 * -----------+---------+-------------+----------------------------------------
 * 2024-04-01 |   1.0   | Deadline039 | 初版
 * 2024-04-04 |   1.1   | Deadline039 | 更改 LOG_ENABLE 宏, 如果关闭此宏开关,
 *            |         |             | 日志函数会变为空函数
 * 2525-03-14 |   1.2   | Deadline039 | 修改输出接口和获取时间接口, 缓冲区加信号量保护,
 *            |         |             | 修改部分宏定义, 移除 RTC 与 FATFS.
 * 2026-04-01 |   2.0   | whyyy       | 
 *           |         |             | 
*
 */

#ifndef __LOGGER_H
#define __LOGGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* ============================= Config ============================= */

#define LOG_ENABLE           1
#define LOG_USE_RTOS         1
#define LOG_USE_MUTEX        1
#define LOG_SHOW_RUNNING_TIME 0

/* 引入 FreeRTOS 队列相关的头文件 */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "message_buffer.h"

#define LOG_MSG_BUFFER_SIZE      128
#define LOG_OUTPUT_NEWLINE   "\n"

/* 数据日志队列深度 */
#define LOG_DATA_QUEUE_LENGTH 5


#define log_data(id, ...)                                                      \
    _log_data(id, sizeof((float[]){__VA_ARGS__}) / sizeof(float),              \
              (float[]){__VA_ARGS__})

/* 输出函数 */
#define LOG_OUTPUT_STREAM_FUNCTION(str, len)                                   \
    do {                                                                       \
        while (len--) {                                                        \
            ITM_SendChar(*str++);                                              \
        }                                                                      \
    } while (0)


 
    
#define EXECUTE_EVERY_MS(ms, last_time_var, code_block)                        \
    do {                                                                       \
        TickType_t _now = xTaskGetTickCount();                                 \
        if (_now - (last_time_var) >= pdMS_TO_TICKS(ms)) {                     \
            code_block;                                                        \
            (last_time_var) += pdMS_TO_TICKS(ms);                              \
        }                                                                      \
    } while (0)
    
/* ============================= Types ============================== */

typedef struct {
    char data[LOG_MSG_BUFFER_SIZE];
    uint16_t len; /* 有效数据长度 */
} log_msg_packet_t;

#if LOG_SHOW_RUNNING_TIME
#define LOG_GET_RUNNING_TIME_HEADER_FILE <bsp.h>
#define LOG_GET_RUNNING_TIME()           HAL_GetTick()
#endif /* LOG_SHOW_RUNNING_TIME */

/**
 * @brief 日志级别
 */
typedef enum {
    LOG_DEBUG,
    LOG_INFO,
    LOG_WARNING,
    LOG_ERROR,
    LOG_FATAL,
    LOG_UNKNOW
} log_level_t;

#define LOG_DATA_COUNT 8 /* 日志数据最大个数 */
/**
 * @brief 日志数据类型
 * @note 可自行添加修改所需类型数据
 */
typedef enum {
    LOG_CHASSIS = 0,
    LOG_REMOTE,
    LOG_NAV,
} log_data_type_t;

typedef struct {
    log_data_type_t id;                 /* 数据类型ID */
    uint8_t count;              /* 有效数据个数 */
    float data[LOG_DATA_COUNT]; /* 数据内容 */
} log_data_packet_t;



/* ======================== External Handles ======================== */

extern MessageBufferHandle_t log_msg_buffer;
extern QueueHandle_t log_data_queue;
/* ============================== APIs ============================== */

void log_init(log_level_t init_level);

void log_message(log_level_t level, char *format, ...);
void _log_data(log_data_type_t id, uint8_t count, const float *data); // 不要调用该函数, 直接使用 log_data 宏

void log_set_level(log_level_t level);
log_level_t log_get_level(void);

/**
 * @brief 定义日志字符串和数据底层发送函数
 * 
 */
typedef void (*log_msg_output_function_t)(const char *str, uint16_t len);
typedef void (*log_data_output_function_t)(const log_data_packet_t *data_packet);

/**
 * @brief 注册日志输出回调函数
 * 
 * @param msg_output_func  日志字符串输出函数
 * @param data_output_func 日志数据输出函数
 */
void log_register_output_callback(log_msg_output_function_t msg_output_func,
                                  log_data_output_function_t data_output_func);

void log_task(void *pvParameters);
#ifdef __cplusplus
}
#endif

#endif /* __LOGGER_H */
