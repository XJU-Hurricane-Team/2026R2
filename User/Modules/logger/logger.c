/**
 * @file    logger.c
 * @author  Deadline039, whyyy
 * @brief   日志记录系统
 * @version 2.0
 * @date    2026-03-27
 * @see     https://www.bilibili.com/video/av1250963900/?p=89
 */

#include "logger.h"

#if LOG_SHOW_RUNNING_TIME
#include LOG_GET_RUNNING_TIME_HEADER_FILE
#endif /* LOG_SHOW_RUNNING_TIME */

#if LOG_USE_RTOS
static SemaphoreHandle_t buf_semp;
MessageBufferHandle_t log_msg_buffer; // 实体定义
QueueHandle_t log_data_queue;         // 实体定义
#endif                                /* LOG_USE_RTOS */

#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#define MAX_MSG_PROCESS_PER_SLICE  4 /* 每次最多处理 3 条字符串日志 */
#define MAX_DATA_PROCESS_PER_SLICE 5 /* 每次最多处理 5 条数据日志 */
#define EXECUTE_EVERY_MS(ms, last_time_var, code_block)                        \
    do {                                                                       \
        TickType_t _now = xTaskGetTickCount();                                 \
        if (_now - (last_time_var) >= pdMS_TO_TICKS(ms)) {                     \
            code_block;                                                        \
            (last_time_var) += pdMS_TO_TICKS(ms);                              \
        }                                                                      \
    } while (0)

static log_level_t current_level;

static log_msg_packet_t packet = {0};
static log_data_packet_t log_data_packet = {0};

static log_msg_output_function_t log_msg_output_function = NULL;
static log_data_output_function_t log_data_output_function = NULL;

static const char log_level_str[6][11] = {
    "[DEBUG]   ", "[INFO]    ", "[WARNING] ",
    "[ERROR]   ", "[FATAL]   ", "[UNKNOW]  ",
};

static TaskHandle_t log_task_handle;

static uint32_t stat_dropped_msg_count = 0; // 记录丢掉的日志总数
static size_t stat_max_buffer_usage = 0; // 记录缓冲区的历史最高占用量（字节）

void log_task(void *pvParameters);

/**
 * @brief 注册回调函数接口
 * @param msg_output_func 文本日志输出回调
 * @param data_output_func 数据日志输出回调
 */
void log_register_output_callback(log_msg_output_function_t msg_output_func,
                                  log_data_output_function_t data_output_func) {
    log_msg_output_function = msg_output_func;
    log_data_output_function = data_output_func;
}

void log_init(log_level_t init_level) {
    if (init_level < LOG_UNKNOW) {
        current_level = init_level;
    }
#if LOG_USE_RTOS
#if LOG_USE_MUTEX
    buf_semp = xSemaphoreCreateMutex();
#else
    buf_semp = xSemaphoreCreateBinary();
#endif
    /* 初始化异步日志队列 */
    log_msg_buffer = xMessageBufferCreate(LOG_MSG_BUFFER_SIZE * 8);
    log_data_queue =
        xQueueCreate(LOG_DATA_QUEUE_LENGTH, sizeof(log_data_packet_t));

    xTaskCreate(log_task, "log_task", 512, NULL, 4, &log_task_handle);

#endif
}

void log_message(log_level_t level, char *format, ...) {
#if LOG_ENABLE
    unsigned int string_length = 0;
    if (level < current_level || log_msg_buffer == NULL || format == NULL) {
        return;
    }

#if LOG_USE_RTOS
    xSemaphoreTake(buf_semp, portMAX_DELAY);
#endif

#if LOG_SHOW_RUNNING_TIME
    unsigned int milliseconds = LOG_GET_RUNNING_TIME();
    int hours = (int)((float)milliseconds / 1000.0f / 3600.0f);
    int minute = (int)((float)milliseconds / 1000.0f / 60.0f) % 60;
    float second = (float)milliseconds / 1000.0f - (float)hours * 3600 -
                   (float)minute * 60;

    snprintf(packet.data, sizeof(packet.data), "[%d:%02d:%02.3f] ", hours,
             minute, second);
    string_length = strlen(packet.data);
#endif

    if (level > LOG_UNKNOW) {
        level = LOG_UNKNOW;
    }

    strncpy(&packet.data[string_length], log_level_str[level],
            sizeof(packet.data) - string_length);
    string_length = strlen(packet.data);

    va_list args;
    va_start(args, format);
    vsnprintf(&packet.data[string_length], sizeof(packet.data) - string_length,
              format, args);
    va_end(args);

    strncat(packet.data, LOG_OUTPUT_NEWLINE,
            sizeof(packet.data) - strlen(packet.data) - 1);
    packet.len = strlen(packet.data);

#if LOG_USE_RTOS

    // xMessageBufferSend(log_msg_buffer, packet.data, packet.len, 0);

    size_t sent_bytes =
        xMessageBufferSend(log_msg_buffer, packet.data, packet.len, 0);
    if (sent_bytes == 0) {
        // 如果返回 0，说明缓冲区满了，一点都没写进去.
        stat_dropped_msg_count++;
    }
    memset(&packet, 0, sizeof(log_msg_packet_t));

    xSemaphoreGive(buf_semp);
#else
    LOG_OUTPUT_STREAM_FUNCTION(packet.data, packet.len);
#endif

#endif /* LOG_ENABLE */
}

void _log_data(log_data_type_t data_id, uint8_t count, const float *data) {
#if LOG_ENABLE

    if (log_data_queue == NULL || count == 0) {
        return;
    }
#if LOG_USE_RTOS
    xSemaphoreTake(buf_semp, portMAX_DELAY);
#endif /* LOG_USE_RTOS */

    log_data_packet.id = (uint8_t)data_id;
    log_data_packet.count = count > LOG_DATA_COUNT ? LOG_DATA_COUNT : count;
    memcpy(log_data_packet.data, data, log_data_packet.count * sizeof(float));

#if LOG_USE_RTOS
    xQueueSendToBack(log_data_queue, &log_data_packet, 0);
    memset(&log_data_packet, 0, sizeof(log_data_packet_t));
    xSemaphoreGive(buf_semp);
#endif /* LOG_USE_RTOS */

#endif /* LOG_ENABLE */
}

#if LOG_USE_RTOS
void log_task(void *pvParameters) {

    char msg_rx_buffer[LOG_MSG_BUFFER_SIZE];
    log_data_packet_t data_rx_buffer;
    size_t total_buffer_size = LOG_MSG_BUFFER_SIZE * 8; // 与创建时的大小一致

    size_t threshold_80 = total_buffer_size * 8 / 10; // 80% 危险水位
    size_t threshold_50 = total_buffer_size * 5 / 10; // 50% 安全水位
    bool warned_80_percent = false;                   // 报警状态锁
    uint32_t last_dropped_count = 0;                  // 上次记录的丢包总数

    TickType_t t_msg = xTaskGetTickCount();
    TickType_t t_data = xTaskGetTickCount();

    while (1) {
        /* ---------------- 字符串日志处理块 ---------------- */
        EXECUTE_EVERY_MS(200, t_msg, {
            if (log_msg_buffer != NULL && log_msg_output_function != NULL) {
                size_t rx_len;
                uint8_t process_count = 0;

                while ((process_count < MAX_MSG_PROCESS_PER_SLICE) &&
                       ((rx_len = xMessageBufferReceive(
                             log_msg_buffer, msg_rx_buffer,
                             sizeof(msg_rx_buffer) - 1, 0)) > 0)) {
                    msg_rx_buffer[rx_len] = '\0';
                    log_msg_output_function(msg_rx_buffer, rx_len);
                    process_count++;
                }
            }
        });

        /* ---------------- 数据日志处理块 ---------------- */
        EXECUTE_EVERY_MS(50, t_data, {
            if (log_data_queue != NULL && log_data_output_function != NULL) {
                uint8_t process_count = 0;

                while ((process_count < MAX_DATA_PROCESS_PER_SLICE) &&
                       (xQueueReceive(log_data_queue, &data_rx_buffer, 0) ==
                        pdTRUE)) {
                    log_data_output_function(&data_rx_buffer);
                    process_count++;
                }
            }
        });

        /* ---------------- 系统状态检测处理块 ---------------- */
        size_t free_space = xMessageBufferSpaceAvailable(log_msg_buffer);
        size_t current_usage = total_buffer_size - free_space;

        /* 缓存使用超过80% */
        if (current_usage > threshold_80) {
            // 打印一次警告
            if (!warned_80_percent) {
                char warn_str[128];
                int len =
                    snprintf(warn_str, sizeof(warn_str),
                             "[LOG WARN] Buffer > 80%%! (%d/%d bytes)\r\n",
                             (int)current_usage, (int)total_buffer_size);
                if (len > 0) {
                    log_msg_output_function(warn_str, len);
                }

                warned_80_percent = true;
            }
        }
        // 降到 50% 以下，解除警报状态
        else if (current_usage < threshold_50 && warned_80_percent) {
            warned_80_percent = false;
            char warn_str[128];
            int len = snprintf(
                warn_str, sizeof(warn_str),
                "[LOG WARN] Buffer < 50%%, back to normal. (%d/%d bytes)\r\n",
                (int)current_usage, (int)total_buffer_size);
            if (len > 0) {
                log_msg_output_function(warn_str, len);
            }
        }

        if (log_msg_output_function != NULL) {
            uint16_t current_dropped = stat_dropped_msg_count;

            // 如果现在的丢包总数 > 上次的丢包总数，打印一次丢包信息
            if (current_dropped > last_dropped_count) {
                char drop_str[64];
                int len = snprintf(drop_str, sizeof(drop_str),
                                   "[LOG ERROR] Total drops: %u)\r\n",
                                   (unsigned int)current_dropped);
                if (len > 0) {
                    log_msg_output_function(drop_str, len);
                }

                // 更新记录，等待下一次增量
                last_dropped_count = current_dropped;
            }
        }

        vTaskDelay(2);
    }
}
#endif /* LOG_USE_RTOS */

/**
 * @brief 设置当前日志级别
 *
 * @param level 日志等级
 */
void log_set_level(log_level_t level) {
    current_level = level;
}

/**
 * @brief 获取当前日志级别
 *
 * @return log_level_t 获取到的日志级别
 */
log_level_t log_get_level(void) {
    return current_level;
}
