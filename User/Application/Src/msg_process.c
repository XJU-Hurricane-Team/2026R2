/**
 * @file    msg_process.c 
 * @author  Glued_Jackrainman PickingChip
 * @brief   消息处理模块，负责遥控器和小电脑消息的接收与分发
 * @version 0.1
 * @date    2026-03-29
 */

#include "includes.h"

#define REMOTE_UART_HANDLE &huart1
#define NUC_UART_HANDLE    &huart5
#define REMOTE_SEND_PERIOD 100 /* 数据上报周期，单位: ms */

static TaskHandle_t msg_polling_task_handle;

/**
 * @brief 消息轮询任务
 *
 * @param pvParameters 任务参数
 */
void msg_polling_task(void *pvParameters) {
    UNUSED(pvParameters);

    message_register_polling_uart(MSG_RC_TO_MASTER, REMOTE_UART_HANDLE, 128,
                                  128);
    message_register_recv_callback(MSG_RC_TO_MASTER, remote_receive_callback);

    while (1) {
        message_polling_data();


        vTaskDelay(5);
    }
}

/**
 * @brief 主板消息处理初始化
 * 
 */
void msg_process_init(void) {
    BaseType_t task_create_res = pdFAIL;

    /* 消息轮询任务初始化 */
    task_create_res = xTaskCreate(msg_polling_task, "msg_polling_task", 256,
                                  NULL, 4, &msg_polling_task_handle);
    configASSERT(task_create_res == pdPASS);


}
