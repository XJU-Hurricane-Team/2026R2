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

nuc_pos_data_t g_nuc_pos_data;
nuc_ctrl_data_t g_nuc_ctrl_data;
nuc_rx_mode_t g_nuc_rx_mode = NUC_RX_MODE_CTRL;

/**
 * @brief 小电脑接收回调函数（运行时按 g_nuc_rx_mode 分发）
 *
 * @param msg_length 消息帧长度
 * @param msg_id_type 消息 ID 和数据类型 (高四位为 ID, 低四位为数据类型)
 * @param[in] msg_data 消息数据接收区
 */
static void nuc_msg_callback(uint32_t msg_length, uint8_t msg_id_type,
                             uint8_t *msg_data) {
    UNUSED(msg_id_type);

    switch (g_nuc_rx_mode) {
        case NUC_RX_MODE_CTRL: {
            if (msg_length != sizeof(nuc_ctrl_data_t)) {
                return;
            }
            nuc_ctrl_data_t temp_data;
            memcpy(&temp_data, msg_data, sizeof(nuc_ctrl_data_t));
            g_nuc_ctrl_data.v = 1000.0f * temp_data.v;
            g_nuc_ctrl_data.vw = temp_data.vw;
            g_nuc_ctrl_data.yaw = temp_data.yaw;
            g_nuc_ctrl_data.lift = temp_data.lift;
            break;
        }
        case NUC_RX_MODE_POSE: {
            if (msg_length != sizeof(nuc_pos_data_t)) {
                return;
            }
            nuc_pos_data_t temp_data;
            memcpy(&temp_data, msg_data, sizeof(nuc_pos_data_t));
            g_nuc_pos_data.x = 1000.0f * temp_data.x;
            g_nuc_pos_data.y = 1000.0f * temp_data.y;
            g_nuc_pos_data.yaw = temp_data.yaw;
            break;
        }
        default:
            return;
    }
}

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
