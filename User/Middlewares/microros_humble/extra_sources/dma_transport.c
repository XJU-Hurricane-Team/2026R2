#include <uxr/client/transport.h>

#include <rmw_microxrcedds_c/config.h>

#include "cubemx.h"
#include "usart_ex/usart_ex.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "dma_transport.h"

#ifdef RMW_UXRCE_TRANSPORT_CUSTOM

bool cubemx_transport_open(struct uxrCustomTransport *transport) {
    UART_HandleTypeDef *uart = (UART_HandleTypeDef *)transport->args;
    return true;
}

bool cubemx_transport_close(struct uxrCustomTransport *transport) {
    UART_HandleTypeDef *uart = (UART_HandleTypeDef *)transport->args;
    HAL_UART_DMAStop(uart);
    return true;
}

size_t cubemx_transport_write(struct uxrCustomTransport *transport,
                              const uint8_t *buf, size_t len, uint8_t *err) {
    UART_HandleTypeDef *uart = (UART_HandleTypeDef *)transport->args;

    HAL_StatusTypeDef ret;
    if (uart->gState == HAL_UART_STATE_READY) {
        ret = HAL_UART_Transmit_DMA(uart, buf, len);
        while (ret == HAL_OK && uart->gState != HAL_UART_STATE_READY) {
            vTaskDelay(1);
        }

        return (ret == HAL_OK) ? len : 0;
    } else {
        return 0;
    }
}

// size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){
//     UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;

//     int wrote = 0;

//      wrote = uart_dmarx_read(uart, buf, len);

//     return wrote;
// }

size_t cubemx_transport_read(struct uxrCustomTransport *transport, uint8_t *buf,
                             size_t len, int timeout, uint8_t *err) {
    UART_HandleTypeDef *uart = (UART_HandleTypeDef *)transport->args;
    size_t total_read = 0;

    if (err != NULL) {
        *err = 0;
    }

    if (uart == NULL || buf == NULL || len == 0U) {
        if (err != NULL) {
            *err = 1;
        }
        return 0;
    }

    if (timeout < 0) {
        timeout = 0;
    }

    if (timeout == 0) {
        return (size_t)uart_dmarx_read(uart, buf, len);
    }

    TickType_t start_tick = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS((uint32_t)timeout);

    while (total_read < len) {
        uint32_t n = uart_dmarx_read(uart, buf + total_read, len - total_read);
        if (n > 0U) {
            // for (uint32_t i = 0; i < n; i++) {
            //     uart_printf(&huart5, "%02X ", (unsigned int)buf[total_read + i]);
            // }
            // uart_printf(&huart5, "\r\n");

            total_read += (size_t)n;
            if (total_read >= len) {
                break;
            }

            taskYIELD();
            continue;
        }

        if ((xTaskGetTickCount() - start_tick) >= timeout_ticks) {
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    return total_read;
}

#endif //RMW_UXRCE_TRANSPORT_CUSTOM

