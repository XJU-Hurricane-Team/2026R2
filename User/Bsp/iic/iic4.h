/**
 * @file    iic4.h
 * @brief   软件I2C4 引脚宏定义及驱动接口
 * @note    请根据实际接线修改以下引脚宏定义
 */

#ifndef __IIC4_H
#define __IIC4_H

#include <cubemx.h>

/* ========== I2C4 引脚宏定义（请按实际接线修改） ========== */
#define IIC4_SCL_GPIO_PORT     GPIOC
#define IIC4_SCL_GPIO_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()
#define IIC4_SCL_GPIO_PIN      GPIO_PIN_10

#define IIC4_SDA_GPIO_PORT     GPIOC
#define IIC4_SDA_GPIO_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()
#define IIC4_SDA_GPIO_PIN      GPIO_PIN_11

/* ========== I2C4 操作宏 ========== */
#define IIC4_SCL(x)                                                            \
    x ? HAL_GPIO_WritePin(IIC4_SCL_GPIO_PORT, IIC4_SCL_GPIO_PIN, GPIO_PIN_SET) \
      : HAL_GPIO_WritePin(IIC4_SCL_GPIO_PORT, IIC4_SCL_GPIO_PIN, GPIO_PIN_RESET)

#define IIC4_SDA(x)                                                            \
    x ? HAL_GPIO_WritePin(IIC4_SDA_GPIO_PORT, IIC4_SDA_GPIO_PIN, GPIO_PIN_SET) \
      : HAL_GPIO_WritePin(IIC4_SDA_GPIO_PORT, IIC4_SDA_GPIO_PIN, GPIO_PIN_RESET)

#define IIC4_READ_SDA HAL_GPIO_ReadPin(IIC4_SDA_GPIO_PORT, IIC4_SDA_GPIO_PIN)

#define IIC4_WAIT_TIME 2 /* IO操作间隔时间, 单位为us */

/* ========== I2C4 函数声明 ========== */
void iic4_init(void);
void iic4_start(void);
void iic4_stop(void);
void iic4_ack(void);
void iic4_nack(void);
void iic4_wait_ack(void);
void iic4_send_byte(uint8_t data);
uint8_t iic4_read_byte(uint8_t ack);

#endif /* __IIC4_H */
