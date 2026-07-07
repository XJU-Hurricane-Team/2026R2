/**
 * @file    iic2.h
 * @brief   软件I2C2 引脚宏定义及驱动接口
 * @note    请根据实际接线修改以下引脚宏定义
 */

#ifndef __IIC2_H
#define __IIC2_H

#include <cubemx.h>

/* ========== I2C2 引脚宏定义（请按实际接线修改） ========== */
#define IIC2_SCL_GPIO_PORT     GPIOC
#define IIC2_SCL_GPIO_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define IIC2_SCL_GPIO_PIN      GPIO_PIN_6

#define IIC2_SDA_GPIO_PORT     GPIOC
#define IIC2_SDA_GPIO_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define IIC2_SDA_GPIO_PIN      GPIO_PIN_7

/* ========== I2C2 操作宏 ========== */
#define IIC2_SCL(x)                                                            \
    x ? HAL_GPIO_WritePin(IIC2_SCL_GPIO_PORT, IIC2_SCL_GPIO_PIN, GPIO_PIN_SET) \
      : HAL_GPIO_WritePin(IIC2_SCL_GPIO_PORT, IIC2_SCL_GPIO_PIN, GPIO_PIN_RESET)

#define IIC2_SDA(x)                                                            \
    x ? HAL_GPIO_WritePin(IIC2_SDA_GPIO_PORT, IIC2_SDA_GPIO_PIN, GPIO_PIN_SET) \
      : HAL_GPIO_WritePin(IIC2_SDA_GPIO_PORT, IIC2_SDA_GPIO_PIN, GPIO_PIN_RESET)

#define IIC2_READ_SDA HAL_GPIO_ReadPin(IIC2_SDA_GPIO_PORT, IIC2_SDA_GPIO_PIN)

#define IIC2_WAIT_TIME 2 /* IO操作间隔时间, 单位为us */

/* ========== I2C2 函数声明 ========== */
void iic2_init(void);
void iic2_start(void);
void iic2_stop(void);
void iic2_ack(void);
void iic2_nack(void);
void iic2_wait_ack(void);
void iic2_send_byte(uint8_t data);
uint8_t iic2_read_byte(uint8_t ack);

#endif /* __IIC2_H */
