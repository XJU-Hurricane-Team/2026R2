/**
 * @file    iic3.h
 * @brief   软件I2C3 引脚宏定义及驱动接口
 * @note    请根据实际接线修改以下引脚宏定义
 */

#ifndef __IIC3_H
#define __IIC3_H

#include <cubemx.h>

/* ========== I2C3 引脚宏定义（请按实际接线修改） ========== */
#define IIC3_SCL_GPIO_PORT     GPIOC
#define IIC3_SCL_GPIO_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define IIC3_SCL_GPIO_PIN      GPIO_PIN_10

#define IIC3_SDA_GPIO_PORT     GPIOC
#define IIC3_SDA_GPIO_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define IIC3_SDA_GPIO_PIN      GPIO_PIN_11

/* ========== I2C3 操作宏 ========== */
#define IIC3_SCL(x)                                                            \
    x ? HAL_GPIO_WritePin(IIC3_SCL_GPIO_PORT, IIC3_SCL_GPIO_PIN, GPIO_PIN_SET) \
      : HAL_GPIO_WritePin(IIC3_SCL_GPIO_PORT, IIC3_SCL_GPIO_PIN, GPIO_PIN_RESET)

#define IIC3_SDA(x)                                                            \
    x ? HAL_GPIO_WritePin(IIC3_SDA_GPIO_PORT, IIC3_SDA_GPIO_PIN, GPIO_PIN_SET) \
      : HAL_GPIO_WritePin(IIC3_SDA_GPIO_PORT, IIC3_SDA_GPIO_PIN, GPIO_PIN_RESET)

#define IIC3_READ_SDA HAL_GPIO_ReadPin(IIC3_SDA_GPIO_PORT, IIC3_SDA_GPIO_PIN)

#define IIC3_WAIT_TIME 2 /* IO操作间隔时间, 单位为us */

/* ========== I2C3 函数声明 ========== */
void iic3_init(void);
void iic3_start(void);
void iic3_stop(void);
void iic3_ack(void);
void iic3_nack(void);
void iic3_wait_ack(void);
void iic3_send_byte(uint8_t data);
uint8_t iic3_read_byte(uint8_t ack);

#endif /* __IIC3_H */
