/**
 * @file    iic4.c
 * @brief   软件I2C4 驱动代码（与 iic.c 逻辑一致，使用 IIC4_ 宏操作独立引脚）
 */

#include "./core_delay/core_delay.h"
#include "./iic/iic4.h"

void iic4_init(void) {
    GPIO_InitTypeDef gpio_initure = {0};

    IIC4_SCL_GPIO_ENABLE();
    IIC4_SDA_GPIO_ENABLE();

    gpio_initure.Pin = IIC4_SCL_GPIO_PIN;
    gpio_initure.Mode = GPIO_MODE_OUTPUT_OD;
    gpio_initure.Pull = GPIO_PULLUP;
    gpio_initure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(IIC4_SCL_GPIO_PORT, &gpio_initure);

    gpio_initure.Pin = IIC4_SDA_GPIO_PIN;
    gpio_initure.Mode = GPIO_MODE_OUTPUT_OD;
    gpio_initure.Pull = GPIO_PULLUP;
    gpio_initure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(IIC4_SDA_GPIO_PORT, &gpio_initure);

    iic4_stop();
}

void iic4_start(void) {
    IIC4_SDA(1);
    IIC4_SCL(1);
    delay_us(IIC4_WAIT_TIME);
    IIC4_SDA(0);
    IIC4_SCL(0);
    delay_us(IIC4_WAIT_TIME);
}

void iic4_stop(void) {
    IIC4_SCL(1);
    IIC4_SDA(0);
    delay_us(IIC4_WAIT_TIME);
    IIC4_SCL(1);
}

void iic4_ack(void) {
    IIC4_SDA(0);
    delay_us(IIC4_WAIT_TIME);
    IIC4_SCL(1);
    delay_us(IIC4_WAIT_TIME);
    IIC4_SCL(0);
    delay_us(IIC4_WAIT_TIME);
    IIC4_SDA(1);
    delay_us(IIC4_WAIT_TIME);
}

void iic4_nack(void) {
    IIC4_SDA(1);
    delay_us(IIC4_WAIT_TIME);
    IIC4_SCL(1);
    delay_us(IIC4_WAIT_TIME);
    IIC4_SCL(0);
    delay_us(IIC4_WAIT_TIME);
}

void iic4_wait_ack(void) {
    IIC4_SCL(1);
    delay_us(IIC4_WAIT_TIME);
    IIC4_SCL(0);
    delay_us(IIC4_WAIT_TIME);
}

void iic4_send_byte(uint8_t data) {
    uint8_t m;
    for (uint8_t i = 0; i < 8; i++) {
        m = data;
        m &= 0x80;
        if (m == 0x80) {
            IIC4_SDA(1);
        } else {
            IIC4_SDA(0);
        }
        data <<= 1;
        delay_us(IIC4_WAIT_TIME);
        IIC4_SCL(1);
        delay_us(IIC4_WAIT_TIME);
        IIC4_SCL(0);
    }
}

uint8_t iic4_read_byte(uint8_t ack) {
    uint8_t receive = 0;
    IIC4_SDA(1);
    for (uint8_t i = 0; i < 8; i++) {
        receive <<= 1;
        IIC4_SCL(1);
        delay_us(IIC4_WAIT_TIME);

        if (IIC4_READ_SDA) {
            receive++;
        }

        IIC4_SCL(0);
        delay_us(IIC4_WAIT_TIME);
    }

    if (!ack) {
        iic4_nack();
    } else {
        iic4_ack();
    }
    return receive;
}
