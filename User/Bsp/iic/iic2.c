/**
 * @file    iic2.c
 * @brief   软件I2C2 驱动代码（与 iic.c 逻辑一致，使用 IIC2_ 宏操作独立引脚）
 */

#include "./core_delay/core_delay.h"
#include "./iic/iic2.h"

void iic2_init(void) {
    GPIO_InitTypeDef gpio_initure = {0};

    IIC2_SCL_GPIO_ENABLE();
    IIC2_SDA_GPIO_ENABLE();

    gpio_initure.Pin = IIC2_SCL_GPIO_PIN;
    gpio_initure.Mode = GPIO_MODE_OUTPUT_OD;
    gpio_initure.Pull = GPIO_PULLUP;
    gpio_initure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(IIC2_SCL_GPIO_PORT, &gpio_initure);

    gpio_initure.Pin = IIC2_SDA_GPIO_PIN;
    gpio_initure.Mode = GPIO_MODE_OUTPUT_OD;
    gpio_initure.Pull = GPIO_PULLUP;
    gpio_initure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(IIC2_SDA_GPIO_PORT, &gpio_initure);

    iic2_stop();
}

void iic2_start(void) {
    IIC2_SDA(1);
    IIC2_SCL(1);
    delay_us(IIC2_WAIT_TIME);
    IIC2_SDA(0);
    IIC2_SCL(0);
    delay_us(IIC2_WAIT_TIME);
}

void iic2_stop(void) {
    IIC2_SCL(1);
    IIC2_SDA(0);
    delay_us(IIC2_WAIT_TIME);
    IIC2_SCL(1);
}

void iic2_ack(void) {
    IIC2_SDA(0);
    delay_us(IIC2_WAIT_TIME);
    IIC2_SCL(1);
    delay_us(IIC2_WAIT_TIME);
    IIC2_SCL(0);
    delay_us(IIC2_WAIT_TIME);
    IIC2_SDA(1);
    delay_us(IIC2_WAIT_TIME);
}

void iic2_nack(void) {
    IIC2_SDA(1);
    delay_us(IIC2_WAIT_TIME);
    IIC2_SCL(1);
    delay_us(IIC2_WAIT_TIME);
    IIC2_SCL(0);
    delay_us(IIC2_WAIT_TIME);
}

void iic2_wait_ack(void) {
    IIC2_SCL(1);
    delay_us(IIC2_WAIT_TIME);
    IIC2_SCL(0);
    delay_us(IIC2_WAIT_TIME);
}

void iic2_send_byte(uint8_t data) {
    uint8_t m;
    for (uint8_t i = 0; i < 8; i++) {
        m = data;
        m &= 0x80;
        if (m == 0x80) {
            IIC2_SDA(1);
        } else {
            IIC2_SDA(0);
        }
        data <<= 1;
        delay_us(IIC2_WAIT_TIME);
        IIC2_SCL(1);
        delay_us(IIC2_WAIT_TIME);
        IIC2_SCL(0);
    }
}

uint8_t iic2_read_byte(uint8_t ack) {
    uint8_t receive = 0;
    IIC2_SDA(1);
    for (uint8_t i = 0; i < 8; i++) {
        receive <<= 1;
        IIC2_SCL(1);
        delay_us(IIC2_WAIT_TIME);

        if (IIC2_READ_SDA) {
            receive++;
        }

        IIC2_SCL(0);
        delay_us(IIC2_WAIT_TIME);
    }

    if (!ack) {
        iic2_nack();
    } else {
        iic2_ack();
    }
    return receive;
}
