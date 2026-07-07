/**
 * @file    iic3.c
 * @brief   软件I2C3 驱动代码（与 iic.c 逻辑一致，使用 IIC3_ 宏操作独立引脚）
 */

#include "./core_delay/core_delay.h"
#include "./iic/iic3.h"

void iic3_init(void) {
    GPIO_InitTypeDef gpio_initure = {0};

    IIC3_SCL_GPIO_ENABLE();
    IIC3_SDA_GPIO_ENABLE();

    gpio_initure.Pin = IIC3_SCL_GPIO_PIN;
    gpio_initure.Mode = GPIO_MODE_OUTPUT_OD;
    gpio_initure.Pull = GPIO_PULLUP;
    gpio_initure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(IIC3_SCL_GPIO_PORT, &gpio_initure);

    gpio_initure.Pin = IIC3_SDA_GPIO_PIN;
    gpio_initure.Mode = GPIO_MODE_OUTPUT_OD;
    gpio_initure.Pull = GPIO_PULLUP;
    gpio_initure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(IIC3_SDA_GPIO_PORT, &gpio_initure);

    iic3_stop();
}

void iic3_start(void) {
    IIC3_SDA(1);
    IIC3_SCL(1);
    delay_us(IIC3_WAIT_TIME);
    IIC3_SDA(0);
    IIC3_SCL(0);
    delay_us(IIC3_WAIT_TIME);
}

void iic3_stop(void) {
    IIC3_SCL(1);
    IIC3_SDA(0);
    delay_us(IIC3_WAIT_TIME);
    IIC3_SCL(1);
}

void iic3_ack(void) {
    IIC3_SDA(0);
    delay_us(IIC3_WAIT_TIME);
    IIC3_SCL(1);
    delay_us(IIC3_WAIT_TIME);
    IIC3_SCL(0);
    delay_us(IIC3_WAIT_TIME);
    IIC3_SDA(1);
    delay_us(IIC3_WAIT_TIME);
}

void iic3_nack(void) {
    IIC3_SDA(1);
    delay_us(IIC3_WAIT_TIME);
    IIC3_SCL(1);
    delay_us(IIC3_WAIT_TIME);
    IIC3_SCL(0);
    delay_us(IIC3_WAIT_TIME);
}

void iic3_wait_ack(void) {
    IIC3_SCL(1);
    delay_us(IIC3_WAIT_TIME);
    IIC3_SCL(0);
    delay_us(IIC3_WAIT_TIME);
}

void iic3_send_byte(uint8_t data) {
    uint8_t m;
    for (uint8_t i = 0; i < 8; i++) {
        m = data;
        m &= 0x80;
        if (m == 0x80) {
            IIC3_SDA(1);
        } else {
            IIC3_SDA(0);
        }
        data <<= 1;
        delay_us(IIC3_WAIT_TIME);
        IIC3_SCL(1);
        delay_us(IIC3_WAIT_TIME);
        IIC3_SCL(0);
    }
}

uint8_t iic3_read_byte(uint8_t ack) {
    uint8_t receive = 0;
    IIC3_SDA(1);
    for (uint8_t i = 0; i < 8; i++) {
        receive <<= 1;
        IIC3_SCL(1);
        delay_us(IIC3_WAIT_TIME);

        if (IIC3_READ_SDA) {
            receive++;
        }

        IIC3_SCL(0);
        delay_us(IIC3_WAIT_TIME);
    }

    if (!ack) {
        iic3_nack();
    } else {
        iic3_ack();
    }
    return receive;
}
