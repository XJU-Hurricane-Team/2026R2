#include "vl53l1_platform.h"
#include "./iic/iic.h"                // I2C1
#include "./iic/iic2.h"               // I2C2
#include "./iic/iic3.h"               // I2C3
#include "./core_delay/core_delay.h"  // 延时函数

/* ========== I2C 总线选择辅助宏 ========== */
/* 根据 Dev->i2c_bus_id 分发到对应 I2C 总线的函数调用 */

#define IIC_SELECT_START(bus)   do { if ((bus)==0) iic_start();  else if ((bus)==1) iic2_start();  else iic3_start();  } while(0)
#define IIC_SELECT_STOP(bus)    do { if ((bus)==0) iic_stop();   else if ((bus)==1) iic2_stop();   else iic3_stop();   } while(0)
#define IIC_SELECT_ACK(bus)     do { if ((bus)==0) iic_ack();    else if ((bus)==1) iic2_ack();    else iic3_ack();    } while(0)
#define IIC_SELECT_NACK(bus)    do { if ((bus)==0) iic_nack();   else if ((bus)==1) iic2_nack();   else iic3_nack();   } while(0)
#define IIC_SELECT_WAIT_ACK(bus) do { if ((bus)==0) iic_wait_ack(); else if ((bus)==1) iic2_wait_ack(); else iic3_wait_ack(); } while(0)
#define IIC_SELECT_SEND(bus, d)  do { if ((bus)==0) iic_send_byte(d); else if ((bus)==1) iic2_send_byte(d); else iic3_send_byte(d); } while(0)
#define IIC_SELECT_READ(bus, a)  ((bus)==0 ? iic_read_byte(a) : ((bus)==1 ? iic2_read_byte(a) : iic3_read_byte(a)))

/**
 * @brief 连续写多个字节
 */
int8_t VL53L1_WriteMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    uint8_t bus = Dev->i2c_bus_id;
    uint8_t addr = Dev->I2cDevAddr;

    IIC_SELECT_START(bus);
    IIC_SELECT_SEND(bus, addr);
    IIC_SELECT_WAIT_ACK(bus);

    IIC_SELECT_SEND(bus, (uint8_t)(index >> 8));
    IIC_SELECT_WAIT_ACK(bus);
    IIC_SELECT_SEND(bus, (uint8_t)(index & 0xFF));
    IIC_SELECT_WAIT_ACK(bus);

    for (uint32_t i = 0; i < count; i++) {
        IIC_SELECT_SEND(bus, pdata[i]);
        IIC_SELECT_WAIT_ACK(bus);
    }

    IIC_SELECT_STOP(bus);
    return 0;
}

/**
 * @brief 连续读多个字节
 */
int8_t VL53L1_ReadMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    uint8_t bus = Dev->i2c_bus_id;
    uint8_t addr = Dev->I2cDevAddr;

    IIC_SELECT_START(bus);
    IIC_SELECT_SEND(bus, addr);
    IIC_SELECT_WAIT_ACK(bus);

    IIC_SELECT_SEND(bus, (uint8_t)(index >> 8));
    IIC_SELECT_WAIT_ACK(bus);
    IIC_SELECT_SEND(bus, (uint8_t)(index & 0xFF));
    IIC_SELECT_WAIT_ACK(bus);

    IIC_SELECT_START(bus);
    IIC_SELECT_SEND(bus, addr | 0x01);
    IIC_SELECT_WAIT_ACK(bus);

    for (uint32_t i = 0; i < count; i++) {
        pdata[i] = IIC_SELECT_READ(bus, (i != count - 1));
    }

    IIC_SELECT_STOP(bus);
    return 0;
}

/**
 * @brief 写单字节
 */
int8_t VL53L1_WrByte(VL53L1_DEV Dev, uint16_t index, uint8_t data) {
    return VL53L1_WriteMulti(Dev, index, &data, 1);
}

/**
 * @brief 写字(16位)
 */
int8_t VL53L1_WrWord(VL53L1_DEV Dev, uint16_t index, uint16_t data) {
    uint8_t buf[2];
    buf[0] = data >> 8;
    buf[1] = data & 0xFF;
    return VL53L1_WriteMulti(Dev, index, buf, 2);
}

/**
 * @brief 写双字(32位)
 */
int8_t VL53L1_WrDWord(VL53L1_DEV Dev, uint16_t index, uint32_t data) {
    uint8_t buf[4];
    buf[0] = (data >> 24) & 0xFF;
    buf[1] = (data >> 16) & 0xFF;
    buf[2] = (data >> 8) & 0xFF;
    buf[3] = (data >> 0) & 0xFF;
    return VL53L1_WriteMulti(Dev, index, buf, 4);
}

/**
 * @brief 读单字节
 */
int8_t VL53L1_RdByte(VL53L1_DEV Dev, uint16_t index, uint8_t *data) {
    return VL53L1_ReadMulti(Dev, index, data, 1);
}

/**
 * @brief 读字(16位)
 */
int8_t VL53L1_RdWord(VL53L1_DEV Dev, uint16_t index, uint16_t *data) {
    int8_t status;
    uint8_t buf[2];
    status = VL53L1_ReadMulti(Dev, index, buf, 2);
    if (!status) {
        *data = ((uint16_t)buf[0] << 8) + (uint16_t)buf[1];
    }
    return status;
}

/**
 * @brief 读双字(32位)
 */
int8_t VL53L1_RdDWord(VL53L1_DEV Dev, uint16_t index, uint32_t *data) {
    int8_t status;
    uint8_t buf[4];
    status = VL53L1_ReadMulti(Dev, index, buf, 4);
    if (!status) {
        *data = ((uint32_t)buf[0] << 24) + ((uint32_t)buf[1] << 16) + ((uint32_t)buf[2] << 8) + (uint32_t)buf[3];
    }
    return status;
}

/**
 * @brief 微秒延时
 */
int8_t VL53L1_WaitUs(VL53L1_DEV Dev, int32_t wait_us) {
    delay_us(wait_us); // 调用你模板中的延时函数[cite: 1]
    return 0;
}

/**
 * @brief 毫秒延时
 */
int8_t VL53L1_WaitMs(VL53L1_DEV Dev, int32_t wait_ms) {
    delay_ms(wait_ms); // 需要确保你的延时库中有该函数
    return 0;
}

// 如果你使用的是 STM32 HAL 库，请确保包含了 HAL 库的头文件，以调用 HAL_GetTick()
// #include "main.h"  

/**
 * @brief 获取系统运行时间 (毫秒级)
 * API 使用此函数来计算超时时间。
 */
int8_t VL53L1_GetTickCount(
	uint32_t *ptime_ms)
{
    // 如果是 STM32 HAL 库，直接返回 HAL_GetTick()。
    // 如果是标准库或其他平台，替换为你工程中的全局毫秒时间戳变量（如 sys_tick）
    *ptime_ms = HAL_GetTick(); 
    return 0; 
}

/**
 * @brief 带有超时的寄存器轮询函数
 * API 在等待数据就绪或设备启动时，会不断读取指定寄存器，并校验是否符合掩码条件，直到超时。
 */
int8_t VL53L1_WaitValueMaskEx(
	VL53L1_DEV Dev,
	uint32_t timeout_ms,
	uint16_t index,
	uint8_t value,
	uint8_t mask,
	uint32_t poll_delay_ms)
{
    int8_t status = 0;
    uint32_t start_time_ms = 0;
    uint32_t current_time_ms = 0;
    uint8_t byte_value = 0;
    uint8_t found = 0;

    // 获取起始时间
    VL53L1_GetTickCount(&start_time_ms);

    while (!found) {
        // 1. 读取指定寄存器
        status = VL53L1_RdByte(Dev, index, &byte_value);
        if (status != 0) {
            return status;
        }

        // 2. 将读到的值与掩码进行与运算，看是否等于预期值
        if ((byte_value & mask) == value) {
            found = 1;
            break;
        }

        // 3. 检查是否超时
        VL53L1_GetTickCount(&current_time_ms);
        // 注意处理定时器溢出翻转的情况 (直接相减通常是安全的)
        if ((current_time_ms - start_time_ms) > timeout_ms) {
            status = -7; // 对应 VL53L1_ERROR_TIME_OUT 错误码
            break;
        }

        // 4. 等待一段时间后进行下一次轮询
        VL53L1_WaitMs(Dev, poll_delay_ms);
    }

    return status;
}