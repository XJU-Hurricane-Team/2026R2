/**
 * @file    flash_store.h
 * @brief   Flash 掉电存储模块 — 通用 API
 * @note    使用 STM32G474 Flash 最后一页 (0x0807F800, 2KB)
 *          结构: Magic(4B) + BootCount(4B) + UserData(24B) = 32B
 *          每次擦除整页 → 写入 4 个 double-word
 *
 * @warning Flash 擦除需要数十毫秒，不要在中断/临界区调用。
 */

#ifndef FLASH_STORE_H
#define FLASH_STORE_H

#include <stdbool.h>
#include <stdint.h>

/* ====================== 用户数据区最大字节数 ====================== */
#define FLASH_STORE_USER_SIZE  24U

/* ====================== API ====================== */

/**
 * @brief 初始化：从 Flash 读取上次数据，boot_count+1 后写回
 * @note  必须在系统初始化完成后、FreeRTOS 调度之前调用
 *         调用后 boot_count 即本次启动的计数值
 */
void flash_store_init(void);

/**
 * @brief 获取当前启动计数
 * @return 本次上电的 boot_count 值（1, 2, 3...）
 */
uint32_t flash_store_get_boot_count(void);

/**
 * @brief 读取用户自定义数据
 * @param buf   输出缓冲区，最大 24 字节
 * @param size  要读取的字节数 (≤ 24)
 * @return 实际读取的字节数，首次上电（无有效数据）返回 0
 */
uint32_t flash_store_user_read(uint8_t *buf, uint32_t size);

/**
 * @brief 写入用户自定义数据（擦除整页后重写全部数据，包括 boot_count）
 * @param buf   输入数据
 * @param size  字节数 (≤ 24)
 * @return true 成功, false 失败
 * @note  此操作会擦除整页 (2KB)，耗时数十ms，boot_count 保持不变
 */
bool flash_store_user_write(const uint8_t *buf, uint32_t size);

/**
 * @brief 保存当前 RAM 中的全部数据到 Flash（boot_count + user_data）
 * @return true 成功, false 失败
 * @note  修改 user_data 后调用此函数持久化
 */
bool flash_store_save(void);

/**
 * @brief 擦除 Flash 存储页（清除全部数据）
 * @note  下次上电 boot_count 从 1 重新开始
 */
void flash_store_erase(void);

#endif /* FLASH_STORE_H */
