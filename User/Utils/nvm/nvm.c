/**
 * @file nvm.c
 * @author xinglu
 * @brief 简易 Flash 非易失存储 —— 使用片内 Flash 最后一页，
 *        以 slot 方式顺序写入，支持掉电不丢失。
 *
 *  存储格式（每 slot 8 字节）：
 *    [31:0] magic = 0x4E564D01
 *    [63:32] value (uint32_t)
 *
 *  上电时向后扫描最后一个有效 slot 读取；
 *  写入时找到首个空白 slot（全 0xFF）写入；
 *  整页写满后自动擦除从头开始。
 * @version 1.0
 * @date 2026-07-07
 */

#include "nvm/nvm.h"
#include "stm32g4xx_hal.h"

/* ========== 硬件相关宏 ========== */

/** STM32G474VE 512KB Flash: 页大小 2KB，共 256 页 */
#define NVM_FLASH_PAGE_SIZE  0x800U     /* 2 KB */
#define NVM_FLASH_START      0x08000000U
#define NVM_FLASH_SIZE       0x00080000U /* 512 KB */

/** 使用最后一页 */
#define NVM_PAGE_NUM         255U
#define NVM_BASE_ADDR        (NVM_FLASH_START + NVM_PAGE_NUM * NVM_FLASH_PAGE_SIZE)

/** slot 大小 = 8 字节（2 个 uint32_t） */
#define NVM_SLOT_SIZE        8U
#define NVM_SLOTS_PER_PAGE   (NVM_FLASH_PAGE_SIZE / NVM_SLOT_SIZE) /* 256 */

/** 有效标记 */
#define NVM_MAGIC            0x4E564D01U

/* ========== 内部变量 ========== */

static uint32_t nvm_cached_value = 0;

/* ========== 静态函数声明 ========== */

/**
 * @brief 在指定页中扫描最后一个有效 slot 的索引。
 * @return 找到返回索引 (0~255)，未找到返回 -1。
 */
static int32_t nvm_find_last_slot(void);

/**
 * @brief 在指定页中扫描第一个空白 slot 的索引。
 * @return 找到返回索引 (0~255)，已满返回 -1。
 */
static int32_t nvm_find_free_slot(void);

/**
 * @brief 擦除 NVM 所在页。
 */
static void nvm_erase_page(void);

/* ========== 公开接口 ========== */

/**
 * @brief NVM 初始化：上电时调用一次，读取最后一次存入的值。
 */
void nvm_init(void) {
    int32_t idx = nvm_find_last_slot();
    if (idx >= 0) {
        uint32_t addr = NVM_BASE_ADDR + (uint32_t)idx * NVM_SLOT_SIZE + 4U;
        nvm_cached_value = *(volatile uint32_t *)addr;
    } else {
        nvm_cached_value = 0;
    }
}

/**
 * @brief 读取当前存储值。
 */
uint32_t nvm_read(void) {
    return nvm_cached_value;
}

/**
 * @brief 写入新值（掉电不丢失）。
 * @note 耗时约 100μs 量级，请在非实时关键路径调用。
 */
void nvm_write(uint32_t value) {
    if (value == nvm_cached_value) {
        return; /* 值未变，跳过 */
    }

    int32_t idx = nvm_find_free_slot();
    if (idx < 0) {
        /* 页已满，擦除后回到第 0 个 slot */
        nvm_erase_page();
        idx = 0;
    }

    uint32_t base = NVM_BASE_ADDR + (uint32_t)idx * NVM_SLOT_SIZE;

    HAL_FLASH_Unlock();

    /* 双字编程：先 magic 后 value */
    uint64_t data = ((uint64_t)value << 32) | NVM_MAGIC;
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, base, data) != HAL_OK) {
        HAL_FLASH_Lock();
        return;
    }

    HAL_FLASH_Lock();

    nvm_cached_value = value;
}

/* ========== 内部实现 ========== */

static int32_t nvm_find_last_slot(void) {
    /* 从最后一格向前扫描 */
    for (int32_t i = (int32_t)(NVM_SLOTS_PER_PAGE - 1); i >= 0; i--) {
        uint32_t addr = NVM_BASE_ADDR + (uint32_t)i * NVM_SLOT_SIZE;
        uint32_t magic = *(volatile uint32_t *)addr;
        if (magic == NVM_MAGIC) {
            return i;
        }
        if (magic != 0xFFFFFFFFU) {
            /* 遇到非空非有效数据，停止扫描 */
            break;
        }
    }
    return -1;
}

static int32_t nvm_find_free_slot(void) {
    for (int32_t i = 0; i < (int32_t)NVM_SLOTS_PER_PAGE; i++) {
        uint32_t addr = NVM_BASE_ADDR + (uint32_t)i * NVM_SLOT_SIZE;
        if (*(volatile uint32_t *)addr == 0xFFFFFFFFU) {
            return i;
        }
    }
    return -1;
}

static void nvm_erase_page(void) {
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .Banks     = FLASH_BANK_2,
        .Page      = NVM_PAGE_NUM,
        .NbPages   = 1,
    };
    uint32_t page_error = 0;
    HAL_FLASHEx_Erase(&erase, &page_error);

    HAL_FLASH_Lock();
}
