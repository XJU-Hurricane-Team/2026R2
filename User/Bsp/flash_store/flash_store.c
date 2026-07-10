/**
 * @file    flash_store.c
 * @brief   Flash 掉电存储模块 — 通用 API 实现
 *
 * 存储布局 (Flash 页 0x0807F800, 2KB):
 *   +--------+--------+---------------------------+
 *   |  magic | count  |      user_data[24]        |
 *   |  4B    |  4B    |          24B              |
 *   +--------+--------+---------------------------+
 *   总共 32B = 4 个 double-word
 *
 * 流程:
 *   init() → 读取 Flash, magic 有效则 count+1, 写回
 *   user_read()  → 直接读 RAM 缓存
 *   user_write() → 更新 RAM 中 user_data, 擦除 Flash 后写回全部
 *   save()       → 擦除 Flash 后写回全部
 *   erase()      → 仅擦除页
 */

#include "includes.h"
#include "flash_store.h"
#include "stm32g4xx_hal_flash.h"
#include "stm32g4xx_hal_flash_ex.h"

/* ======================== Flash 存储参数 ======================== */

/* STM32G474: 512KB Flash, 使用最后一页 (0x0807F800) */
#define FLASH_STORE_ADDR        (FLASH_BASE + 0x7F800U)
#define FLASH_STORE_MAGIC       0x5A5A0FFEU

/* 数据结构 32B = 4 个 double-word */
#define FLASH_STORE_DW_COUNT    4U

/* ======================== 存储数据结构 ======================== */

#pragma pack(push, 1)
typedef struct {
    uint32_t magic;                            /* 魔数校验 */
    uint32_t boot_count;                       /* 启动计数 */
    uint8_t  user_data[FLASH_STORE_USER_SIZE]; /* 用户自定义数据 (24B) */
} flash_store_data_t;
#pragma pack(pop)

/* 编译期检查结构体大小 */
_Static_assert(sizeof(flash_store_data_t) == 32, "flash_store_data_t must be 32 bytes");
_Static_assert(sizeof(flash_store_data_t) % 8 == 0, "flash_store_data_t must be double-word aligned");

/* ======================== 静态变量 (RAM 缓存) ======================== */

static flash_store_data_t g_data;    /* Flash 数据的 RAM 副本 */

/* ======================== 内部函数 ======================== */

/** @brief 清除所有 Flash 待处理错误标志 */
static void clear_flash_errors(void) {
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_SR_ERRORS);
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ECCR_ERRORS);
}

/** @brief 运行时获取 (Bank, Page) */
static void get_bank_and_page(uint32_t *bank, uint32_t *page) {
    if (READ_BIT(FLASH->OPTR, FLASH_OPTR_DBANK) != 0U) {
        /* 双 Bank: 0x0807F800 属于 Bank2 */
        *bank = FLASH_BANK_2;
        *page = (FLASH_STORE_ADDR - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
    } else {
        *bank = FLASH_BANK_1;
        *page = (FLASH_STORE_ADDR - FLASH_BASE) / FLASH_PAGE_SIZE;
    }
}

/** @brief 擦除存储页 (需已解锁) */
static bool erase_page(uint32_t bank, uint32_t page) {
    FLASH_EraseInitTypeDef cfg = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .Banks     = bank,
        .Page      = page,
        .NbPages   = 1,
    };
    uint32_t page_err = 0;
    HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&cfg, &page_err);
    if (st != HAL_OK) {
        log_message(LOG_ERROR, "Flash Store: erase err=0x%08lX", HAL_FLASH_GetError());
        clear_flash_errors();
        return false;
    }
    return true;
}

/** @brief 编程一个 double-word (需已解锁) */
static bool program_dword(uint32_t addr, uint64_t val) {
    HAL_StatusTypeDef st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, val);
    if (st != HAL_OK) {
        log_message(LOG_ERROR, "Flash Store: prog@0x%08lX err=0x%08lX", addr, HAL_FLASH_GetError());
        clear_flash_errors();
        return false;
    }
    return true;
}

/** @brief 擦除 + 写入全部数据 (一次解锁完成) */
static bool erase_and_write(const flash_store_data_t *data) {
    uint32_t bank, page;
    get_bank_and_page(&bank, &page);

    clear_flash_errors();

    if (HAL_FLASH_Unlock() != HAL_OK) { return false; }

    bool ok = true;
    if (!erase_page(bank, page)) { ok = false; }

    if (ok) {
        const uint64_t *p = (const uint64_t *)data;
        for (uint32_t i = 0; i < FLASH_STORE_DW_COUNT; i++) {
            if (!program_dword(FLASH_STORE_ADDR + i * 8U, p[i])) {
                ok = false;
                break;
            }
        }
    }

    HAL_FLASH_Lock();
    return ok;
}

/** @brief 仅擦除存储页 */
static bool erase_only(void) {
    uint32_t bank, page;
    get_bank_and_page(&bank, &page);
    clear_flash_errors();
    if (HAL_FLASH_Unlock() != HAL_OK) { return false; }
    bool ok = erase_page(bank, page);
    HAL_FLASH_Lock();
    return ok;
}

/** @brief 从 Flash 读取数据到 RAM，返回是否有效 */
static bool load_from_flash(void) {
    const flash_store_data_t *pf = (const flash_store_data_t *)FLASH_STORE_ADDR;

    if (pf->magic != FLASH_STORE_MAGIC) {
        clear_flash_errors();
        return false;
    }
    g_data = *pf;
    clear_flash_errors();
    return true;
}

/* ======================== 对外 API ======================== */

/** @brief 初始化：加载 → count+1 → 写回 */
void flash_store_init(void) {
    uint32_t bank, page;
    get_bank_and_page(&bank, &page);
    // log_message(LOG_INFO, "Flash Store: bank=%lu page=%lu addr=0x%08lX",
    //             bank, page, FLASH_STORE_ADDR);

    if (load_from_flash()) {
        g_data.boot_count++;
        // log_message(LOG_INFO, "Flash Store: prev=%lu → now=%lu",
        //             g_data.boot_count - 1, g_data.boot_count);
    } else {
        g_data.magic      = FLASH_STORE_MAGIC;
        g_data.boot_count = 1;
        memset(g_data.user_data, 0, sizeof(g_data.user_data));
        // log_message(LOG_INFO, "Flash Store: first boot, count=1");
    }

    if (erase_and_write(&g_data)) {
        log_message(LOG_INFO, "Flash Store: saved OK");
    } else {
        log_message(LOG_ERROR, "Flash Store: save FAILED!");
    }
}

/** @brief 获取启动计数 */
uint32_t flash_store_get_boot_count(void) {
    return g_data.boot_count;
}

/** @brief 读取用户数据 */
uint32_t flash_store_user_read(uint8_t *buf, uint32_t size) {
    if (size > FLASH_STORE_USER_SIZE) size = FLASH_STORE_USER_SIZE;
    memcpy(buf, g_data.user_data, size);
    return size;
}

/** @brief 写入用户数据（擦除后重写全部） */
bool flash_store_user_write(const uint8_t *buf, uint32_t size) {
    if (size > FLASH_STORE_USER_SIZE) size = FLASH_STORE_USER_SIZE;
    memset(g_data.user_data, 0, sizeof(g_data.user_data));
    memcpy(g_data.user_data, buf, size);
    return erase_and_write(&g_data);
}

/** @brief 保存当前 RAM 数据到 Flash */
bool flash_store_save(void) {
    return erase_and_write(&g_data);
}

/** @brief 擦除存储页 */
void flash_store_erase(void) {
    erase_only();
    memset(&g_data, 0, sizeof(g_data));
    log_message(LOG_INFO, "Flash Store: erased, next boot count=1");
}
