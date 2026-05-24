#include "ws2812.h"

// 占空比定义 (频率 800kHz)
#define CODE_0 68
#define CODE_1 136
#define RESET_PULSES 50 // 复位周期 

// 核心大数组：21个灯 * 24位 + 50个复位周期 = 554
uint16_t pwm_data[NUM_LEDS * 24 + RESET_PULSES] = {0};


// ==========================================
// 底层驱动函数
// ==========================================

/**
 * @brief  将单颗灯珠的 RGB 颜色“装填”到 DMA 数组中
 * @param  led_index 灯珠序号 (0 开始)
 * @param  color     24位颜色值 (格式: 0xRRGGBB)
 */
void WS2812_SetColor(uint16_t led_index, uint32_t color) {
    if (led_index >= NUM_LEDS) {
        return; // 防越界保护
    }

    // 提取 RGB 通道
    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8)  & 0xFF;
    uint8_t b = color & 0xFF;
    
    // WS2812 发送顺序是 GRB
    uint32_t grb = (g << 16) | (r << 8) | b;
    
    // 定位到该灯珠在数组里的起始位置
    uint16_t start_index = led_index * 24;
    
    // 转化为 PWM 占空比数值
    for (int i = 23; i >= 0; i--) {
        if (grb & (1 << i)) {
            pwm_data[start_index + (23 - i)] = CODE_1; 
        } else {
            pwm_data[start_index + (23 - i)] = CODE_0; 
        }
    }
}

/**
 * @brief  启动硬件 DMA 发送 
 */
void WS2812_Send(void) {
  
    // HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)pwm_data, (NUM_LEDS * 24 + RESET_PULSES));
}


// ==========================================
// 应用层功能函数 (业务逻辑直接调用)
// ==========================================

/**
 * @brief  全局颜色填充 (用于状态常亮或全灭)
 * @param  color 24位 RGB 颜色值
 */
void WS2812_Fill(uint32_t color) {
    for (int i = 0; i < NUM_LEDS; i++) {
        WS2812_SetColor(i, color);
    }
    WS2812_Send(); 
}

/**
 * @brief  单步流水灯 (非阻塞式，每次调用往前跑一颗灯)
 * @param  color 流水灯的颜色
 */
void WS2812_Water_Step(uint32_t color) {
    static uint16_t water_index = 0; 

    // 1. 全局熄灭
    for (int i = 0; i < NUM_LEDS; i++) {
        WS2812_SetColor(i, COLOR_OFF);
    }
    
    // 2. 点亮当前进度
    WS2812_SetColor(water_index, color);
    WS2812_Send();

    // 3. 步进
    water_index++;
    if (water_index >= NUM_LEDS) {
        water_index = 0;
    }
}