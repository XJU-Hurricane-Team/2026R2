#ifndef __WS2812_H
#define __WS2812_H

#include <cubemx.h>

// --- 硬件参数配置 ---
#define NUM_LEDS 21        // 灯带总灯珠数量

// --- 常用颜色宏定义 (0xRRGGBB) ---
#define COLOR_OFF    0x000000 // 熄灭
#define COLOR_RED    0xFF0000 // 红色 
#define COLOR_GREEN  0x00FF00 // 绿色 
#define COLOR_BLUE   0x0000FF // 蓝色 
#define COLOR_YELLOW 0xFFFF00 // 黄色 
#define COLOR_WHITE  0xFFFFFF // 白色

void WS2812_SetColor(uint16_t led_index, uint32_t color);
void WS2812_Send(void);
void WS2812_Fill(uint32_t color);
void WS2812_Water_Step(uint32_t color);

#endif /* __WS2812_H */