#include "utils_delay.h"

#include "ch32x035.h"

// 初始化标志
static uint8_t initialized = 0;

// 预计算的频率分频因子
static uint32_t us_divider;
static uint32_t ms_divider;

void delay_init(void) {
    if (initialized) return;

    // 禁用 SysTick
    SysTick->CTLR = 0;

    // 清除状态和计数器
    SysTick->SR = 0;
    SysTick->CNT = 0;

    // 设置为最大计数值（向上计数模式）
    SysTick->CMP = 0xFFFFFFFFFFFFFFFFULL;

    // 预计算分频因子
    us_divider = SystemCoreClock / 1000000;  // 微秒分频
    ms_divider = SystemCoreClock / 1000;     // 毫秒分频

    // 启用 SysTick
    SysTick->CTLR = SysTick_CTLR_STE_Msk | SysTick_CTLR_STCLK_Msk;

    // 设置初始化标志
    initialized = 1;
}

void delay_us(uint32_t us) {
    if (us == 0) return;

    uint64_t target_count = SysTick->CNT + (uint64_t)us * us_divider;

    while (SysTick->CNT < target_count);
}

void delay_ms(uint32_t ms) {
    if (ms == 0) return;

    uint64_t target_count = SysTick->CNT + (uint64_t)ms * ms_divider;

    while (SysTick->CNT < target_count);
}

uint32_t micros(void) {
    return SysTick->CNT / us_divider;
}

uint32_t millis(void) {
    return SysTick->CNT / ms_divider;
}