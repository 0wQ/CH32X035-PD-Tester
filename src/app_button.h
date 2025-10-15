#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 按键事件类型
 */
typedef enum {
    BUTTON_EVENT_LEFT,
    BUTTON_EVENT_RIGHT,
    BUTTON_EVENT_DOWN,
    BUTTON_EVENT_DOWN_DOUBLE_CLICK,
    BUTTON_EVENT_DOWN_LONG,
} button_event_t;

/**
 * @brief 按键事件回调函数类型
 * @param event 按键事件
 */
typedef void (*button_event_callback_t)(button_event_t event);

/**
 * @brief 初始化应用按键模块
 * @param callback 按键事件回调函数
 */
void app_button_init(button_event_callback_t callback);

/**
 * @brief 按键轮询处理函数，需要定期调用（建议5ms）
 */
void app_button_process(void);