#include "app_button.h"
#include "ch32x035.h"
#include "bits_button.h"
#include "utils_print.h"

// 按键 ID 定义
typedef enum {
    BTN_LEFT = 0,
    BTN_RIGHT,
    BTN_DOWN,
    BTN_INVALID,
    BTN_MAX,
} user_button_t;

// 按键事件回调函数指针
static button_event_callback_t g_event_callback = NULL;

// 按键参数和实例
static const bits_btn_obj_param_t default_param = {
    .long_press_period_triger_ms = 30,
    .long_press_start_time_ms = 800,
    .short_press_time_ms = 250,
    .time_window_time_ms = 100,
};

static button_obj_t btns[] = {
    BITS_BUTTON_INIT(BTN_LEFT, 0, &default_param),
    BITS_BUTTON_INIT(BTN_RIGHT, 0, &default_param),
    BITS_BUTTON_INIT(BTN_DOWN, 0, &default_param),
};

/**
 * @brief 读取按键 GPIO 状态
 */
static uint8_t read_key_state(struct button_obj_t *btn) {
    switch (btn->key_id) {
        case BTN_LEFT:
            return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1);
        case BTN_RIGHT:
            return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3);
        case BTN_DOWN:
            return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
        default:
            return 0;
    }
}

/**
 * @brief 按键事件处理回调（来自 bits_button 库）
 */
static void bits_btn_result_cb(struct button_obj_t *btn, struct bits_btn_result result) {
    LOG("Button: id=%d, event=%d, value=%d, cnt=%d\n", result.key_id, result.event, result.key_value, result.long_press_period_trigger_cnt);

    if (g_event_callback == NULL) return;

    // 单击事件
    if (result.event == BTN_STATE_FINISH) {
        switch (result.key_id) {
            case BTN_DOWN:
                g_event_callback(BUTTON_EVENT_DOWN);
                break;
            default:
                break;
        }
    }

    // 按下事件
    if (result.event == BTN_STATE_PRESSED) {
        switch (result.key_id) {
            case BTN_LEFT:
                g_event_callback(BUTTON_EVENT_LEFT);
                break;
            case BTN_RIGHT:
                g_event_callback(BUTTON_EVENT_RIGHT);
                break;
            default:
                break;
        }
    }

    // 长按事件
    if (result.event == BTN_STATE_LONG_PRESS) {
        if (result.key_value == 0b11 && result.key_id == BTN_DOWN) {  // long press start
            g_event_callback(BUTTON_EVENT_DOWN_LONG);
        }

        if (result.long_press_period_trigger_cnt > 0) {  // long press hold
            // LOG("[%d] 长按保持 周期:%d\r\n", result.key_id, result.long_press_period_trigger_cnt);

            switch (result.key_id) {
                case BTN_LEFT:
                    g_event_callback(BUTTON_EVENT_LEFT);
                    break;
                case BTN_RIGHT:
                    g_event_callback(BUTTON_EVENT_RIGHT);
                    break;
            }
        }
    }

    // BTN_DOWN 双击事件
    if (result.event == BTN_STATE_FINISH && result.key_value == BITS_BTN_DOUBLE_CLICK_KV && result.key_id == BTN_DOWN) {
        g_event_callback(BUTTON_EVENT_DOWN_DOUBLE_CLICK);
    }
}

/**
 * @brief 初始化按键 GPIO
 */
static void button_gpio_init(void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/**
 * @brief 初始化按键模块
 * @param callback 按键事件回调函数
 */
void app_button_init(button_event_callback_t callback) {
    // 保存回调函数
    g_event_callback = callback;

    // 初始化 GPIO
    button_gpio_init();

    // 初始化按键库
    int32_t ret = bits_button_init(btns, ARRAY_SIZE(btns), NULL, 0, read_key_state, bits_btn_result_cb, NULL);
    if (ret) {
        LOG("Button init failed: %d\n", ret);
    }
}

/**
 * @brief 按键轮询处理（需要 5ms 定期调用）
 */
void app_button_process(void) {
    bits_button_ticks();
}