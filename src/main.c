#include "adc.h"
#include "debug.h"
#include "millis.h"
#include "multi_button.h"
#include "ui.h"
#include "usbpd_sink.h"

/* BUTTON */
struct Button btn_left;
struct Button btn_right;
struct Button btn_down;

enum Button_IDs {
    btn_left_id,
    btn_right_id,
    btn_down_id,
};

uint8_t read_button_gpio(uint8_t button_id) {
    switch (button_id) {
        case btn_left_id:
            return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1);
        case btn_right_id:
            return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3);
        case btn_down_id:
            return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
        default:
            return 0;
    }
}

void btn_left_callback(void *btn) {
    Source_PDO_Storage_t caps = usbpd_sink_get_source_caps();
    uint8_t current_pos = usbpd_sink_get_pdo_position();

    // 找到当前position在数组中的索引
    int8_t current_index = -1;
    for (uint8_t i = 0; i < caps.pdo_count; i++) {
        if (caps.pdos[i].position == current_pos) {
            current_index = i;
            break;
        }
    }

    // 向左移动到前一个PDO
    if (current_index > 0) {
        current_index--;
    } else {
        current_index = caps.pdo_count - 1;  // 循环到最后一个
    }

    printf("btn left, position=%d\n", caps.pdos[current_index].position);
    usbpd_sink_set_pdo_position(caps.pdos[current_index].position);
}

void btn_right_callback(void *btn) {
    Source_PDO_Storage_t caps = usbpd_sink_get_source_caps();
    uint8_t current_pos = usbpd_sink_get_pdo_position();

    // 找到当前position在数组中的索引
    int8_t current_index = -1;
    for (uint8_t i = 0; i < caps.pdo_count; i++) {
        if (caps.pdos[i].position == current_pos) {
            current_index = i;
            break;
        }
    }

    // 向右移动到下一个PDO
    if (current_index < caps.pdo_count - 1) {
        current_index++;
    } else {
        current_index = 0;  // 循环到第一个
    }

    printf("btn right, position=%d\n", caps.pdos[current_index].position);
    usbpd_sink_set_pdo_position(caps.pdos[current_index].position);
}

void btn_down_callback(void *btn) {
    printf("btn down, position=%d\n", usbpd_sink_get_pdo_position());
    // 打印完整的 PDO 存储结构内容
    // usbpd_sink_print_source_caps();
}

void _button_init(void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(GPIOB, GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_11);

    button_init(&btn_left, read_button_gpio, 0, btn_left_id);
    button_init(&btn_right, read_button_gpio, 0, btn_right_id);
    button_init(&btn_down, read_button_gpio, 0, btn_down_id);

    button_attach(&btn_left, PRESS_DOWN, btn_left_callback);
    button_attach(&btn_right, PRESS_DOWN, btn_right_callback);
    button_attach(&btn_down, PRESS_DOWN, btn_down_callback);

    button_start(&btn_left);
    button_start(&btn_right);
    button_start(&btn_down);
}

int main(void) {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(921600);

    // 初始化计时器
    millis_init();

    printf("\n\nusb pd test:\n");

    // 初始化 Button
    _button_init();

    // 初始化 UI
    ui_init();

    // 初始化 ADC
    adc_init();

    // 初始化 PD Sink
    usbpd_sink_init();

    while (1) {
        static uint32_t last_btn_millis = 0;
        if (millis() - last_btn_millis >= 20) {
            last_btn_millis = millis();
            button_ticks();
        }

        static uint32_t last_ui_millis = 0;
        if (millis() - last_ui_millis >= 100) {
            last_ui_millis = millis();
            ui_update_vbus(adc_get_vbus_mv());
            ui_update_pd_pos(usbpd_sink_get_pdo_position());
        }
    }
}