#include <stdbool.h>
#include <stdint.h>

#include "ch32x035.h"
#include "debug.h"

#include "adc.h"
#include "millis.h"
#include "multi_button.h"
#include "qc.h"
#include "usbpd_sink.h"
#include "ui.h"

// QC
static bool is_using_qc_mode = false;
static usb_qc_voltage_t qc_voltage = USB_QC_VOLTAGE_5V;

// BUTTON
static Button btn_left;
static Button btn_right;
static Button btn_down;

enum Button_IDs {
    btn_left_id,
    btn_right_id,
    btn_down_id,
};

static uint8_t read_button_gpio(uint8_t button_id) {
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

static void btn_left_callback(void *btn) {
    printf("btn left\n");

    if (is_using_qc_mode) {
        if (qc_voltage == USB_QC_VOLTAGE_5V) {
            qc_voltage = USB_QC_VOLTAGE_20V;  // 循环到最高档
        } else {
            qc_voltage--;
        }
        usb_qc_request(qc_voltage);
    } else {
        Source_PDO_Storage_t caps = usbpd_sink_get_source_caps();
        uint8_t current_pos = usbpd_sink_get_pdo_position();

        // 找到当前 position 在数组中的索引
        int8_t current_index = -1;
        for (uint8_t i = 0; i < caps.pdo_count; i++) {
            if (caps.pdos[i].position == current_pos) {
                current_index = i;
                break;
            }
        }

        // 向左移动到前一个 PDO
        if (current_index > 0) {
            current_index--;
        } else {
            current_index = caps.pdo_count - 1;  // 循环到最后一个
        }

        usbpd_sink_set_pdo_position(caps.pdos[current_index].position);
    }
}

static void btn_right_callback(void *btn) {
    printf("btn right\n");

    if (is_using_qc_mode) {
        qc_voltage++;
        if (qc_voltage >= USB_QC_VOLTAGE_MAX) {
            qc_voltage = USB_QC_VOLTAGE_5V;  // 循环到最低档
        }
        usb_qc_request(qc_voltage);
    } else {
        Source_PDO_Storage_t caps = usbpd_sink_get_source_caps();
        uint8_t current_pos = usbpd_sink_get_pdo_position();

        // 找到当前 position 在数组中的索引
        int8_t current_index = -1;
        for (uint8_t i = 0; i < caps.pdo_count; i++) {
            if (caps.pdos[i].position == current_pos) {
                current_index = i;
                break;
            }
        }

        // 向右移动到下一个 PDO
        if (current_index < caps.pdo_count - 1) {
            current_index++;
        } else {
            current_index = 0;  // 循环到第一个
        }

        usbpd_sink_set_pdo_position(caps.pdos[current_index].position);
    }
}

static void btn_down_callback(void *btn) {
    printf("btn down\n");

    static uint8_t rotation = 0;
    rotation = (rotation + 1) % 2;
    ui_set_rotation(rotation);
}

static void _button_init(void) {
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
    printf("\n\nusb pd test:\n");

    // 初始化计时器
    millis_init();

    // 初始化 ADC
    adc_init();

    // 初始化 UI
    ui_init();
    ui_update_vbus(adc_get_vbus_mv());

    // 初始化 Button
    _button_init();

    // 检测按钮是否已按下，切换 qc
    if (read_button_gpio(btn_right_id) == 0) {
        is_using_qc_mode = true;
        usb_qc_request(qc_voltage);

        // 等待按钮释放和 1.25s
        uint32_t _millis = millis();
        while (read_button_gpio(btn_right_id) == 0 || millis() - _millis <= 1250) {
            ui_update_vbus(adc_get_vbus_mv());
            ui_update_qc(qc_voltage);
            Delay_Ms(50);
        }
    }

    if (!is_using_qc_mode) {
        // 初始化 PD Sink
        usbpd_sink_init();
    }

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

            if (is_using_qc_mode) {
                ui_update_qc(qc_voltage);
            } else {
                ui_update_pd_pos(usbpd_sink_get_pdo_position());
            }
        }
    }
}