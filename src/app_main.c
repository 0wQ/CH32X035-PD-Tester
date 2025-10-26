#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "ch32x035.h"
#include "utils_print.h"
#include "utils_delay.h"

#include "app_control.h"
#include "app_button.h"
#include "app_ui.h"

/**
 * @brief 按键事件处理器（传递按键事件到 app_control / app_ui）
 */
static void button_event_handler(button_event_t event) {
    // 特殊处理：控制 UI 旋转
    if (event == BUTTON_EVENT_DOWN_LONG) {
        app_ui_toggle_rotation();
    }
    // 将按键事件传递给 app_control 处理
    app_control_handle_button(event);
}

int main(void) {
    // 系统初始化
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    delay_init();
    print_init(921600);
    LOG("USB PD/QC Tester\n");

    // 初始化各层模块
    app_control_init();  // 此处包含 pd 初始化，需要尽量靠前，上电后需要抢在 E-Marker 前回复第一个 SOP1 VDM
    app_ui_init();
    app_button_init(button_event_handler);

    while (1) {
        // 按键处理
        static uint32_t last_btn_millis = 0;
        if (millis() - last_btn_millis >= 5) {
            last_btn_millis = millis();
            app_button_process();
        }

        // UI 更新
        static uint32_t last_ui_millis = 0;
        if (millis() - last_ui_millis >= 50) {
            last_ui_millis = millis();

            // 从 app_control 获取状态
            power_state_t state = app_control_get_power_state();

            // 转换为 UI 数据格式
            ui_display_data_t ui_data = {0};
            ui_data.vbus_voltage_mv = state.vbus_voltage;
            ui_data.negotiate_voltage_mv = state.negotiate_voltage;
            ui_data.negotiate_current_ma = state.negotiate_current;
            ui_data.negotiate_epr_avs_pdp = state.negotiate_epr_avs_pdp;
            ui_data.is_edit_mode = state.is_edit_mode;
            strncpy(ui_data.mode_name, state.power_mode_name, sizeof(ui_data.mode_name) - 1);
            ui_data.mode_name[sizeof(ui_data.mode_name) - 1] = '\0';
            strncpy(ui_data.mode_desc, state.power_mode_desc, sizeof(ui_data.mode_desc) - 1);
            ui_data.mode_desc[sizeof(ui_data.mode_desc) - 1] = '\0';

            // 更新 UI
            app_ui_update(&ui_data);
        }
    }
}