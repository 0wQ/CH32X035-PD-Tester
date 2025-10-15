#include "app_control.h"
#include "utils_print.h"
#include "utils_delay.h"
#include "usbpd_sink.h"
#include "qc.h"
#include "adc.h"

static power_state_t g_power_state = {
    .power_mode = POWER_MODE_NONE,
    .pd_position = 1,
    .is_epr_ready = false,
    .qc_voltage = USB_QC_VOLTAGE_5V,
    .negotiate_voltage = 0,
    .negotiate_current = 0,
    .vbus_voltage = 0,
};

/**
 * @brief 初始化控制层
 */
void app_control_init(void) {
    adc_init();

    // 检测启动模式
    g_power_state.power_mode = POWER_MODE_NONE;

    // 初始化 PD Sink
    usbpd_sink_init();
    for (uint16_t t = 0; t < 500; t++) {
        if (usbpd_sink_get_ready()) {
            g_power_state.power_mode = POWER_MODE_PD;
            break;
        }
        delay_ms(1);
    }

    // 检测 DCP
    if (g_power_state.power_mode == POWER_MODE_NONE) {
        usb_qc_type_t qc_type = usb_qc_check();
        if (qc_type == USB_QC_TYPE_DCP) {
            g_power_state.power_mode = POWER_MODE_QC2;
        }
    }

    // 初始化 QC
    if (g_power_state.power_mode == POWER_MODE_QC2) {
        usb_qc_request(USB_QC_VOLTAGE_5V);
        delay_ms(1250);
    }

    // 初始化 PD position
    if (g_power_state.power_mode == POWER_MODE_PD) {
        g_power_state.pd_position = usbpd_sink_get_position();
    }
}

/**
 * @brief 处理左键 - 向前切换档位
 */
static void handle_left_button(void) {
    LOG("Control: handle left\n");

    if (g_power_state.power_mode == POWER_MODE_QC2) {
        // QC 模式：向前切换
        if (g_power_state.qc_voltage == USB_QC_VOLTAGE_5V) {
            g_power_state.qc_voltage = USB_QC_VOLTAGE_20V;  // 循环到最高档
        } else {
            g_power_state.qc_voltage--;
        }
    }

    if (g_power_state.power_mode == POWER_MODE_PD) {
        // PD 模式：仅更新 position，不实际切换
        const pd_available_pdos_t *pdos_ptr = usbpd_sink_get_available_pdos();

        // 找到当前 position 在数组中的索引
        int8_t current_index = -1;
        for (uint8_t i = 0; i < pdos_ptr->pdo_count; i++) {
            if (pdos_ptr->pdo[i].position == g_power_state.pd_position) {
                current_index = i;
                break;
            }
        }

        // 向前移动
        if (current_index > 0) {
            current_index--;
        } else {
            current_index = pdos_ptr->pdo_count - 1;  // 循环到最后
        }

        g_power_state.pd_position = pdos_ptr->pdo[current_index].position;
    }
}

/**
 * @brief 处理右键 - 向后切换档位
 */
static void handle_right_button(void) {
    LOG("Control: handle right\n");

    if (g_power_state.power_mode == POWER_MODE_QC2) {
        // QC 模式：向后切换
        g_power_state.qc_voltage++;
        if (g_power_state.qc_voltage >= USB_QC_VOLTAGE_MAX) {
            g_power_state.qc_voltage = USB_QC_VOLTAGE_5V;  // 循环到最低档
        }
    }

    if (g_power_state.power_mode == POWER_MODE_PD) {
        // PD 模式：仅更新 position，不实际切换
        const pd_available_pdos_t *pdos_ptr = usbpd_sink_get_available_pdos();

        // 找到当前 position 在数组中的索引
        int8_t current_index = -1;
        for (uint8_t i = 0; i < pdos_ptr->pdo_count; i++) {
            if (pdos_ptr->pdo[i].position == g_power_state.pd_position) {
                current_index = i;
                break;
            }
        }

        // 向后移动
        if (current_index < pdos_ptr->pdo_count - 1) {
            current_index++;
        } else {
            current_index = 0;  // 循环到第一个
        }

        g_power_state.pd_position = pdos_ptr->pdo[current_index].position;
    }
}

/**
 * @brief 处理下键 - 切换屏幕和打印信息
 */
static void handle_down_button(void) {
    LOG("Control: handle down\n");

    if (g_power_state.power_mode == POWER_MODE_QC2) {
        // QC 模式：请求选中的电压
        usb_qc_request(g_power_state.qc_voltage);
    }

    if (g_power_state.power_mode == POWER_MODE_PD) {
        // PD 模式：切换到选中的 position
        usbpd_sink_set_position(g_power_state.pd_position);
        // usbpd_sink_print_pdo();
    }
}

/**
 * @brief 处理下键长按 - PD Hard Reset
 */
static void handle_down_long_button(void) {
    LOG("Control: handle down long press\n");
    usbpd_sink_hard_reset();
}

/**
 * @brief 处理按键事件（外部接口）
 */
void app_control_handle_button(button_event_t event) {
    switch (event) {
        case BUTTON_EVENT_LEFT:
            handle_left_button();
            break;
        case BUTTON_EVENT_RIGHT:
            handle_right_button();
            break;
        case BUTTON_EVENT_DOWN:
            handle_down_button();
            break;
        case BUTTON_EVENT_DOWN_LONG:
            handle_down_long_button();
            break;
        default:
            break;
    }
}

/**
 * @brief 获取当前电源状态
 */
power_state_t app_control_get_power_state(void) {
    power_state_t state = {0};
    memcpy(&state, &g_power_state, sizeof(power_state_t));

    // 读取 VBUS 电压
    state.vbus_voltage = adc_get_vbus_mv();
    state.power_mode = g_power_state.power_mode;
    strcpy(state.power_mode_name, "N/A");

    // QC 模式
    if (state.power_mode == POWER_MODE_QC2) {
        strcpy(state.power_mode_name, "QC2");
        state.qc_voltage = g_power_state.qc_voltage;
        state.negotiate_voltage = QC_VOLTAGE_MAP[g_power_state.qc_voltage];
        state.negotiate_current = 0;  // QC 模式不显示电流
    }

    // PD 模式
    if (state.power_mode == POWER_MODE_PD) {
        strcpy(state.power_mode_name, "PD ");

        const pd_available_pdos_t *pdos_ptr = usbpd_sink_get_available_pdos();

        // 使用 g_power_state.pd_position（用户选择的 position）
        state.pd_position = g_power_state.pd_position;
        state.is_epr_ready = usbpd_sink_get_epr_ready();

        // 查找选中的 PDO 信息（用于预览）
        for (uint8_t i = 0; i < pdos_ptr->pdo_count; i++) {
            if (pdos_ptr->pdo[i].position == g_power_state.pd_position) {
                // state.negotiate_voltage = pdos_ptr->pdo[i].fixed.voltage;
                // state.negotiate_current = pdos_ptr->pdo[i].fixed.current;
                switch (pdos_ptr->pdo[i].pdo_type) {
                    case PDO_TYPE_FIXED_SUPPLY:
                        strcpy(state.power_mode_name, state.is_epr_ready ? "EPR" : "PD ");
                        state.negotiate_voltage = pdos_ptr->pdo[i].fixed.voltage;
                        state.negotiate_current = pdos_ptr->pdo[i].fixed.current;
                        break;
                    case PDO_TYPE_APDO:
                        if (pdos_ptr->pdo[i].apdo_subtype == APDO_TYPE_SPR_PPS) {
                            strcpy(state.power_mode_name, "PPS");
                            state.negotiate_voltage = pdos_ptr->pdo[i].pps.max_voltage;
                            state.negotiate_current = pdos_ptr->pdo[i].pps.current;
                        }
                        if (pdos_ptr->pdo[i].apdo_subtype == APDO_TYPE_EPR_AVS) {
                            strcpy(state.power_mode_name, "AVS");
                            state.negotiate_voltage = pdos_ptr->pdo[i].epr_avs.max_voltage;
                            state.negotiate_current = (uint16_t)(((uint32_t)pdos_ptr->pdo[i].epr_avs.pdp * 1000000U) / pdos_ptr->pdo[i].epr_avs.max_voltage);
                            state.negotiate_epr_avs_pdp = pdos_ptr->pdo[i].epr_avs.pdp;
                        }
                        break;
                    default:
                        state.negotiate_voltage = 0;
                        state.negotiate_current = 0;
                        break;
                }
                break;
            }
        }
    }

    return state;
}