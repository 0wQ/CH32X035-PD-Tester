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
    .is_edit_mode = false,
    .edit_voltage = 0,
};

// APDO 电压记忆（使用 position 作为索引，最多支持 11 个 PDO）
static uint16_t g_apdo_voltage_memory[12] = {0};  // position 1-11，索引 0 不使用

/**
 * @brief 获取 APDO 档位的记忆电压（如果存在）
 * @param position PDO position (1-based)
 * @return 记忆的电压值 (mV)，如果没有记忆则返回 0
 */
static uint16_t get_apdo_memory_voltage(uint8_t position) {
    if (position >= 1 && position <= 11) {
        return g_apdo_voltage_memory[position];
    }
    return 0;
}

/**
 * @brief 保存 APDO 档位的编辑电压
 * @param position PDO position (1-based)
 * @param voltage 电压值 (mV)
 */
static void save_apdo_memory_voltage(uint8_t position, uint16_t voltage) {
    if (position >= 1 && position <= 11) {
        g_apdo_voltage_memory[position] = voltage;
        LOG("Saved APDO voltage memory: position=%u, voltage=%umV\n", position, voltage);
    }
}

static uint16_t get_apdo_voltage_step(USBPD_APDO_Subtype_t apdo_subtype) {
    switch (apdo_subtype) {
        case APDO_TYPE_SPR_PPS:
            return 20;  // SPR PPS: 20mV steps
        case APDO_TYPE_EPR_AVS:
            return 100;  // EPR AVS: 100mV steps
        case APDO_TYPE_SPR_AVS:
            return 100;  // SPR AVS: 100mV steps
        default:
            return 100;
    }
}

/**
 * @brief 获取 APDO 的电压范围和步进
 */
static bool get_apdo_voltage_range(uint8_t position, uint16_t *min_voltage, uint16_t *max_voltage, uint16_t *step) {
    const pd_available_pdos_t *pdos_ptr = usbpd_sink_get_available_pdos();

    for (uint8_t i = 0; i < pdos_ptr->pdo_count; i++) {
        if (pdos_ptr->pdo[i].position == position) {
            if (pdos_ptr->pdo[i].pdo_type == PDO_TYPE_APDO) {
                *step = get_apdo_voltage_step(pdos_ptr->pdo[i].apdo_subtype);

                switch (pdos_ptr->pdo[i].apdo_subtype) {
                    case APDO_TYPE_SPR_PPS:
                        *min_voltage = pdos_ptr->pdo[i].pps.min_voltage;
                        *max_voltage = pdos_ptr->pdo[i].pps.max_voltage;
                        return true;
                    case APDO_TYPE_SPR_AVS:
                        *min_voltage = pdos_ptr->pdo[i].spr_avs.min_voltage;
                        *max_voltage = pdos_ptr->pdo[i].spr_avs.max_voltage;
                        return true;
                    case APDO_TYPE_EPR_AVS:
                        *min_voltage = pdos_ptr->pdo[i].epr_avs.min_voltage;
                        *max_voltage = pdos_ptr->pdo[i].epr_avs.max_voltage;
                        return true;
                    default:
                        return false;
                }
            }
            return false;  // 不是 APDO
        }
    }
    return false;  // 未找到
}

/**
 * @brief 检查当前 PDO 是否为 APDO
 */
static bool is_apdo(uint8_t position) {
    const pd_available_pdos_t *pdos_ptr = usbpd_sink_get_available_pdos();

    for (uint8_t i = 0; i < pdos_ptr->pdo_count; i++) {
        if (pdos_ptr->pdo[i].position == position) {
            return pdos_ptr->pdo[i].pdo_type == PDO_TYPE_APDO;
        }
    }
    return false;
}

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
 * @brief 处理左键 - 向前切换档位或减小电压
 */
static void handle_left_button(void) {
    LOG("Control: handle left\n");

    // 编辑模式下：减小电压
    if (g_power_state.is_edit_mode && g_power_state.power_mode == POWER_MODE_PD) {
        uint16_t min_voltage, max_voltage, step;
        if (get_apdo_voltage_range(g_power_state.pd_position, &min_voltage, &max_voltage, &step)) {
            if (g_power_state.edit_voltage > min_voltage) {
                // 按步进减小电压
                if (g_power_state.edit_voltage >= min_voltage + step) {
                    g_power_state.edit_voltage -= step;
                } else {
                    g_power_state.edit_voltage = min_voltage;
                }
                LOG("Edit voltage decreased to: %u mV\n", g_power_state.edit_voltage);
            }
        }
        return;
    }

    // 非编辑模式：切换档位
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
 * @brief 处理右键 - 向后切换档位或增加电压
 */
static void handle_right_button(void) {
    LOG("Control: handle right\n");

    // 编辑模式下：增加电压
    if (g_power_state.is_edit_mode && g_power_state.power_mode == POWER_MODE_PD) {
        uint16_t min_voltage, max_voltage, step;
        if (get_apdo_voltage_range(g_power_state.pd_position, &min_voltage, &max_voltage, &step)) {
            if (g_power_state.edit_voltage < max_voltage) {
                // 按步进增加电压
                if (g_power_state.edit_voltage <= max_voltage - step) {
                    g_power_state.edit_voltage += step;
                } else {
                    g_power_state.edit_voltage = max_voltage;
                }
                LOG("Edit voltage increased to: %umV\n", g_power_state.edit_voltage);
            }
        }
        return;
    }

    // 非编辑模式：切换档位
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
 * @brief 处理下键 - 请求电压
 */
static void handle_down_button(void) {
    LOG("Control: handle down\n");

    if (g_power_state.power_mode == POWER_MODE_QC2) {
        // QC 模式：请求选中的电压
        usb_qc_request(g_power_state.qc_voltage);
    }

    if (g_power_state.power_mode == POWER_MODE_PD) {
        if (g_power_state.is_edit_mode) {
            // 编辑模式：请求编辑的电压
            bool success = usbpd_sink_set_apdo_position_with_voltage(g_power_state.pd_position, g_power_state.edit_voltage);
            if (success) {
                LOG("Request APDO voltage: %umV\n", g_power_state.edit_voltage);
                // 保存编辑的电压到记忆中
                save_apdo_memory_voltage(g_power_state.pd_position, g_power_state.edit_voltage);
            } else {
                LOG("Failed to request APDO voltage\n");
            }
        } else {
            // 非编辑模式：切换到选中的 position
            const pd_available_pdos_t *pdos_ptr = usbpd_sink_get_available_pdos();

            // 查找选中的 PDO
            for (uint8_t i = 0; i < pdos_ptr->pdo_count; i++) {
                if (pdos_ptr->pdo[i].position == g_power_state.pd_position) {
                    if (pdos_ptr->pdo[i].pdo_type == PDO_TYPE_FIXED_SUPPLY) {
                        usbpd_sink_set_fpdo_position(g_power_state.pd_position);
                    } else if (pdos_ptr->pdo[i].pdo_type == PDO_TYPE_APDO) {
                        // APDO：优先使用记忆电压，否则使用最小电压
                        uint16_t request_voltage = get_apdo_memory_voltage(g_power_state.pd_position);

                        if (request_voltage == 0) {
                            // 没有记忆，使用最小电压
                            switch (pdos_ptr->pdo[i].apdo_subtype) {
                                case APDO_TYPE_SPR_PPS:
                                    request_voltage = pdos_ptr->pdo[i].pps.min_voltage;
                                    break;
                                case APDO_TYPE_EPR_AVS:
                                    request_voltage = pdos_ptr->pdo[i].epr_avs.min_voltage;
                                    break;
                                case APDO_TYPE_SPR_AVS:
                                    request_voltage = pdos_ptr->pdo[i].spr_avs.min_voltage;
                                    break;
                                default:
                                    break;
                            }
                        }

                        if (request_voltage > 0) {
                            bool success = usbpd_sink_set_apdo_position_with_voltage(g_power_state.pd_position, request_voltage);
                            if (success) {
                                // 保存请求的电压到记忆中
                                save_apdo_memory_voltage(g_power_state.pd_position, request_voltage);
                            }
                        }
                    }
                    break;
                }
            }
        }
    }
}

/**
 * @brief 处理下键长按 - 调试打印
 */
static void handle_down_long_button(void) {
    LOG("Control: handle down long press\n");
    usbpd_sink_debug_available_pdos();
}

/**
 * @brief 处理下键双击 - 进入/退出编辑模式
 */
static void handle_down_double_click(void) {
    LOG("Control: handle down double click\n");

    if (g_power_state.power_mode != POWER_MODE_PD) {
        return;  // 仅 PD 模式支持编辑
    }

    // 检查是否为 APDO
    if (!is_apdo(g_power_state.pd_position)) {
        LOG("Select PDO is not APDO, cannot enter edit mode\n");
        return;
    }

    // 切换编辑模式
    g_power_state.is_edit_mode = !g_power_state.is_edit_mode;

    if (g_power_state.is_edit_mode) {
        // 进入编辑模式：优先使用记忆电压，否则使用最小值
        uint16_t min_voltage, max_voltage, step;
        if (get_apdo_voltage_range(g_power_state.pd_position, &min_voltage, &max_voltage, &step)) {
            // 尝试获取记忆的电压
            uint16_t memory_voltage = get_apdo_memory_voltage(g_power_state.pd_position);

            if (memory_voltage >= min_voltage && memory_voltage <= max_voltage) {
                // 使用记忆的电压
                g_power_state.edit_voltage = memory_voltage;
                LOG("Enter edit mode, use memory voltage: %umV\n", g_power_state.edit_voltage);
            } else {
                // 没有有效记忆，使用最小电压
                g_power_state.edit_voltage = min_voltage;
                LOG("Enter edit mode, use min voltage: %umV\n", g_power_state.edit_voltage);
            }
        } else {
            g_power_state.is_edit_mode = false;  // 获取范围失败，退出编辑模式
            LOG("Failed to get voltage range, cannot enter edit mode\n");
        }
    } else {
        // 退出编辑模式
        LOG("Exit edit mode\n");
        g_power_state.edit_voltage = 0;
    }
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
        case BUTTON_EVENT_DOWN_DOUBLE_CLICK:
            handle_down_double_click();
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
    strcpy(state.power_mode_desc, "");

    // QC 模式
    if (state.power_mode == POWER_MODE_QC2) {
        strcpy(state.power_mode_name, "QC2");
        state.qc_voltage = g_power_state.qc_voltage;
        state.negotiate_voltage = QC_VOLTAGE_MAP[g_power_state.qc_voltage];
        state.negotiate_current = 0;  // QC 模式不显示电流
    }

    // PD 模式
    if (state.power_mode == POWER_MODE_PD) {
        strcpy(state.power_mode_name, state.is_epr_ready ? "EPR" : "SPR");
        strcpy(state.power_mode_desc, "");

        const pd_available_pdos_t *pdos_ptr = usbpd_sink_get_available_pdos();

        // 使用 g_power_state.pd_position（用户选择的 position）
        state.pd_position = g_power_state.pd_position;
        state.is_epr_ready = usbpd_sink_get_epr_ready();
        state.is_edit_mode = g_power_state.is_edit_mode;
        state.edit_voltage = g_power_state.edit_voltage;

        // 查找选中的 PDO 信息（用于预览）
        for (uint8_t i = 0; i < pdos_ptr->pdo_count; i++) {
            if (pdos_ptr->pdo[i].position == g_power_state.pd_position) {
                switch (pdos_ptr->pdo[i].pdo_type) {
                    case PDO_TYPE_FIXED_SUPPLY:
                        strcpy(state.power_mode_name, pdos_ptr->pdo[i].fixed.voltage > 20000 ? "EPR" : "SPR");
                        strcpy(state.power_mode_desc, "FIX");
                        state.negotiate_voltage = pdos_ptr->pdo[i].fixed.voltage;
                        state.negotiate_current = pdos_ptr->pdo[i].fixed.current;
                        break;
                    case PDO_TYPE_APDO:
                        if (pdos_ptr->pdo[i].apdo_subtype == APDO_TYPE_SPR_PPS) {
                            strcpy(state.power_mode_name, "SPR");
                            strcpy(state.power_mode_desc, "PPS");
                            // 编辑模式显示编辑电压，否则显示记忆电压（如无记忆则显示最小电压）
                            if (state.is_edit_mode) {
                                state.negotiate_voltage = state.edit_voltage;
                            } else {
                                uint16_t memory_voltage = get_apdo_memory_voltage(g_power_state.pd_position);
                                state.negotiate_voltage = memory_voltage > 0 ? memory_voltage : pdos_ptr->pdo[i].pps.min_voltage;
                            }
                            state.negotiate_current = pdos_ptr->pdo[i].pps.current;
                        }
                        if (pdos_ptr->pdo[i].apdo_subtype == APDO_TYPE_EPR_AVS) {
                            strcpy(state.power_mode_name, "EPR");
                            strcpy(state.power_mode_desc, "AVS");
                            // 编辑模式显示编辑电压，否则显示记忆电压（如无记忆则显示最小电压）
                            if (state.is_edit_mode) {
                                state.negotiate_voltage = state.edit_voltage;
                            } else {
                                uint16_t memory_voltage = get_apdo_memory_voltage(g_power_state.pd_position);
                                state.negotiate_voltage = memory_voltage > 0 ? memory_voltage : pdos_ptr->pdo[i].epr_avs.min_voltage;
                            }
                            state.negotiate_current = (uint16_t)(((uint32_t)pdos_ptr->pdo[i].epr_avs.pdp * 1000000U) / state.negotiate_voltage);
                            state.negotiate_current = state.negotiate_current > 5000 ? 5000 : state.negotiate_current;
                            state.negotiate_epr_avs_pdp = pdos_ptr->pdo[i].epr_avs.pdp;
                        }
                        if (pdos_ptr->pdo[i].apdo_subtype == APDO_TYPE_SPR_AVS) {
                            strcpy(state.power_mode_name, "SPR");
                            strcpy(state.power_mode_desc, "AVS");
                            // 编辑模式显示编辑电压，否则显示记忆电压（如无记忆则显示最小电压）
                            if (state.is_edit_mode) {
                                state.negotiate_voltage = state.edit_voltage;
                            } else {
                                uint16_t memory_voltage = get_apdo_memory_voltage(g_power_state.pd_position);
                                state.negotiate_voltage = memory_voltage > 0 ? memory_voltage : pdos_ptr->pdo[i].spr_avs.min_voltage;
                            }
                            state.negotiate_current = state.negotiate_voltage > 15000 ? pdos_ptr->pdo[i].spr_avs.max_current_15v_20v : pdos_ptr->pdo[i].spr_avs.max_current_9v_15v;
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