#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "qc.h"
#include "app_button.h"

typedef enum {
    POWER_MODE_NONE = 0,
    POWER_MODE_PD,
    POWER_MODE_QC2,
} power_mode_t;

/**
 * @brief 电源状态信息
 */
typedef struct {
    power_mode_t power_mode;  // 当前电源模式

    char power_mode_name[4];  // 电源模式名称

    uint8_t pd_position;          // PD PDO 位置 (1-based)
    bool is_epr_ready;            // PD 是否已进入 EPR 模式
    usb_qc_voltage_t qc_voltage;  // QC 电压档位

    uint16_t negotiate_voltage;      // 当前协商电压
    uint16_t negotiate_current;      // 当前协商电流
    uint16_t negotiate_epr_avs_pdp;  // epr avs pdp（w）

    uint16_t vbus_voltage;  // VBUS 电压
} power_state_t;

/**
 * @brief 初始化控制层
 */
void app_control_init(void);

/**
 * @brief 处理按键事件
 * @param event 按键事件类型
 */
void app_control_handle_button(button_event_t event);

/**
 * @brief 获取当前电源状态
 * @return 电源状态结构体
 */
power_state_t app_control_get_power_state(void);
