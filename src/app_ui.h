#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief UI 显示数据结构
 */
typedef struct {
    uint16_t vbus_voltage_mv;        // VBUS 电压 (mV)
    uint16_t negotiate_voltage_mv;   // 当前协商电压 (mV)
    uint16_t negotiate_current_ma;   // 当前协商电流 (mA)
    uint16_t negotiate_epr_avs_pdp;  // 当前协商功率 (W)
    char mode_name[4];               // 模式名称 ("QC2", "EPR", "SPR")
} ui_display_data_t;

/**
 * @brief 初始化UI模块
 */
void app_ui_init(void);

/**
 * @brief 更新UI显示
 * @param ui_display_data 显示数据
 */
void app_ui_update(const ui_display_data_t *ui_display_data);

/**
 * @brief 设置屏幕旋转
 * @param rotation 旋转状态(0=正常, 1=180度)
 */
void app_ui_set_rotation(uint8_t rotation);

/**
 * @brief 切换屏幕旋转
 */
void app_ui_toggle_rotation(void);

/**
 * @brief 清空显示
 */
void app_ui_clear(void);