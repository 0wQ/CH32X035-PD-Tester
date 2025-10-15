#include "app_ui.h"
#include <stdio.h>
#include <string.h>
#include "u8g2.h"
#include "u8g2_hal_ch32.h"

static u8g2_t u8g2;
static const u8g2_cb_t *rotation = &u8g2_cb_r0;

/**
 * @brief 格式化毫单位值（通用）
 * @param buf 输出缓冲区
 * @param buf_size 缓冲区大小
 * @param milli_val 毫单位值（mV 或 mA）
 * @param unit 单位字符（'V'或'A'）
 */
static void format_milli_value(char *buf, size_t buf_size, uint32_t milli_val, char unit) {
    // 四舍五入（+5 表示保留两位或一位小数时的精度偏移）
    uint32_t val = milli_val + 5;
    uint32_t int_part = val / 1000;
    uint32_t frac_part = (val % 1000) / 10;

    // 智能格式判断：
    // 如果整数部分 >= 100（即三位数或以上），只显示 1 位小数
    if (int_part >= 100) {
        frac_part = (val % 1000) / 100;  // 取一位小数
        snprintf(buf, buf_size, "%03u.%01u%c", (unsigned int)int_part, (unsigned int)frac_part, unit);
    } else {
        // 否则显示两位小数
        snprintf(buf, buf_size, "%02u.%02u%c", (unsigned int)int_part, (unsigned int)frac_part, unit);
    }
}

/**
 * @brief 初始化 U8G2 显示
 */
void app_ui_init(void) {
    // 初始化 U8G2
    u8g2_Setup_ssd1306_i2c_64x32_1f_f(&u8g2, rotation, u8x8_byte_hw_i2c, u8x8_gpio_and_delay);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);
    u8g2_SetContrast(&u8g2, 50);
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_6x10_mr);

    // 绘制初始静态内容
    u8g2_DrawStr(&u8g2, 0, 10, "VBUS");
    u8g2_DrawStr(&u8g2, 18, 25, "<");
    ui_display_data_t ui_data = {0};
    app_ui_update(&ui_data);
    u8g2_SendBuffer(&u8g2);
}

/**
 * @brief 设置屏幕旋转
 */
void app_ui_set_rotation(uint8_t r) {
    switch (r) {
        case 1:
            rotation = &u8g2_cb_r2;
            break;
        case 0:
        default:
            rotation = &u8g2_cb_r0;
            break;
    }

    // 重新初始化显示
    app_ui_init();
}

/**
 * @brief 切换屏幕旋转
 */
void app_ui_toggle_rotation(void) {
    static uint8_t rotation_state = 0;
    rotation_state = (rotation_state + 1) % 2;
    app_ui_set_rotation(rotation_state);
}

/**
 * @brief 更新 UI 显示
 */
void app_ui_update(const ui_display_data_t *ui_display_data) {
    if (ui_display_data == NULL) return;

    char buf[16];

    // 更新 VBUS 电压
    format_milli_value(buf, sizeof(buf), ui_display_data->vbus_voltage_mv, 'V');
    u8g2_DrawStr(&u8g2, 28, 10, buf);

    // 更新协商电压
    format_milli_value(buf, sizeof(buf), ui_display_data->negotiate_voltage_mv, 'V');
    u8g2_DrawStr(&u8g2, 28, 20, buf);

    // 更新模式标识
    u8g2_DrawStr(&u8g2, 0, 25, strlen(ui_display_data->mode_name) > 0 ? ui_display_data->mode_name : "N/A");

    // 更新协商电流/功率
    if (ui_display_data->negotiate_epr_avs_pdp > 0) {
        format_milli_value(buf, sizeof(buf), ui_display_data->negotiate_epr_avs_pdp * 1000, 'W');
        // snprintf(buf, sizeof(buf), "%05uW", ui_display_data->negotiate_epr_avs_pdp);
    } else {
        format_milli_value(buf, sizeof(buf), ui_display_data->negotiate_current_ma, 'A');
    }

    u8g2_DrawStr(&u8g2, 28, 30, buf);
    u8g2_SendBuffer(&u8g2);
}

/**
 * @brief 清空 UI 显示
 */
void app_ui_clear(void) {
    u8g2_ClearBuffer(&u8g2);
    u8g2_SendBuffer(&u8g2);
}