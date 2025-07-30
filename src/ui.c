#include "ui.h"

#include "usbpd_sink.h"

static u8g2_t u8g2;
static const u8g2_cb_t *rotation = &u8g2_cb_r0;  // 默认旋转为 0

void ui_init(void) {
    // 初始化 U8G2
    u8g2_Setup_ssd1306_i2c_64x32_1f_f(&u8g2, rotation, u8x8_byte_hw_i2c, u8x8_gpio_and_delay);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);
    u8g2_SetContrast(&u8g2, 50);
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_6x10_mf);

    u8g2_DrawStr(&u8g2, 0, 10, "VBUS");
    u8g2_DrawStr(&u8g2, 28, 10, "00.00V");
    u8g2_DrawStr(&u8g2, 0, 25, " N <");
    u8g2_DrawStr(&u8g2, 28, 20, "--.--V");
    u8g2_DrawStr(&u8g2, 28, 30, "--.--A");

    u8g2_SendBuffer(&u8g2);
}

void ui_update_vbus(uint16_t vbus_mv) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%05.2fV", vbus_mv / 1000.0f);
    u8g2_DrawStr(&u8g2, 28, 10, buf);
    u8g2_SendBuffer(&u8g2);
}

void ui_update_pd_pos(uint8_t pos) {
    Source_PDO_Storage_t source_caps = usbpd_sink_get_source_caps();
    if (pos == 0) {
        return;
    }

    uint8_t is_support_epr = source_caps.pdos[0].fixed.epr_capable;

    Source_PDO_t *pdo = NULL;
    for (uint8_t i = 0; i < source_caps.pdo_count; i++) {
        if (source_caps.pdos[i].position == pos) {
            pdo = &source_caps.pdos[i];
            break;
        }
    }
    if (pdo == NULL) {
        return;
    }

    char buf[16];
    u8g2_DrawStr(&u8g2, 0, 25, is_support_epr ? "EPR<" : "SPR<");
    snprintf(buf, sizeof(buf), "%05.2fV", pdo->fixed.voltage / 1000.0f);
    u8g2_DrawStr(&u8g2, 28, 20, buf);
    snprintf(buf, sizeof(buf), "%05.2fA", pdo->fixed.current / 1000.0f);
    u8g2_DrawStr(&u8g2, 28, 30, buf);
    u8g2_SendBuffer(&u8g2);
}

void ui_set_rotation(uint8_t r) {
    switch (r) {
        case 1:
            rotation = &u8g2_cb_r2;
            break;
        case 0:
        default:
            rotation = &u8g2_cb_r0;
            break;
    }
    ui_init();
}
