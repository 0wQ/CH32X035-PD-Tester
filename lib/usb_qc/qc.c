/*
 * Based on CH32X035xx project (https://gitee.com/honrun_he/CH32X035xx)
 * Original: Modules/USB_QC/DriverQC.c
 * Copyright (c) Honrun — MIT License
 */

#include "qc.h"

#include "ch32x035.h"
#include "utils_delay.h"

static void dm_set(usb_qc_dp_dm_voltage_t voltage) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_16;

    switch (voltage) {
        case USB_QC_DP_DM_VOLTAGE_0V0:
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
            GPIO_Init(GPIOC, &GPIO_InitStructure);
            AFIO->CTLR &= ~(AFIO_CTLR_UDM_PUE_1 | AFIO_CTLR_UDM_PUE_0);
            AFIO->CTLR &= ~AFIO_CTLR_UDM_BC_VSRC;
            break;

        case USB_QC_DP_DM_VOLTAGE_0V6:
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
            GPIO_Init(GPIOC, &GPIO_InitStructure);
            AFIO->CTLR &= ~(AFIO_CTLR_UDM_PUE_1 | AFIO_CTLR_UDM_PUE_0);
            AFIO->CTLR |= AFIO_CTLR_UDM_PUE_1;
            AFIO->CTLR |= AFIO_CTLR_UDM_BC_VSRC;
            break;

        case USB_QC_DP_DM_VOLTAGE_3V3:
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
            GPIO_Init(GPIOC, &GPIO_InitStructure);
            AFIO->CTLR |= AFIO_CTLR_UDM_PUE_1 | AFIO_CTLR_UDM_PUE_0;
            AFIO->CTLR &= ~AFIO_CTLR_UDM_BC_VSRC;
            break;
    }

    AFIO->CTLR |= AFIO_CTLR_USB_PHY_V33;
    AFIO->CTLR |= AFIO_CTLR_USB_IOEN;
}

static void dp_set(usb_qc_dp_dm_voltage_t voltage) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_17;

    switch (voltage) {
        case USB_QC_DP_DM_VOLTAGE_0V0:
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
            GPIO_Init(GPIOC, &GPIO_InitStructure);
            AFIO->CTLR &= ~(AFIO_CTLR_UDP_PUE_1 | AFIO_CTLR_UDP_PUE_0);
            AFIO->CTLR &= ~AFIO_CTLR_UDP_BC_VSRC;
            break;

        case USB_QC_DP_DM_VOLTAGE_0V6:
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
            GPIO_Init(GPIOC, &GPIO_InitStructure);
            AFIO->CTLR &= ~(AFIO_CTLR_UDP_PUE_1 | AFIO_CTLR_UDP_PUE_0);
            AFIO->CTLR |= AFIO_CTLR_UDP_PUE_1;
            AFIO->CTLR |= AFIO_CTLR_UDP_BC_VSRC;
            break;

        case USB_QC_DP_DM_VOLTAGE_3V3:
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
            GPIO_Init(GPIOC, &GPIO_InitStructure);
            AFIO->CTLR |= AFIO_CTLR_UDP_PUE_1 | AFIO_CTLR_UDP_PUE_0;
            AFIO->CTLR &= ~AFIO_CTLR_UDP_BC_VSRC;
            break;
    }

    AFIO->CTLR |= AFIO_CTLR_USB_PHY_V33;
    AFIO->CTLR |= AFIO_CTLR_USB_IOEN;
}

// Vbc_ref=0.4V  Vbc_src=0.6V
static uint8_t dm_get(void) {
    return (AFIO->CTLR & AFIO_CTLR_UDM_BC_CMPO) != 0;
}

static uint8_t dp_get(void) {
    return (AFIO->CTLR & AFIO_CTLR_UDP_BC_CMPO) != 0;
}

void usb_qc_request(usb_qc_voltage_t voltage) {
    switch (voltage) {
        case USB_QC_VOLTAGE_5V:
            dm_set(USB_QC_DP_DM_VOLTAGE_0V0);
            dp_set(USB_QC_DP_DM_VOLTAGE_0V6);
            break;
        case USB_QC_VOLTAGE_9V:
            dm_set(USB_QC_DP_DM_VOLTAGE_0V6);
            dp_set(USB_QC_DP_DM_VOLTAGE_3V3);
            break;
        case USB_QC_VOLTAGE_12V:
            dm_set(USB_QC_DP_DM_VOLTAGE_0V6);
            dp_set(USB_QC_DP_DM_VOLTAGE_0V6);
            break;
        case USB_QC_VOLTAGE_20V:
            dm_set(USB_QC_DP_DM_VOLTAGE_3V3);
            dp_set(USB_QC_DP_DM_VOLTAGE_3V3);
            break;
        default:
            break;
    }
}

usb_qc_type_t usb_qc_check(void) {
    delay_ms(350);

    /* 在 DP 上输出 0.6V 左右的电压，然后检测 DM 的电平状态 */
    dm_set(USB_QC_DP_DM_VOLTAGE_0V0);
    dp_set(USB_QC_DP_DM_VOLTAGE_0V6);
    delay_ms(15);

    /* 如果 DM 电平呈现 0 状态，则说明供电端（Source）的 DM、DP 没有短接，可以判定 Source 为 SDP，否则 Source 为 DCP 或者 CDP */
    if (dm_get() == 0) {
        return USB_QC_TYPE_SDP;
    }

    /* 在 DM 数据线上输出 0.6V 左右的电压，然后检测 DP 的电平状态 */
    dm_set(USB_QC_DP_DM_VOLTAGE_0V6);
    dp_set(USB_QC_DP_DM_VOLTAGE_0V0);
    delay_ms(15);

    /* 如果 DP 电平呈现 0 状态，则说明供电端（Source）为虚短状态，判断 Source 为 CDP，否则 Source 为 DCP */
    if (dp_get() == 0) {
        return USB_QC_TYPE_CDP;
    }

    /* 在 DP 数据线上输出 0.6V 左右的电压，然后检测 DM 的电平状态 */
    dm_set(USB_QC_DP_DM_VOLTAGE_0V0);
    dp_set(USB_QC_DP_DM_VOLTAGE_0V6);
    delay_ms(135);

    /* 在 DP 上输出 0.6V 左右的电压，等待 DMDP 的短接释放，现象为 DP 电平为 0.6V，DM 下降到 0V。此时说明 BC1.2 运行成功 */
    return (dm_get() == 0) ? USB_QC_TYPE_BC : USB_QC_TYPE_DCP;
}
