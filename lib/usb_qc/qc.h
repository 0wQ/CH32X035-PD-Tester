#pragma once

#include <stdint.h>

// QC dp dm 电压定义
typedef enum {
    USB_QC_DP_DM_VOLTAGE_0V0,
    USB_QC_DP_DM_VOLTAGE_0V6,
    USB_QC_DP_DM_VOLTAGE_3V3,
} usb_qc_dp_dm_voltage_t;

// QC 电压档位
typedef enum {
    USB_QC_VOLTAGE_5V,
    USB_QC_VOLTAGE_9V,
    USB_QC_VOLTAGE_12V,
    USB_QC_VOLTAGE_20V,
    USB_QC_VOLTAGE_MAX,
} usb_qc_voltage_t;

// QC 电压映射表
static const uint16_t QC_VOLTAGE_MAP[USB_QC_VOLTAGE_MAX] = {
    5000,
    9000,
    12000,
    20000,
};

// QC 设备类型定义
typedef enum {
    USB_QC_TYPE_SDP,  // 标准下行端口 (Standard Downstream Port)
    USB_QC_TYPE_DCP,  // 专用充电端口 (Dedicated Charging Port)
    USB_QC_TYPE_CDP,  // 充电下行端口 (Charging Downstream Port)
    USB_QC_TYPE_BC,   // BC1.2 充电器
} usb_qc_type_t;

void usb_qc_request(usb_qc_voltage_t voltage);
usb_qc_type_t usb_qc_check(void);
