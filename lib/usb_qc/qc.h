#ifndef __QC_H
#define __QC_H

#include "ch32x035.h"

// QC电压等级定义
#define USB_QC_VOLTAGE_0V0      0
#define USB_QC_VOLTAGE_0V6      1
#define USB_QC_VOLTAGE_3V3      2

// QC设备类型定义
#define USB_QC_TYPE_SDP         0   // 标准下行端口(Standard Downstream Port)
#define USB_QC_TYPE_DCP         1   // 专用充电端口(Dedicated Charging Port)
#define USB_QC_TYPE_CDP         2   // 充电下行端口(Charging Downstream Port)
#define USB_QC_TYPE_BC          3   // BC1.2充电器

// 函数声明
void vQcDmSet(uint8_t ucValue);     // 设置DM电压
void vQcDpSet(uint8_t ucValue);     // 设置DP电压

// QC电压请求函数
void vQcRequest5V(void);            // 请求5V输出
void vQcRequest9V(void);            // 请求9V输出
void vQcRequest12V(void);           // 请求12V输出
void vQcRequest20V(void);           // 请求20V输出

#endif /* __QC_H */ 