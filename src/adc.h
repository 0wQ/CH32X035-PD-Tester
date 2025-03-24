#pragma once

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "ch32x035_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// ADC DMA 缓冲区大小
#define ADC_BUFFER_SIZE 256

// 校准点结构体
typedef struct {
    uint16_t actual;    // 实际电压值 (mV)
    uint16_t measured;  // ADC 测量值 (mV)
} CalibrationPoint;

// 校准参数
static const uint16_t ADC_VREF_VOLTAGE = 3248;  // ADC 参考电压 (mV)
static const uint32_t VBUS_DIV_R1 = 100000;     // VBUS 上分压电阻 (Ω)
static const uint32_t VBUS_DIV_R2 = 6800;       // VBUS 下分压电阻 (Ω)ß
static const bool VBUS_CAL_ENABLE = false;      // 是否启用校准

// 校准点数据 (需按从小到大排序) 实际电压值 (mV), 测量值 (mV)
static const CalibrationPoint VBUS_CAL_POINTS[] = {
    {5000, 5001},
    {20000, 20001},
};

static const int16_t VBUS_CAL_POINTS_COUNT = sizeof(VBUS_CAL_POINTS) / sizeof(CalibrationPoint);
static const float VBUS_CONVERT_SCALE = ADC_VREF_VOLTAGE / 4095.0f * ((float)(VBUS_DIV_R1 + VBUS_DIV_R2) / (float)VBUS_DIV_R2);

void adc_init();
uint16_t get_vbus_voltage(void);

#ifdef __cplusplus
}
#endif