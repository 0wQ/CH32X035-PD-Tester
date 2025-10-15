#pragma once

#include <stdint.h>

#include "ch32x035_gpio.h"

// ADC GPIO pins
#define ADC_GPIO_PORT GPIOB
#define ADC_GPIO_CLK  RCC_APB2Periph_GPIOB
#define ADC_GPIO_PIN  GPIO_Pin_0
#define ADC_CHANNEL   ADC_Channel_8

// ADC oversampling configuration
#define ADC_OVERSAMPLE_BITS 4
#define ADC_MAX_COUNT       (1U << (ADC_OVERSAMPLE_BITS + 12))
#define ADC_SAMPLE_COUNT    (1U << (ADC_OVERSAMPLE_BITS * 2))
#define ADC_CHANNEL_COUNT   1
#define ADC_BUFFER_SIZE     (ADC_CHANNEL_COUNT * ADC_SAMPLE_COUNT)

// Calibration parameters
#define VBUS_CAL_ENABLE  1       // Enable/disable calibration
#define ADC_VREF_VOLTAGE 3300    // ADC reference voltage (mV)
#define VBUS_DIV_R1      100000  // VBUS upper voltage divider resistor (Ω)
#define VBUS_DIV_R2      6800    // VBUS lower voltage divider resistor (Ω)

// Calibration point data structure
typedef struct {
    float actual;    // Actual voltage value (mV)
    float measured;  // ADC measured value (mV)
} CalibrationPoint;

// Calibration points must be sorted by measured value in ascending order
// These values are obtained with calibration disabled
static const CalibrationPoint VBUS_CAL_POINTS[] = {
    {5147, 5330},
    {9154, 9350},
    {12139, 12310},
    {15133, 15310},
    {20111, 20360},
    {28085, 28460},
};

static const uint16_t VBUS_CAL_POINTS_COUNT = sizeof(VBUS_CAL_POINTS) / sizeof(CalibrationPoint);
static const float VBUS_CONVERT_SCALE = (float)ADC_VREF_VOLTAGE / (float)ADC_MAX_COUNT * (((float)VBUS_DIV_R1 + (float)VBUS_DIV_R2) / (float)VBUS_DIV_R2);

void adc_init();
uint32_t adc_get_raw(void);
uint16_t adc_raw_to_vbus_mv(uint32_t adc_raw);
uint16_t adc_get_vbus_mv(void);
