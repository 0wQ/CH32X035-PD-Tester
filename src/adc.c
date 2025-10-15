#include "adc.h"
#include <stdbool.h>
#include <stdint.h>
#include "ch32x035.h"

static uint16_t adc_buffer[ADC_BUFFER_SIZE] __attribute__((aligned(4)));

void adc_init(void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    ADC_InitTypeDef ADC_InitStructure = {0};
    DMA_InitTypeDef DMA_InitStructure = {0};

    // Enable clocks
    RCC_APB2PeriphClockCmd(ADC_GPIO_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    // Configure ADC GPIO pin
    GPIO_InitStructure.GPIO_Pin = ADC_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(ADC_GPIO_PORT, &GPIO_InitStructure);

    // Configure ADC clock
    ADC_CLKConfig(ADC1, ADC_CLK_Div16);

    // Configure ADC
    ADC_DeInit(ADC1);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = ADC_CHANNEL_COUNT;
    ADC_Init(ADC1, &ADC_InitStructure);

    // Enable ADC
    ADC_Cmd(ADC1, ENABLE);

    // Configure DMA
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->RDATAR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)adc_buffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = ADC_BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);

    // Enable DMA
    DMA_Cmd(DMA1_Channel1, ENABLE);

    // Configure ADC channel
    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL, 1, ADC_SampleTime_11Cycles);

    // Enable ADC DMA request
    ADC_DMACmd(ADC1, ENABLE);

    // Start ADC conversion
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

/**
 * @brief       Find the calibration segment for a given measured value
 * @param       value Measured value to find segment for
 * @param       index Returns the index of the segment start point
 * @return      true: Segment found; false: Out of range
 */
static bool find_calibration_segment(float value, uint16_t *index) {
    // Range check
    if (value < (float)VBUS_CAL_POINTS[0].measured || value > (float)VBUS_CAL_POINTS[VBUS_CAL_POINTS_COUNT - 1].measured) {
        return false;
    }
    // Find segment
    for (uint16_t i = 0; i < VBUS_CAL_POINTS_COUNT - 1; i++) {
        if (value >= (float)VBUS_CAL_POINTS[i].measured && value <= (float)VBUS_CAL_POINTS[i + 1].measured) {
            *index = i;
            return true;
        }
    }
    return false;
}

/**
 * @brief       Convert raw ADC value to VBUS voltage
 * @param       adc_raw Raw ADC sample value
 * @return      VBUS voltage in mV
 */
uint16_t adc_raw_to_vbus_mv(uint32_t adc_raw) {
    float vbus_uncalibrated = (float)adc_raw * VBUS_CONVERT_SCALE;

#if VBUS_CAL_ENABLE
    uint16_t segment_index;
    float vbus_calibrated;
    const CalibrationPoint *p1, *p2;

    if (find_calibration_segment(vbus_uncalibrated, &segment_index)) {
        // Piecewise linear interpolation
        p1 = &VBUS_CAL_POINTS[segment_index];
        p2 = &VBUS_CAL_POINTS[segment_index + 1];

        // Calculate linear interpolation: y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
        float measured_diff = p2->measured - p1->measured;
        float actual_diff = p2->actual - p1->actual;
        float input_offset = vbus_uncalibrated - p1->measured;
        vbus_calibrated = p1->actual + (input_offset * actual_diff) / measured_diff;
    } else {
        // Linear extrapolation when out of calibration range
        if (vbus_uncalibrated < VBUS_CAL_POINTS[0].measured) {
            p1 = &VBUS_CAL_POINTS[0];
            p2 = &VBUS_CAL_POINTS[1];
        } else {
            p1 = &VBUS_CAL_POINTS[VBUS_CAL_POINTS_COUNT - 2];
            p2 = &VBUS_CAL_POINTS[VBUS_CAL_POINTS_COUNT - 1];
        }

        // Calculate slope and extrapolated value: y = y1 + slope * (x - x1)
        float measured_diff = p2->measured - p1->measured;
        float actual_diff = p2->actual - p1->actual;
        float input_offset = vbus_uncalibrated - p1->measured;
        float slope = actual_diff / measured_diff;  // slope = (y2 - y1) / (x2 - x1)
        vbus_calibrated = p1->actual + slope * input_offset;
    }

    // Clamp values
    vbus_calibrated = vbus_calibrated < 0.0f ? 0.0f : vbus_calibrated;
    vbus_calibrated = vbus_calibrated > 65535.0f ? 65535.0f : vbus_calibrated;

#else
    float vbus_calibrated = vbus_uncalibrated;
#endif

    // Round to nearest integer
    return (uint16_t)(vbus_calibrated + 0.5f);
}

/**
 * @brief       Perform oversampling on ADC buffer data
 * @param       None
 * @return      Oversampled ADC value
 */
uint32_t adc_oversample(void) {
    uint32_t sum = 0;
    for (uint16_t i = 0; i < ADC_SAMPLE_COUNT; i++) {
        sum += adc_buffer[i * ADC_CHANNEL_COUNT + 0];
    }
    return sum >> ADC_OVERSAMPLE_BITS;  // Right shift 4 bits to get 16-bit value
}

/**
 * @brief       Get ADC value
 * @param       None
 * @return      ADC value
 */
uint32_t adc_get_raw(void) {
    return adc_oversample();
}

/**
 * @brief       Get VBUS voltage value
 * @param       None
 * @return      VBUS voltage in mV
 */
uint16_t adc_get_vbus_mv(void) {
    return adc_raw_to_vbus_mv(adc_get_raw());
}