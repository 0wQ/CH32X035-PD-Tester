#include "adc.h"

#include "ch32x035.h"
#include "debug.h"

uint16_t adc_buffer[ADC_BUFFER_SIZE] __attribute__((aligned(4)));

static void dma_init(DMA_Channel_TypeDef *DMA_CHx, uint32_t padr, uint32_t madr, uint32_t bufsize) {
    DMA_InitTypeDef DMA_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit(DMA_CHx);
    DMA_InitStructure.DMA_PeripheralBaseAddr = padr;
    DMA_InitStructure.DMA_MemoryBaseAddr = madr;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = bufsize;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA_CHx, &DMA_InitStructure);
    DMA_Cmd(DMA_CHx, ENABLE);
}

void adc_init(void) {
    {
        GPIO_InitTypeDef GPIO_InitStructure = {0};
        ADC_InitTypeDef ADC_InitStructure = {0};

        RCC_APB2PeriphClockCmd(ADC_GPIO_CLK, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

        GPIO_InitStructure.GPIO_Pin = ADC_GPIO_PIN;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
        GPIO_Init(ADC_GPIO_PORT, &GPIO_InitStructure);

        ADC_CLKConfig(ADC1, ADC_CLK_Div16);

        ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
        ADC_InitStructure.ADC_ScanConvMode = ENABLE;
        ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
        ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
        ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
        ADC_InitStructure.ADC_NbrOfChannel = ADC_CHANNEL_COUNT;
        ADC_DeInit(ADC1);
        ADC_Init(ADC1, &ADC_InitStructure);

        ADC_Cmd(ADC1, ENABLE);
    }

    dma_init(DMA1_Channel1, (uint32_t)&ADC1->RDATAR, (uint32_t)adc_buffer, ADC_BUFFER_SIZE);

    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL, 1, ADC_SampleTime_11Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 2, ADC_SampleTime_11Cycles);

    ADC_DMACmd(ADC1, ENABLE);

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

/**
 * @brief       在校准点数组中查找给定测量值所在的区间
 * @param       value 待查找的测量值
 * @param       index 返回区间起始点的索引
 * @return      true: 找到区间; false: 超出范围
 */
static bool find_calibration_segment(float value, size_t *index) {
    // 超出范围检查
    if (value < (float)VBUS_CAL_POINTS[0].measured ||
        value > (float)VBUS_CAL_POINTS[VBUS_CAL_POINTS_COUNT - 1].measured) {
        return false;
    }
    // 查找区间
    for (size_t i = 0; i < VBUS_CAL_POINTS_COUNT - 1; i++) {
        if (value >= (float)VBUS_CAL_POINTS[i].measured &&
            value <= (float)VBUS_CAL_POINTS[i + 1].measured) {
            *index = i;
            return true;
        }
    }
    return false;
}

/**
 * @brief       将 ADC 原始值转换为 VBUS 电压值
 * @param       adc_raw 原始 ADC 采样值
 * @return      VBUS 电压值，单位 mV
 */
uint16_t adc_raw_to_vbus_mv(uint16_t adc_raw) {
    float vbus_uncalibrated = (float)adc_raw * VBUS_CONVERT_SCALE;
    float vbus_calibrated;

    if (VBUS_CAL_ENABLE) {
        size_t segment_index;
        if (find_calibration_segment(vbus_uncalibrated, &segment_index)) {
            // 分段线性插值
            const CalibrationPoint *p1 = &VBUS_CAL_POINTS[segment_index];
            const CalibrationPoint *p2 = &VBUS_CAL_POINTS[segment_index + 1];

            float t = (vbus_uncalibrated - (float)p1->measured) / ((float)p2->measured - (float)p1->measured);
            vbus_calibrated = (float)p1->actual + t * ((float)p2->actual - (float)p1->actual);
        } else {
            // 超出校准范围时，使用最近的校准点进行线性外推
            if (vbus_uncalibrated < (float)VBUS_CAL_POINTS[0].measured) {
                const CalibrationPoint *p1 = &VBUS_CAL_POINTS[0];
                const CalibrationPoint *p2 = &VBUS_CAL_POINTS[1];
                float k = ((float)p2->actual - (float)p1->actual) / ((float)p2->measured - (float)p1->measured);
                vbus_calibrated = (float)p1->actual + k * (vbus_uncalibrated - (float)p1->measured);
            } else {
                const CalibrationPoint *p1 = &VBUS_CAL_POINTS[VBUS_CAL_POINTS_COUNT - 2];
                const CalibrationPoint *p2 = &VBUS_CAL_POINTS[VBUS_CAL_POINTS_COUNT - 1];
                float k = ((float)p2->actual - (float)p1->actual) / ((float)p2->measured - (float)p1->measured);
                vbus_calibrated = (float)p2->actual + k * (vbus_uncalibrated - (float)p2->measured);
            }
        }
    } else {
        vbus_calibrated = vbus_uncalibrated;
    }

    // 限幅并四舍五入
    vbus_calibrated = vbus_calibrated < 0.0f ? 0.0f : vbus_calibrated;
    vbus_calibrated = vbus_calibrated > 65535.0f ? 65535.0f : vbus_calibrated;
    return (uint16_t)(vbus_calibrated + 0.5f);
}

/**
 * @brief       计算 ADC DMA Buffer 的平均值
 * @param       None
 * @return      adc_raw 平均值
 */
uint16_t adc_get_avg_raw() {
    uint32_t sum = 0;

    for (uint16_t i = 0; i < ADC_SAMPLE_COUNT; i++) {
        sum += adc_buffer[i * ADC_CHANNEL_COUNT + 0];
    }

    return (uint16_t)(sum / ADC_SAMPLE_COUNT);
}

/**
 * @brief       获取 VBUS 电压值
 * @param       None
 * @return      VBUS 电压值，单位 mV
 */
uint16_t adc_get_vbus_mv(void) {
    return adc_raw_to_vbus_mv(adc_get_avg_raw());
}

/**
 * @brief       获取 VDD 电压值
 * @param       None
 * @return      VDD 电压值，单位 mV
 */
uint16_t adc_get_vdd_mv(void) {
    // for (uint16_t i = 0; i < ADC_BUFFER_SIZE; i++) {
    //     printf("%d,", adc_buffer[i]);
    // }
    // printf("\n");

    uint32_t sum = 0;

    for (uint16_t i = 0; i < ADC_SAMPLE_COUNT; i++) {
        sum += adc_buffer[i * ADC_CHANNEL_COUNT + 1];
    }

    uint16_t vdd_adc_raw = sum / ADC_SAMPLE_COUNT;
    uint16_t vdd_mv = 1200 * 4095 / vdd_adc_raw;
    return vdd_mv;
}