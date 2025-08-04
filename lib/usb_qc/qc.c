#include "qc.h"

#include "ch32x035.h"

static void qc_dm_set(usb_qc_dp_dm_voltage_t voltage) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_16;

    switch (voltage) {
        case USB_QC_DP_DM_VOLTAGE_0V0:
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
            GPIO_Init(GPIOC, &GPIO_InitStructure);
            AFIO->CTLR &= ~AFIO_CTLR_UDM_BC_VSRC;
            break;

        case USB_QC_DP_DM_VOLTAGE_0V6:
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
            GPIO_Init(GPIOC, &GPIO_InitStructure);
            AFIO->CTLR |= AFIO_CTLR_UDM_BC_VSRC;
            break;

        case USB_QC_DP_DM_VOLTAGE_3V3:
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
            GPIO_Init(GPIOC, &GPIO_InitStructure);
            AFIO->CTLR &= ~AFIO_CTLR_UDM_BC_VSRC;
            AFIO->CTLR |= AFIO_CTLR_USB_IOEN | AFIO_CTLR_UDM_PUE_1 | AFIO_CTLR_UDM_PUE_0;
            break;
    }
}

static void qc_dp_set(usb_qc_dp_dm_voltage_t voltage) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_17;

    switch (voltage) {
        case USB_QC_DP_DM_VOLTAGE_0V0:
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
            GPIO_Init(GPIOC, &GPIO_InitStructure);
            AFIO->CTLR &= ~AFIO_CTLR_UDP_BC_VSRC;
            break;

        case USB_QC_DP_DM_VOLTAGE_0V6:
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
            GPIO_Init(GPIOC, &GPIO_InitStructure);
            AFIO->CTLR |= AFIO_CTLR_UDP_BC_VSRC;
            break;

        case USB_QC_DP_DM_VOLTAGE_3V3:
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
            GPIO_Init(GPIOC, &GPIO_InitStructure);
            AFIO->CTLR &= ~AFIO_CTLR_UDP_BC_VSRC;
            AFIO->CTLR |= AFIO_CTLR_USB_IOEN | AFIO_CTLR_UDP_PUE_1 | AFIO_CTLR_UDP_PUE_0;
            break;
    }
}

void usb_qc_request(usb_qc_voltage_t voltage) {
    switch (voltage) {
        case USB_QC_VOLTAGE_5V:
            qc_dm_set(USB_QC_DP_DM_VOLTAGE_0V0);
            qc_dp_set(USB_QC_DP_DM_VOLTAGE_0V6);
            break;
        case USB_QC_VOLTAGE_9V:
            qc_dm_set(USB_QC_DP_DM_VOLTAGE_0V6);
            qc_dp_set(USB_QC_DP_DM_VOLTAGE_3V3);
            break;
        case USB_QC_VOLTAGE_12V:
            qc_dm_set(USB_QC_DP_DM_VOLTAGE_0V6);
            qc_dp_set(USB_QC_DP_DM_VOLTAGE_0V6);
            break;
        case USB_QC_VOLTAGE_20V:
            qc_dm_set(USB_QC_DP_DM_VOLTAGE_3V3);
            qc_dp_set(USB_QC_DP_DM_VOLTAGE_3V3);
            break;
        default:
            break;
    }
}