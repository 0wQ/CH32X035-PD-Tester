#include "qc.h"

void vQcDmSet(uint8_t ucValue) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_16;

    switch (ucValue) {
        case 0:
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
            GPIO_Init(GPIOC, &GPIO_InitStructure);
            AFIO->CTLR &= ~AFIO_CTLR_UDM_BC_VSRC;
            break;

        case 1:
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
            GPIO_Init(GPIOC, &GPIO_InitStructure);
            AFIO->CTLR |= AFIO_CTLR_UDM_BC_VSRC;
            break;

        case 2:
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
            GPIO_Init(GPIOC, &GPIO_InitStructure);
            AFIO->CTLR &= ~AFIO_CTLR_UDM_BC_VSRC;
            AFIO->CTLR |= AFIO_CTLR_USB_IOEN | AFIO_CTLR_UDM_PUE_1 | AFIO_CTLR_UDM_PUE_0;
            break;
    }
}

void vQcDpSet(uint8_t ucValue) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_17;

    switch (ucValue) {
        case 0:
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
            GPIO_Init(GPIOC, &GPIO_InitStructure);
            AFIO->CTLR &= ~AFIO_CTLR_UDP_BC_VSRC;
            break;

        case 1:
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
            GPIO_Init(GPIOC, &GPIO_InitStructure);
            AFIO->CTLR |= AFIO_CTLR_UDP_BC_VSRC;
            break;

        case 2:
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
            GPIO_Init(GPIOC, &GPIO_InitStructure);
            AFIO->CTLR &= ~AFIO_CTLR_UDP_BC_VSRC;
            AFIO->CTLR |= AFIO_CTLR_USB_IOEN | AFIO_CTLR_UDP_PUE_1 | AFIO_CTLR_UDP_PUE_0;
            break;
    }
}

void vQcRequest5V(void) {
    vQcDmSet(USB_QC_VOLTAGE_0V0);
    vQcDpSet(USB_QC_VOLTAGE_0V6);
}

void vQcRequest9V(void) {
    vQcDmSet(USB_QC_VOLTAGE_0V6);
    vQcDpSet(USB_QC_VOLTAGE_3V3);
}

void vQcRequest12V(void) {
    vQcDmSet(USB_QC_VOLTAGE_0V6);
    vQcDpSet(USB_QC_VOLTAGE_0V6);
}

void vQcRequest20V(void) {
    vQcDmSet(USB_QC_VOLTAGE_3V3);
    vQcDpSet(USB_QC_VOLTAGE_3V3);
}
