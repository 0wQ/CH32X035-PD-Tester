#include "usbpd_rdo.h"

/******************************************************************************
 * RDO 构建函数
 *****************************************************************************/

void usbpd_rdo_build_fixed(USBPD_RDO_t *rdo, uint8_t position, uint16_t voltage_mv, uint16_t current_ma) {
    rdo->d32 = 0;  // 清零
    rdo->FixedAndVariable.MaxOperatingCurrent10mAunits = current_ma / 10;
    rdo->FixedAndVariable.OperatingCurrentIn10mAunits = current_ma / 10;
    rdo->FixedAndVariable.ObjectPosition = position;
    rdo->FixedAndVariable.USBCommunicationsCapable = 1;
    rdo->FixedAndVariable.NoUSBSuspend = 1;
    rdo->FixedAndVariable.EPRCapable = 1;
}

void usbpd_rdo_build_pps(USBPD_RDO_t *rdo, uint8_t position, uint16_t voltage_mv, uint16_t current_ma) {
    rdo->d32 = 0;  // 清零
    rdo->PPS.OperatingCurrentIn50mAunits = current_ma / 50;
    rdo->PPS.OutputVoltageIn20mVunits = voltage_mv / 20;
    rdo->PPS.ObjectPosition = position;
    rdo->PPS.USBCommunicationsCapable = 1;
    rdo->PPS.NoUSBSuspend = 1;
    rdo->PPS.EPRCapable = 1;
}

void usbpd_rdo_build_avs(USBPD_RDO_t *rdo, uint8_t position, uint16_t voltage_mv, uint16_t current_ma) {
    rdo->d32 = 0;  // 清零
    rdo->AVS.OperatingCurrentIn50mAunits = current_ma / 50;
    rdo->AVS.OutputVoltageIn25mVunits = (voltage_mv / 25) & ~0x3;  // 最低两位清零，步进 100mV
    rdo->AVS.EPRCapable = 1;
    rdo->AVS.NoUSBSuspend = 1;
    rdo->AVS.USBCommunicationsCapable = 1;
    rdo->AVS.ObjectPosition = position;
}

/******************************************************************************
 * 请求消息格式化函数
 *****************************************************************************/

uint8_t usbpd_rdo_format_spr_request(uint8_t *buffer, const USBPD_RDO_t *rdo, uint8_t message_id, USBPD_SpecificationRevision_t pd_version) {
    USBPD_MessageHeader_t header = {0};
    header.MessageHeader.MessageID = message_id;
    header.MessageHeader.MessageType = USBPD_DATA_MSG_REQUEST;
    header.MessageHeader.NumberOfDataObjects = 1;
    header.MessageHeader.SpecificationRevision = pd_version;

    buffer[0] = header.d16 & 0xFF;
    buffer[1] = (header.d16 >> 8) & 0xFF;
    buffer[2] = rdo->d32 & 0xFF;
    buffer[3] = (rdo->d32 >> 8) & 0xFF;
    buffer[4] = (rdo->d32 >> 16) & 0xFF;
    buffer[5] = (rdo->d32 >> 24) & 0xFF;

    return 6;  // 返回数据长度
}

uint8_t usbpd_rdo_format_epr_request(uint8_t *buffer, const USBPD_RDO_t *rdo, uint32_t pdo_raw, uint8_t message_id) {
    USBPD_MessageHeader_t header = {0};
    header.MessageHeader.MessageType = USBPD_DATA_MSG_EPR_REQUEST;
    header.MessageHeader.NumberOfDataObjects = 2;
    header.MessageHeader.MessageID = message_id;
    header.MessageHeader.SpecificationRevision = USBPD_SPECIFICATION_REV3;

    buffer[0] = header.d16 & 0xFF;
    buffer[1] = (header.d16 >> 8) & 0xFF;

    // RDO
    buffer[2] = rdo->d32 & 0xFF;
    buffer[3] = (rdo->d32 >> 8) & 0xFF;
    buffer[4] = (rdo->d32 >> 16) & 0xFF;
    buffer[5] = (rdo->d32 >> 24) & 0xFF;

    // PDO 副本
    buffer[6] = pdo_raw & 0xFF;
    buffer[7] = (pdo_raw >> 8) & 0xFF;
    buffer[8] = (pdo_raw >> 16) & 0xFF;
    buffer[9] = (pdo_raw >> 24) & 0xFF;

    return 10;  // 返回数据长度
}