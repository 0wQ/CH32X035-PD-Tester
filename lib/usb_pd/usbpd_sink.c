#include "usbpd_sink.h"

#include <stdint.h>
#include "ch32x035.h"
#include "utils_print.h"
#include "utils_delay.h"

#if 0
#define pd_printf(format, ...) LOG(format, ##__VA_ARGS__)
#else
#define pd_printf(x...)
#endif

/******************************************************************************
 * Static Variables
 *****************************************************************************/

static pd_control_t pd_control_g;
static uint8_t usbpd_rx_buffer[USBPD_DATA_MAX_LEN] __attribute__((aligned(4)));
static uint8_t usbpd_tx_buffer[USBPD_DATA_MAX_LEN] __attribute__((aligned(4)));

/******************************************************************************
 * Function Declaration
 *****************************************************************************/

static void usbpd_sink_rx_mode(void);
static void usbpd_sink_phy_send_data(uint8_t *buffer, uint8_t length, uint8_t sop);

/******************************************************************************
 * Basic Function
 *****************************************************************************/

static USBPD_CC_State_t usbpd_sink_check_cc_connect(void) {
    USBPD_CC_State_t ccLine = USBPD_CCNONE;

    USBPD->PORT_CC1 &= ~(CC_CMP_Mask | PA_CC_AI);
    USBPD->PORT_CC1 |= CC_CMP_22;
    delay_us(2);
    if (USBPD->PORT_CC1 & PA_CC_AI) {
        ccLine = USBPD_CC1;
    }

    USBPD->PORT_CC2 &= ~(CC_CMP_Mask | PA_CC_AI);
    USBPD->PORT_CC2 |= CC_CMP_22;
    delay_us(2);
    if (USBPD->PORT_CC2 & PA_CC_AI) {
        ccLine = USBPD_CC2;
    }

    // 恢复为 CC_CMP_66
    USBPD->PORT_CC1 &= ~(CC_CMP_Mask | PA_CC_AI);
    USBPD->PORT_CC1 |= CC_CMP_66;
    USBPD->PORT_CC2 &= ~(CC_CMP_Mask | PA_CC_AI);
    USBPD->PORT_CC2 |= CC_CMP_66;

    return ccLine;
}

static void usbpd_sink_rx_mode(void) {
    // 清除全部状态
    USBPD->CONFIG |= PD_ALL_CLR;
    USBPD->CONFIG &= ~PD_ALL_CLR;

    // 设置为接收模式
    USBPD->BMC_CLK_CNT = UPD_TMR_RX_48M;
    USBPD->DMA = (uint32_t)usbpd_rx_buffer;

    // 开始接收
    USBPD->CONTROL &= ~PD_TX_EN;
    USBPD->CONTROL |= BMC_START;
}

static void usbpd_sink_state_reset(void) {
    NVIC_DisableIRQ(USBPD_IRQn);

    // USBPD->PORT_CC1 = CC_CMP_66;
    // USBPD->PORT_CC2 = CC_CMP_66;

    // 重置就绪状态
    pd_control_g.is_ready = false;

    // 重置 PD 版本
    pd_control_g.pd_version = DEF_PD_REVISION_30;

    // 重置状态机
    pd_control_g.pd_state = PD_STATE_CHECK_CONNECT;
    pd_control_g.pd_last_state = PD_STATE_CHECK_CONNECT;

    // 重置 PDO
    pd_control_g.pdo_pos = 1;
    pd_control_g.available_pdos.pdo_count = 0;
    pd_control_g.spr_source_cap_buffer_pdo_count = 0;
    pd_control_g.epr_source_cap_buffer_pdo_count = 0;
    pd_control_g.epr_source_cap_buffer_size = 0;
    pd_control_g.epr_source_cap_buffer_chunk_number = 0;

    // 重置 CC 计数
    pd_control_g.cc_none_times = 0;
    pd_control_g.cc1_connect_times = 0;
    pd_control_g.cc2_connect_times = 0;

    // 重置 Message ID
    pd_control_g.sink_message_id = 0;
    pd_control_g.cable_message_id = 0;

    // 重置 GoodCRC 状态
    // pd_control_g.sink_goodcrc_over = true;
    // pd_control_g.source_goodcrc_over = true;

    // 重置 EPR 相关变量
    pd_control_g.is_epr_ready = false;
    pd_control_g.source_epr_capable = false;  // 默认 source 不支持 EPR，后续根据判断
    pd_control_g.cable_epr_capable = true;    // 默认 cable 支持 EPR

    // 重置定时器
    pd_control_g.epr_keepalive_timer = 0;
    pd_control_g.pps_periodic_timer = 0;
}

static void usbpd_sink_phy_send_data(uint8_t *buffer, uint8_t length, uint8_t sop) {
    delay_us(90);  // 确保 GoodCRC 已发送

    USBPD->BMC_CLK_CNT = UPD_TMR_TX_48M;
    USBPD->TX_SEL = sop;
    USBPD->DMA = (uint32_t)buffer;
    USBPD->BMC_TX_SZ = length;

    USBPD->STATUS |= IF_TX_END;
    USBPD->CONTROL |= PD_TX_EN;
    USBPD->CONTROL |= BMC_START;

    while ((USBPD->STATUS & IF_TX_END) == 0);
    USBPD->STATUS |= IF_TX_END;

    usbpd_sink_rx_mode();
}

// 废弃
static void usbpd_sink_wait_goodcrc_and_send_data(uint8_t *buffer, uint8_t length) {
    // 等待接收 source goodcrc
    // while (!pd_control_g.source_goodcrc_over);

    // 发送 SOP0 数据包
    // usbpd_sink_phy_send_data(buffer, length, UPD_SOP0);

    // 重新标记 source goodcrc 未收到
    // pd_control_g.source_goodcrc_over = false;
}

static void usbpd_sink_send_goodcrc(uint8_t message_id, bool is_cable) {
    USBPD_MessageHeader_t header = {0};
    header.MessageHeader.MessageType = USBPD_CONTROL_MSG_GOODCRC;
    header.MessageHeader.PortDataRole = 0;
    header.MessageHeader.SpecificationRevision = pd_control_g.pd_version;
    header.MessageHeader.PortPowerRole_CablePlug = is_cable;  // Cable Plug
    header.MessageHeader.MessageID = message_id;              // GoodCRC 回复相同的 MessageID
    header.MessageHeader.NumberOfDataObjects = 0;
    header.MessageHeader.Extended = 0;

    *(uint16_t *)&usbpd_tx_buffer[0] = header.d16;
    usbpd_sink_phy_send_data(usbpd_tx_buffer, 2, is_cable ? UPD_SOP1 : UPD_SOP0);

    // 在发送 SOP0 类型 GoodCRC 后，重置定时器
    if (!is_cable) {
        pd_control_g.epr_keepalive_timer = 0;
        pd_control_g.pps_periodic_timer = 0;
    }
}

void usbpd_sink_hard_reset(void) {
    usbpd_sink_phy_send_data(usbpd_tx_buffer, 0, UPD_HARD_RESET);
    usbpd_sink_state_reset();
}

bool usbpd_sink_get_ready(void) {
    return pd_control_g.is_ready;
}

bool usbpd_sink_get_epr_ready(void) {
    return pd_control_g.is_epr_ready;
}

/******************************************************************************
 * Initialization Function
 *****************************************************************************/

static void timer_init(void) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};
    TIM_TimeBaseInitStructure.TIM_Period = 1000 - 1;  // 1ms
    TIM_TimeBaseInitStructure.TIM_Prescaler = 48 - 1;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0x00;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    NVIC_SetPriority(TIM3_IRQn, 0x10);
    NVIC_EnableIRQ(TIM3_IRQn);

    TIM_Cmd(TIM3, ENABLE);
}

void usbpd_sink_init(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBPD, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    AFIO->CTLR |= USBPD_IN_HVT | USBPD_PHY_V33;

    // 设置 CC 以正常 VDD 电压驱动输出
    USBPD->PORT_CC1 &= ~CC_LVE;
    USBPD->PORT_CC2 &= ~CC_LVE;

    // 清除全部状态
    USBPD->STATUS = BUF_ERR | IF_RX_BIT | IF_RX_BYTE | IF_RX_ACT | IF_RX_RESET | IF_TX_END;

    // 开启发送完成中断、接收完成中断、接收复位中断
    USBPD->CONFIG = IE_TX_END | IE_RX_ACT | IE_RX_RESET | PD_DMA_EN;

    // PD Sink
    USBPD->PORT_CC1 = CC_CMP_66 | CC_PD;
    USBPD->PORT_CC2 = CC_CMP_66 | CC_PD;

    usbpd_sink_rx_mode();

    NVIC_SetPriority(USBPD_IRQn, 0);
    NVIC_EnableIRQ(USBPD_IRQn);

    timer_init();
}

/******************************************************************************
 * SPR Function
 *****************************************************************************/

static void usbpd_sink_spr_fixed_request(uint8_t position) {
    // 先找到最大的 SPR Fixed PDO
    uint8_t max_spr_fixed_pdo_pos = 0;
    uint16_t max_spr_fixed_voltage = 0;
    for (uint8_t i = 0; i < pd_control_g.available_pdos.pdo_count; i++) {
        pd_pdo_t *current_pdo = &pd_control_g.available_pdos.pdo[i];
        if (current_pdo->pdo_type == PDO_TYPE_FIXED_SUPPLY && current_pdo->fixed.voltage <= 20000) {
            if (current_pdo->fixed.voltage >= max_spr_fixed_voltage) {
                max_spr_fixed_voltage = current_pdo->fixed.voltage;
                max_spr_fixed_pdo_pos = current_pdo->position;
            }
        }
    }

    // 如果请求的位置超出范围，使用最大的 SPR Fixed PDO
    if (position > max_spr_fixed_pdo_pos && max_spr_fixed_pdo_pos > 0) {
        pd_printf("Position %d out of range, using max SPR fixed PDO at position %d\n", position, max_spr_fixed_pdo_pos);
        position = max_spr_fixed_pdo_pos;
    }

    // 查找对应位置的 PDO
    pd_pdo_t *pdo = NULL;
    for (uint8_t i = 0; i < pd_control_g.available_pdos.pdo_count; i++) {
        if (pd_control_g.available_pdos.pdo[i].position == position &&
            pd_control_g.available_pdos.pdo[i].pdo_type == PDO_TYPE_FIXED_SUPPLY) {
            pdo = &pd_control_g.available_pdos.pdo[i];
            break;
        }
    }

    // 未找到对应的 PDO
    if (pdo == NULL) return;

    // 检查 SPR Fixed PDO 电压应小于等于 20V
    if (pdo->fixed.voltage > 20000) {
        return;
    }

    USBPD_MessageHeader_t header = {0};
    header.MessageHeader.MessageID = pd_control_g.sink_message_id;
    header.MessageHeader.MessageType = USBPD_DATA_MSG_REQUEST;
    header.MessageHeader.NumberOfDataObjects = 1;
    header.MessageHeader.SpecificationRevision = pd_control_g.pd_version;

    USBPD_RDO_t rdo = {0};
    rdo.FixedAndVariable.MaxOperatingCurrent10mAunits = pdo->fixed.current / 10;
    rdo.FixedAndVariable.OperatingCurrentIn10mAunits = pdo->fixed.current / 10;
    rdo.FixedAndVariable.ObjectPosition = position;
    rdo.FixedAndVariable.USBCommunicationsCapable = 1;
    rdo.FixedAndVariable.NoUSBSuspend = 1;
    rdo.FixedAndVariable.EPRCapable = 1;

    usbpd_tx_buffer[0] = header.d16 & 0xFF;
    usbpd_tx_buffer[1] = (header.d16 >> 8) & 0xFF;
    usbpd_tx_buffer[2] = rdo.d32 & 0xff;
    usbpd_tx_buffer[3] = (rdo.d32 >> 8) & 0xff;
    usbpd_tx_buffer[4] = (rdo.d32 >> 16) & 0xff;
    usbpd_tx_buffer[5] = (rdo.d32 >> 24) & 0xff;

    // 发送之后再打印日志有可能会漏 source 回复的 goodcrc, 导致 message id 未自增
    pd_printf("Sending SPR request:\n  Position: %d\n  Msg Header: 0x%04x\n  SPR FRDO: 0x%08x\n", position, header.d16, rdo.d32);
    usbpd_sink_phy_send_data(usbpd_tx_buffer, 6, UPD_SOP0);
}

static void usbpd_sink_spr_pps_request(uint8_t position) {
    // 查找对应位置的 PDO
    pd_pdo_t *pdo = NULL;
    for (uint8_t i = 0; i < pd_control_g.available_pdos.pdo_count; i++) {
        if (pd_control_g.available_pdos.pdo[i].position == position &&
            pd_control_g.available_pdos.pdo[i].pdo_type == PDO_TYPE_APDO &&
            pd_control_g.available_pdos.pdo[i].apdo_subtype == APDO_TYPE_SPR_PPS) {
            pdo = &pd_control_g.available_pdos.pdo[i];
            break;
        }
    }

    // 未找到对应的 PDO
    if (pdo == NULL) return;

    USBPD_MessageHeader_t header = {0};
    header.MessageHeader.MessageID = pd_control_g.sink_message_id;
    header.MessageHeader.MessageType = USBPD_DATA_MSG_REQUEST;
    header.MessageHeader.NumberOfDataObjects = 1u;
    header.MessageHeader.SpecificationRevision = pd_control_g.pd_version;

    USBPD_RDO_t rdo = {0};
    rdo.PPS.OperatingCurrentIn50mAunits = pdo->pps.current / 50;
    rdo.PPS.OutputVoltageIn20mVunits = pdo->pps.max_voltage / 20;
    rdo.PPS.ObjectPosition = position;
    rdo.PPS.USBCommunicationsCapable = 1;
    rdo.PPS.NoUSBSuspend = 1;
    rdo.PPS.EPRCapable = 1;

    usbpd_tx_buffer[0] = header.d16 & 0xFF;
    usbpd_tx_buffer[1] = (header.d16 >> 8) & 0xFF;
    usbpd_tx_buffer[2] = rdo.d32 & 0xff;
    usbpd_tx_buffer[3] = (rdo.d32 >> 8) & 0xff;
    usbpd_tx_buffer[4] = (rdo.d32 >> 16) & 0xff;
    usbpd_tx_buffer[5] = (rdo.d32 >> 24) & 0xff;

    // 发送之后再打印日志有可能会漏 source 回复的 goodcrc, 导致 message id 未自增
    pd_printf("Sending SPR request:\n  Position: %d\n  Msg Header: 0x%04x\n  SPR PPS RDO: 0x%08x\n", position, header.d16, rdo.d32);
    usbpd_sink_phy_send_data(usbpd_tx_buffer, 6, UPD_SOP0);
}

static void usbpd_sink_spr_request(uint8_t position) {
    pd_pdo_t *pdo = NULL;
    for (uint8_t i = 0; i < pd_control_g.available_pdos.pdo_count; i++) {
        if (pd_control_g.available_pdos.pdo[i].position == position) {
            pdo = &pd_control_g.available_pdos.pdo[i];
            break;
        }
    }
    // 未找到对应的 PDO
    if (pdo == NULL) return;

    // 发送 SPR PDO 请求
    if (pdo->pdo_type == PDO_TYPE_FIXED_SUPPLY) {
        usbpd_sink_spr_fixed_request(position);
    }
    if (pdo->pdo_type == PDO_TYPE_APDO && pdo->apdo_subtype == APDO_TYPE_SPR_PPS) {
        usbpd_sink_spr_pps_request(position);
    }
}

/******************************************************************************
 * EPR Mode Function
 *****************************************************************************/

static void usbpd_sink_epr_keep_alive(void) {
    // 检查是否已进入 EPR 模式
    if (!pd_control_g.is_epr_ready) return;

    USBPD_MessageHeader_t header = {0};
    header.MessageHeader.MessageType = ExtendedMessageType_ExtendedControl;
    header.MessageHeader.SpecificationRevision = USBPD_SPECIFICATION_REV3;
    header.MessageHeader.NumberOfDataObjects = 1;
    header.MessageHeader.Extended = 1;
    header.MessageHeader.MessageID = pd_control_g.sink_message_id;

    // Extended header
    USBPD_ExtendedMessageHeader_t ext_header = {0};
    ext_header.ExtendedMessageHeader.Chunked = 1;
    ext_header.ExtendedMessageHeader.ChunkNumber = 0;
    ext_header.ExtendedMessageHeader.RequestChunk = 0;
    ext_header.ExtendedMessageHeader.DataSize = 2;

    // Extended Control Message
    usbpd_tx_buffer[0] = header.d16 & 0xFF;
    usbpd_tx_buffer[1] = (header.d16 >> 8) & 0xFF;
    usbpd_tx_buffer[2] = ext_header.d16 & 0xFF;
    usbpd_tx_buffer[3] = (ext_header.d16 >> 8) & 0xFF;
    usbpd_tx_buffer[4] = 0x03;  // EPR_KeepAlive
    usbpd_tx_buffer[5] = 0x00;  // Reserved

    pd_printf("Sending EPR keep alive\n");
    usbpd_sink_phy_send_data(usbpd_tx_buffer, 6, UPD_SOP0);
    pd_control_g.pd_state = PD_STATE_WAIT_EPR_KEEP_ALIVE_ACK;
}

static void usbpd_sink_epr_fixed_request(uint8_t position) {
    // 查找最大 Fixed PDO, 电压小于等于 36V
    uint8_t max_fixed_pdo_pos = 0;
    uint16_t max_fixed_voltage = 0;
    for (uint8_t i = 0; i < pd_control_g.available_pdos.pdo_count; i++) {
        pd_pdo_t *current_pdo = &pd_control_g.available_pdos.pdo[i];
        if (current_pdo->pdo_type == PDO_TYPE_FIXED_SUPPLY && current_pdo->fixed.voltage <= 36000) {
            if (current_pdo->fixed.voltage >= max_fixed_voltage) {
                max_fixed_voltage = current_pdo->fixed.voltage;
                max_fixed_pdo_pos = current_pdo->position;
            }
        }
    }

    // 如果请求的位置超出范围，使用最大的 Fixed PDO
    if (position > max_fixed_pdo_pos && max_fixed_pdo_pos > 0) {
        pd_printf("Position %d out of range, using max EPR fixed PDO at position %d\n", position, max_fixed_pdo_pos);
        position = max_fixed_pdo_pos;
    }

    // 查找对应位置的 PDO
    pd_pdo_t *pdo = NULL;
    for (uint8_t i = 0; i < pd_control_g.available_pdos.pdo_count; i++) {
        if (pd_control_g.available_pdos.pdo[i].position == position &&
            pd_control_g.available_pdos.pdo[i].pdo_type == PDO_TYPE_FIXED_SUPPLY) {
            pdo = &pd_control_g.available_pdos.pdo[i];
            break;
        }
    }

    // 未找到对应的 PDO
    if (pdo == NULL) return;

    // 构建消息头
    USBPD_MessageHeader_t header = {0};
    header.MessageHeader.MessageType = USBPD_DATA_MSG_EPR_REQUEST;
    header.MessageHeader.NumberOfDataObjects = 2;
    header.MessageHeader.MessageID = pd_control_g.sink_message_id;
    header.MessageHeader.SpecificationRevision = USBPD_SPECIFICATION_REV3;

    // 构建 RDO
    USBPD_RDO_t rdo = {0};
    rdo.FixedAndVariable.ObjectPosition = position;
    rdo.FixedAndVariable.EPRCapable = 1;
    rdo.FixedAndVariable.NoUSBSuspend = 1;
    rdo.FixedAndVariable.USBCommunicationsCapable = 1;
    rdo.FixedAndVariable.MaxOperatingCurrent10mAunits = pdo->fixed.current / 10;
    rdo.FixedAndVariable.OperatingCurrentIn10mAunits = pdo->fixed.current / 10;

    usbpd_tx_buffer[0] = header.d16 & 0xFF;
    usbpd_tx_buffer[1] = (header.d16 >> 8) & 0xFF;

    usbpd_tx_buffer[2] = rdo.d32 & 0xff;
    usbpd_tx_buffer[3] = (rdo.d32 >> 8) & 0xff;
    usbpd_tx_buffer[4] = (rdo.d32 >> 16) & 0xff;
    usbpd_tx_buffer[5] = (rdo.d32 >> 24) & 0xff;

    // EPR 请求需要包含 PDO 的副本
    usbpd_tx_buffer[6] = pdo->raw & 0xff;
    usbpd_tx_buffer[7] = (pdo->raw >> 8) & 0xff;
    usbpd_tx_buffer[8] = (pdo->raw >> 16) & 0xff;
    usbpd_tx_buffer[9] = (pdo->raw >> 24) & 0xff;

    pd_printf("Sending EPR request:\n  Position: %d\n  Msg Header: 0x%04x\n  EPR RDO: 0x%08x\n  Copy of PDO: 0x%08x\n", position, header.d16, rdo.d32, pdo->raw);
    usbpd_sink_phy_send_data(usbpd_tx_buffer, 10, UPD_SOP0);
}

static void usbpd_sink_epr_pps_request(uint8_t position) {
    // 查找对应位置的 PDO
    pd_pdo_t *pdo = NULL;
    for (uint8_t i = 0; i < pd_control_g.available_pdos.pdo_count; i++) {
        if (pd_control_g.available_pdos.pdo[i].position == position &&
            pd_control_g.available_pdos.pdo[i].pdo_type == PDO_TYPE_APDO &&
            pd_control_g.available_pdos.pdo[i].apdo_subtype == APDO_TYPE_SPR_PPS) {
            pdo = &pd_control_g.available_pdos.pdo[i];
            break;
        }
    }

    // 未找到对应的 PDO
    if (pdo == NULL) return;

    USBPD_MessageHeader_t header = {0};
    header.MessageHeader.MessageType = USBPD_DATA_MSG_EPR_REQUEST;
    header.MessageHeader.NumberOfDataObjects = 2;
    header.MessageHeader.MessageID = pd_control_g.sink_message_id;
    header.MessageHeader.SpecificationRevision = USBPD_SPECIFICATION_REV3;

    USBPD_RDO_t rdo = {0};
    rdo.PPS.OperatingCurrentIn50mAunits = pdo->pps.current / 50;
    rdo.PPS.OutputVoltageIn20mVunits = pdo->pps.max_voltage / 20;
    rdo.PPS.EPRCapable = 1;
    rdo.PPS.ObjectPosition = position;
    rdo.PPS.USBCommunicationsCapable = 1;
    rdo.PPS.NoUSBSuspend = 1;

    usbpd_tx_buffer[0] = header.d16 & 0xFF;
    usbpd_tx_buffer[1] = (header.d16 >> 8) & 0xFF;

    usbpd_tx_buffer[2] = rdo.d32 & 0xff;
    usbpd_tx_buffer[3] = (rdo.d32 >> 8) & 0xff;
    usbpd_tx_buffer[4] = (rdo.d32 >> 16) & 0xff;
    usbpd_tx_buffer[5] = (rdo.d32 >> 24) & 0xff;

    // EPR 请求需要包含 PDO 的副本
    usbpd_tx_buffer[6] = pdo->raw & 0xff;
    usbpd_tx_buffer[7] = (pdo->raw >> 8) & 0xff;
    usbpd_tx_buffer[8] = (pdo->raw >> 16) & 0xff;
    usbpd_tx_buffer[9] = (pdo->raw >> 24) & 0xff;

    pd_printf("Sending EPR request:\n  Position: %d\n  Msg Header: 0x%04x\n  EPR RDO: 0x%08x\n  Copy of PDO: 0x%08x\n", position, header.d16, rdo.d32, pdo->raw);
    usbpd_sink_phy_send_data(usbpd_tx_buffer, 10, UPD_SOP0);
}

static void usbpd_sink_epr_avs_request(uint8_t position) {
    // 查找对应位置的 PDO
    pd_pdo_t *pdo = NULL;
    for (uint8_t i = 0; i < pd_control_g.available_pdos.pdo_count; i++) {
        if (pd_control_g.available_pdos.pdo[i].position == position &&
            pd_control_g.available_pdos.pdo[i].pdo_type == PDO_TYPE_APDO &&
            pd_control_g.available_pdos.pdo[i].apdo_subtype == APDO_TYPE_EPR_AVS) {
            pdo = &pd_control_g.available_pdos.pdo[i];
            break;
        }
    }

    // 未找到对应的 PDO
    if (pdo == NULL) return;

    // 构建消息头
    USBPD_MessageHeader_t header = {0};
    header.MessageHeader.MessageType = USBPD_DATA_MSG_EPR_REQUEST;
    header.MessageHeader.NumberOfDataObjects = 2;
    header.MessageHeader.MessageID = pd_control_g.sink_message_id;
    header.MessageHeader.SpecificationRevision = USBPD_SPECIFICATION_REV3;

    uint16_t current_ma = ((uint32_t)pdo->epr_avs.pdp * 1000000U) / pdo->epr_avs.max_voltage;

    // 构建 RDO
    USBPD_RDO_t rdo = {0};
    rdo.AVS.OperatingCurrentIn50mAunits = current_ma / 50;
    rdo.AVS.OutputVoltageIn25mVunits = (pdo->epr_avs.max_voltage / 25) & ~0x3;  // 输出电压以 25mV 为单位，最低两位有效位必须设为零，从而使有效的电压步进尺寸为 100mV。
    // rdo.AVS.OutputVoltageIn25mVunits = (23000 / 25) & ~0x3;  // 测试写死 23V
    rdo.AVS.EPRCapable = 1;
    rdo.AVS.NoUSBSuspend = 1;
    rdo.AVS.USBCommunicationsCapable = 1;
    rdo.AVS.ObjectPosition = position;

    usbpd_tx_buffer[0] = header.d16 & 0xFF;
    usbpd_tx_buffer[1] = (header.d16 >> 8) & 0xFF;

    usbpd_tx_buffer[2] = rdo.d32 & 0xff;
    usbpd_tx_buffer[3] = (rdo.d32 >> 8) & 0xff;
    usbpd_tx_buffer[4] = (rdo.d32 >> 16) & 0xff;
    usbpd_tx_buffer[5] = (rdo.d32 >> 24) & 0xff;

    // EPR 请求需要包含 PDO 的副本
    usbpd_tx_buffer[6] = pdo->raw & 0xff;
    usbpd_tx_buffer[7] = (pdo->raw >> 8) & 0xff;
    usbpd_tx_buffer[8] = (pdo->raw >> 16) & 0xff;
    usbpd_tx_buffer[9] = (pdo->raw >> 24) & 0xff;

    pd_printf("Sending EPR request:\n  Position: %d\n  Msg Header: 0x%04x\n  EPR RDO: 0x%08x\n  Copy of PDO: 0x%08x\n", position, header.d16, rdo.d32, pdo->raw);
    usbpd_sink_phy_send_data(usbpd_tx_buffer, 10, UPD_SOP0);
}

static void usbpd_sink_epr_request(uint8_t position) {
    // 检查是否已进入 EPR 模式
    if (!pd_control_g.is_epr_ready) {
        pd_printf("Not in EPR mode\n");
        return;
    }

    // 查找对应位置的 PDO
    pd_pdo_t *pdo = NULL;
    for (uint8_t i = 0; i < pd_control_g.available_pdos.pdo_count; i++) {
        if (pd_control_g.available_pdos.pdo[i].position == position) {
            pdo = &pd_control_g.available_pdos.pdo[i];
        }
    }

    // 未找到对应的 PDO
    if (pdo == NULL) {
        pd_printf("No PDO found at position %d\n", position);
        return;
    };

    switch (pdo->pdo_type) {
        case PDO_TYPE_FIXED_SUPPLY:
            usbpd_sink_epr_fixed_request(position);
            break;
        case PDO_TYPE_APDO:
            if (pdo->apdo_subtype == APDO_TYPE_SPR_PPS) {
                usbpd_sink_epr_pps_request(position);
            }
            if (pdo->apdo_subtype == APDO_TYPE_EPR_AVS) {
                usbpd_sink_epr_avs_request(position);
            }
            break;
        default:
            break;
    }
}

/******************************************************************************
 * USB PD Protocol Parsing and State Machine Handler
 *****************************************************************************/

/**
 * @brief Source Power Data Objects 解析函数
 * @param pdos Source Power Data Objects 数据指针
 * @param pdo_count PDO 数量
 * @param is_epr 是否是 EPR Capabilities Message
 * @note 解析 pdos 将可用的 pdo 存储到 pd_control_g.available_pdos 中
 */
static void usbpd_sink_pdos_analyse(const uint32_t *pdos, const uint8_t pdo_count, const bool is_epr) {
    // EPR 能力消息中的功率数据对象应按以下顺序发送：
    // 1) 如 SPR 能力消息中所报告的 SPR (A)PDOs，EPR 能力消息的消息头中的数据对象数量字段应与 SPR 能力消息的消息头中的数据对象数量字段相同
    // 2) 如果 SPR 能力消息中包含的 PDOs 少于 7 个，则未使用的数据对象应填充为零
    // 3) 如第 6.4.1 Capabilities Message 中定义的 EPR (A)PDOs 应从数据对象位置 8 开始，并按以下顺序发送：
    //     a) 如果存在，提供 28V、36V 或 48V 的固定供电 PDO 应按电压顺序发送，从低到高
    //     b) 应发送一个 EPRAVS APDO

    if (pdo_count == 0) return;
    pd_printf(is_epr ? "EPR Source Capabilities:\n" : "SPR Source Capabilities:\n");

    // 重置 PDO 计数
    pd_control_g.available_pdos.pdo_count = 0;

    // 解析 PDO
    USBPD_SourcePDO_t pdo_parser = {0};
    for (uint8_t i = 0; i < pdo_count; i++) {
        pdo_parser.d32 = pdos[i];
        pd_pdo_t *pdo = &pd_control_g.available_pdos.pdo[pd_control_g.available_pdos.pdo_count];

        // 保存原始 PDO 数据
        pdo->raw = pdos[i];
        pdo->position = i + 1;  // PDO 位置从 1 开始

        // 如果是 EPR 类型，position 最小应从 8 开始
        if (pdo->position < 8) {
            // EPR Fixed
            if (pdo_parser.General.PDO_Type == PDO_TYPE_FIXED_SUPPLY && POWER_DECODE_50MV(pdo_parser.Fixed.VoltageIn50mVunits) > 20000) {
                pdo->position = 8;
            }
            // EPR AVS
            if (pdo_parser.General.PDO_Type == PDO_TYPE_APDO && pdo_parser.General.APDO_SubType == APDO_TYPE_EPR_AVS) {
                pdo->position = 8;
            }
        }

        pd_printf("  PDO[#%d][RAW:0x%08x]: ", i + 1, pdo_parser.d32);

        // 首先检查 PDO 是否为空
        if (pdo_parser.d32 == 0) {
            pd_printf("Empty\n");
            continue;
        }

        switch (pdo_parser.General.PDO_Type) {
            case PDO_TYPE_FIXED_SUPPLY: {
                uint16_t voltage = POWER_DECODE_50MV(pdo_parser.Fixed.VoltageIn50mVunits);
                uint16_t current = POWER_DECODE_10MA(pdo_parser.Fixed.MaxCurrentIn10mAunits);

                pd_control_g.available_pdos.pdo_count++;
                pd_control_g.source_epr_capable = pdo_parser.Fixed.EPRCapable ? 1 : pd_control_g.source_epr_capable;  // EPRCapable=1 可能只在 5V PDO 中出现，后续可能为 0
                pdo->pdo_type = PDO_TYPE_FIXED_SUPPLY;
                pdo->fixed.voltage = voltage;
                pdo->fixed.current = current;
                pdo->fixed.epr_capable = pdo_parser.Fixed.EPRCapable;

                pd_printf("%s Fixed: %dmV, %dmA%s\n", voltage <= 20000 ? "SPR" : "EPR", voltage, current, pdo_parser.Fixed.EPRCapable ? " (EPR Capable)" : "");
                break;
            }
            case PDO_TYPE_BATTERY: {
                uint16_t max_voltage = POWER_DECODE_50MV(pdo_parser.Battery.MaxVoltageIn50mVunits);
                uint16_t min_voltage = POWER_DECODE_50MV(pdo_parser.Battery.MinVoltageIn50mVunits);
                uint16_t max_power = POWER_DECODE_250MW(pdo_parser.Battery.MaxAllowablePowerIn250mWunits);
                pd_printf("Battery: %d-%dmV, %dmW\n", min_voltage, max_voltage, max_power);
                break;
            }
            case PDO_TYPE_VARIABLE_SUPPLY: {
                uint16_t max_voltage = POWER_DECODE_50MV(pdo_parser.Variable.MaxVoltageIn50mVunits);
                uint16_t min_voltage = POWER_DECODE_50MV(pdo_parser.Variable.MinVoltageIn50mVunits);
                uint16_t current = POWER_DECODE_10MA(pdo_parser.Variable.MaxCurrentIn10mAunits);
                pd_printf("Variable Supply: %d-%dmV, %dmA\n", min_voltage, max_voltage, current);
                break;
            }
            case PDO_TYPE_APDO: {
                switch (pdo_parser.General.APDO_SubType) {
                    case APDO_TYPE_SPR_PPS: {
                        uint16_t min_voltage = POWER_DECODE_100MV(pdo_parser.SPR_PPS.MinVoltageIn100mVunits);
                        uint16_t max_voltage = POWER_DECODE_100MV(pdo_parser.SPR_PPS.MaxVoltageIn100mVunits);
                        uint16_t current = POWER_DECODE_50MA(pdo_parser.SPR_PPS.MaxCurrentIn50mAunits);

                        pd_control_g.available_pdos.pdo_count++;
                        pdo->pdo_type = PDO_TYPE_APDO;
                        pdo->apdo_subtype = APDO_TYPE_SPR_PPS;
                        pdo->pps.min_voltage = min_voltage;
                        pdo->pps.max_voltage = max_voltage;
                        pdo->pps.current = current;

                        pd_printf("SPR PPS: %d-%dmV, %dmA\n", min_voltage, max_voltage, current);
                        break;
                    }
                    case APDO_TYPE_EPR_AVS: {
                        uint16_t pdp = pdo_parser.EPR_AVS.PDPIn1Wunits;
                        uint16_t min_voltage = POWER_DECODE_100MV(pdo_parser.EPR_AVS.MinVoltageIn100mVunits);
                        uint16_t max_voltage = POWER_DECODE_100MV(pdo_parser.EPR_AVS.MaxVoltageIn100mVunits);

                        pd_control_g.available_pdos.pdo_count++;
                        pdo->pdo_type = PDO_TYPE_APDO;
                        pdo->apdo_subtype = APDO_TYPE_EPR_AVS;
                        pdo->epr_avs.pdp = pdp;
                        pdo->epr_avs.min_voltage = min_voltage;
                        pdo->epr_avs.max_voltage = max_voltage;

                        pd_printf("EPR AVS: %d-%dmV, %dW\n", min_voltage, max_voltage, pdp);
                        break;
                    }
                    case APDO_TYPE_SPR_AVS: {
                        uint16_t max_current_9v_15v = pdo_parser.SPR_AVS.MaxCurrentFor9V15VIn10mAunits;
                        uint16_t max_current_15v_20v = pdo_parser.SPR_AVS.MaxCurrentFor15V20VIn10mAunits;

                        // 无设备测试，暂时不处理 SPR AVS APDO
                        // pd_control_g.available_pdos.pdo_count++;
                        // pdo->pdo_type = PDO_TYPE_APDO;
                        // pdo->apdo_subtype = APDO_TYPE_SPR_AVS;

                        pd_printf("SPR AVS: 9V-15V, %dmA | 15V-20V, %dmA\n", max_current_9v_15v, max_current_15v_20v);
                        break;
                    }
                    case APDO_TYPE_RESERVED: {
                        pd_printf("Reserved APDO type\n");
                        break;
                    }
                }
                break;
            }
        }
    }
}

/**
 * @brief PD 状态机处理函数
 * @note 需在 TIM3_IRQHandler 中调用
 */
static void usbpd_sink_state_process(void) {
    pd_state_t _pd_state = pd_control_g.pd_state;

    if (pd_control_g.pd_last_state != _pd_state) {
        pd_printf("PD State: %d -> %d\n", pd_control_g.pd_last_state, pd_control_g.pd_state);
    }

    switch (pd_control_g.pd_state) {
        case PD_STATE_DISCONNECTED: {
            usbpd_sink_state_reset();
            break;
        }
        case PD_STATE_CONNECT: {
            // 确保进入此状态后只执行一次
            if (pd_control_g.pd_last_state != pd_control_g.pd_state) {
                usbpd_sink_rx_mode();
                NVIC_EnableIRQ(USBPD_IRQn);
            }
            break;
        }
        case PD_STATE_RECEIVED_SPR_SOURCE_CAP: {
            usbpd_sink_pdos_analyse(pd_control_g.spr_source_cap_buffer, pd_control_g.spr_source_cap_buffer_pdo_count, false);
            pd_control_g.pd_state = PD_STATE_SEND_SPR_REQUEST;
            break;
        }
        case PD_STATE_SEND_SPR_REQUEST: {
            usbpd_sink_spr_request(pd_control_g.pdo_pos);
            pd_control_g.pd_state = PD_STATE_WAIT_ACCEPT;
            break;
        }
        case PD_STATE_RECEIVED_PS_RDY: {
            pd_control_g.is_ready = true;

#ifdef EPR_MODE_ENABLE
            // 如果未进入 EPR 模式，并且 source 和 cable 都支持 EPR, 则进入 EPR 模式
            if (!pd_control_g.is_epr_ready && pd_control_g.source_epr_capable && pd_control_g.cable_epr_capable) {
                pd_printf("Found EPR capable PDO, entering EPR mode\n");
                pd_control_g.pd_state = PD_STATE_SEND_EPR_ENTER;
                break;
            }
#endif

            pd_control_g.pd_state = PD_STATE_IDLE;
            // pd_control_g.pd_state = pd_control_g.is_epr_ready ? PD_STATE_SEND_EPR_REQUEST : PD_STATE_SEND_SPR_REQUEST;  // speedtest
            break;
        }
        case PD_STATE_SEND_EPR_ENTER: {
            pd_control_g.is_epr_ready = false;  // 标记此时还未进入 EPR 模式

            USBPD_MessageHeader_t header = {0};
            header.MessageHeader.MessageType = USBPD_DATA_MSG_EPR_MODE;
            header.MessageHeader.SpecificationRevision = USBPD_SPECIFICATION_REV3;
            header.MessageHeader.NumberOfDataObjects = 1u;
            header.MessageHeader.MessageID = pd_control_g.sink_message_id;

            uint32_t eprmdo = 0;
            eprmdo |= (1u << 24);  // Action(B31-24), Enter(0x01)
            eprmdo |= (0u << 16);  // Data(B23-16)

            *(uint16_t *)&usbpd_tx_buffer[0] = header.d16;
            *(uint32_t *)&usbpd_tx_buffer[2] = eprmdo;

            pd_printf("EPR mode: send Enter\n");
            usbpd_sink_phy_send_data(usbpd_tx_buffer, 6, UPD_SOP0);
            pd_control_g.pd_state = PD_STATE_WAIT_EPR_ENTER_RESPONSE;
            break;
        }
        case PD_STATE_SEND_EPR_SRC_CAP_REQ_CHUNK: {
            USBPD_MessageHeader_t header = {0};
            header.MessageHeader.MessageType = ExtendedMessageType_EPRSourceCapabilities;
            header.MessageHeader.SpecificationRevision = pd_control_g.pd_version;
            header.MessageHeader.MessageID = pd_control_g.sink_message_id;
            header.MessageHeader.NumberOfDataObjects = 1;
            header.MessageHeader.Extended = 1;

            USBPD_ExtendedMessageHeader_t ext_header = {0};
            ext_header.ExtendedMessageHeader.ChunkNumber = pd_control_g.epr_source_cap_buffer_chunk_number + 1;
            ext_header.ExtendedMessageHeader.RequestChunk = 1;
            ext_header.ExtendedMessageHeader.Chunked = 1;
            ext_header.ExtendedMessageHeader.DataSize = 0;

            *(uint16_t *)&usbpd_tx_buffer[0] = header.d16;
            *(uint16_t *)&usbpd_tx_buffer[2] = ext_header.d16;
            usbpd_tx_buffer[4] = 0;
            usbpd_tx_buffer[5] = 0;

            usbpd_sink_phy_send_data(usbpd_tx_buffer, 6, UPD_SOP0);
            pd_control_g.pd_state = PD_STATE_WAIT_EPR_MODE_SOURCE_CAP;
            break;
        }
        case PD_STATE_RECEIVED_EPR_SOURCE_CAP: {
            usbpd_sink_pdos_analyse(pd_control_g.epr_source_cap_buffer, pd_control_g.epr_source_cap_buffer_pdo_count, true);
            pd_control_g.pd_state = PD_STATE_SEND_EPR_REQUEST;
            break;
        }
        case PD_STATE_SEND_EPR_REQUEST: {
            usbpd_sink_epr_request(pd_control_g.pdo_pos);
            pd_control_g.pd_state = PD_STATE_WAIT_ACCEPT;
            break;
        }
        case PD_STATE_SEND_NOT_SUPPORTED: {
            USBPD_MessageHeader_t header = {0};
            header.MessageHeader.MessageID = pd_control_g.sink_message_id;
            header.MessageHeader.MessageType = USBPD_CONTROL_MSG_NOT_SUPPORTED;
            header.MessageHeader.SpecificationRevision = pd_control_g.pd_version;

            *(uint16_t *)&usbpd_tx_buffer[0] = header.d16;

            usbpd_sink_phy_send_data(usbpd_tx_buffer, 2, UPD_SOP0);
            pd_control_g.pd_state = PD_STATE_IDLE;
            break;
        }
        case PD_STATE_SEND_VDM_ACK_DISCOVER_IDENTITY: {
            USBPD_MessageHeader_t header = {0};
            header.MessageHeader.MessageID = pd_control_g.sink_message_id;
            header.MessageHeader.MessageType = USBPD_DATA_MSG_VENDOR_DEFINED;
            header.MessageHeader.SpecificationRevision = pd_control_g.pd_version;
            header.MessageHeader.NumberOfDataObjects = 4;

            *(uint16_t *)&usbpd_tx_buffer[0] = header.d16;
            *(uint32_t *)&usbpd_tx_buffer[2] = 0xFF00A041;
            *(uint32_t *)&usbpd_tx_buffer[6] = 0x18002E99;
            *(uint32_t *)&usbpd_tx_buffer[10] = 0x00000000;
            *(uint32_t *)&usbpd_tx_buffer[14] = 0x00000000;

            usbpd_sink_phy_send_data(usbpd_tx_buffer, (2 + 4 * 4), UPD_SOP0);
            pd_control_g.pd_state = PD_STATE_IDLE;
            break;
        }
        case PD_STATE_SEND_VDM_ACK_DISCOVER_SVIDS: {
            USBPD_MessageHeader_t header = {0};
            header.MessageHeader.MessageID = pd_control_g.sink_message_id;
            header.MessageHeader.MessageType = USBPD_DATA_MSG_VENDOR_DEFINED;
            header.MessageHeader.SpecificationRevision = pd_control_g.pd_version;
            header.MessageHeader.NumberOfDataObjects = 4;

            *(uint16_t *)&usbpd_tx_buffer[0] = header.d16;
            *(uint32_t *)&usbpd_tx_buffer[2] = 0xFF00A042;
            *(uint32_t *)&usbpd_tx_buffer[6] = 0x00000000;
            *(uint32_t *)&usbpd_tx_buffer[10] = 0x04C0C737;
            *(uint32_t *)&usbpd_tx_buffer[14] = 0x00000000;
            *(uint32_t *)&usbpd_tx_buffer[18] = 0x00000000;

            usbpd_sink_phy_send_data(usbpd_tx_buffer, (2 + 5 * 4), UPD_SOP0);
            pd_control_g.pd_state = PD_STATE_IDLE;
            break;
        }
        default: {
            break;
        }
    }

    pd_control_g.pd_last_state = _pd_state;
}

/**
 * @brief 解析 PD SOP0 数据包，并回复 GoodCRC
 * @note 需在 USBPD_IRQHandler 中调用
 */
static void usbpd_sink_protocol_analysis_sop0(const uint8_t *rx_buffer, const uint8_t rx_length) {
    USBPD_MessageHeader_t header = {0};
    header.d16 = *(uint16_t *)rx_buffer;

    bool is_goodcrc_msg = (header.MessageHeader.Extended == 0) &&
                          (header.MessageHeader.NumberOfDataObjects == 0) &&
                          (header.MessageHeader.MessageType == USBPD_CONTROL_MSG_GOODCRC);

    // 如果当前收到消息非 GoodCRC，则需回复 GoodCRC
    if (!is_goodcrc_msg) {
        usbpd_sink_send_goodcrc(header.MessageHeader.MessageID, false);
    }

    // Non-Extended message
    if (header.MessageHeader.Extended == 0) {

        // Control Message (A Message is defined as a Control Message when the Number of Data Objects field in the Message Header is set to zero)
        if (header.MessageHeader.NumberOfDataObjects == 0) {
            switch (header.MessageHeader.MessageType) {
                case USBPD_CONTROL_MSG_GOODCRC: {
                    is_goodcrc_msg = true;
                    pd_control_g.sink_message_id = (pd_control_g.sink_message_id + 1) & 0x07;  // 消息 ID 自增
                    // pd_control_g.source_goodcrc_over = true;                                   // 标记已收到 GoodCRC
                    pd_control_g.epr_keepalive_timer = 0;  // 重置定时器
                    pd_control_g.pps_periodic_timer = 0;   // 重置定时器
                    break;
                }
                case USBPD_CONTROL_MSG_ACCEPT: {
                    pd_control_g.pd_state = PD_STATE_WAIT_PS_RDY;
                    break;
                }
                case USBPD_CONTROL_MSG_PS_RDY: {
                    pd_control_g.pd_state = PD_STATE_RECEIVED_PS_RDY;
                    break;
                }
                case USBPD_CONTROL_MSG_SOFT_RESET: {
                    pd_control_g.pd_state = PD_STATE_DISCONNECTED;
                    usbpd_sink_state_reset();
                    pd_printf("USBPD_CONTROL_MSG_SOFT_RESET\n");
                    break;
                }
                case USBPD_CONTROL_MSG_NOT_SUPPORTED: {
                    // 在发送 EPR MODE Enter 后，如果收到 NOT_SUPPORTED 回复，则认为不支持 EPR
                    if (pd_control_g.pd_state == PD_STATE_WAIT_EPR_ENTER_RESPONSE) {
                        pd_control_g.cable_epr_capable = 0;
                        pd_control_g.source_epr_capable = 0;
                        pd_control_g.pd_state = PD_STATE_IDLE;
                    }
                    break;
                }
                case USBPD_CONTROL_MSG_VCONN_SWAP:
                default: {
                    pd_control_g.pd_state = PD_STATE_SEND_NOT_SUPPORTED;
                    // pd_printf("USBPD_ControlMessageType_t unhandled message type: %d\n", messageHeader->MessageHeader.MessageType);
                    break;
                }
            }
        }

        // Data Message
        if (header.MessageHeader.NumberOfDataObjects > 0) {
            switch (header.MessageHeader.MessageType) {
                case USBPD_DATA_MSG_SRC_CAP: {
                    // 先保存原始数据
                    memcpy(pd_control_g.spr_source_cap_buffer, &rx_buffer[2], header.MessageHeader.NumberOfDataObjects * 4);
                    // 更新 SPR PDO 数量
                    pd_control_g.spr_source_cap_buffer_pdo_count = header.MessageHeader.NumberOfDataObjects;
                    // 在状态机解析 SPR PDO
                    pd_control_g.pd_state = PD_STATE_RECEIVED_SPR_SOURCE_CAP;
                    // 更新 pd version
                    pd_control_g.pd_version = header.MessageHeader.SpecificationRevision;
                    break;
                }
                case USBPD_DATA_MSG_EPR_MODE: {
                    uint32_t eprmdo = *(uint32_t *)&rx_buffer[2];
                    uint8_t action = (eprmdo >> 24) & 0xF;
                    uint8_t data = (eprmdo >> 16) & 0xFF;

                    switch (action) {
                        case 2: {  // Enter Acknowledged
                            pd_control_g.is_epr_ready = false;
                            // pd_printf("EPR mode: Enter Acknowledged\n");
                            break;
                        }
                        case 3: {  // Enter Succeeded
                            pd_control_g.is_epr_ready = true;
                            pd_control_g.pd_state = PD_STATE_WAIT_EPR_MODE_SOURCE_CAP;
                            // pd_printf("EPR mode: Enter Succeeded\n");
                            break;
                        }
                        case 4: {  // Enter Failed
                            pd_control_g.is_epr_ready = false;
                            pd_control_g.cable_epr_capable = 0;  // 认为 Cable 不支持 EPR, TODO: 可能要判断具体失败原因
                            pd_control_g.pd_state = PD_STATE_SEND_SPR_REQUEST;
                            pd_printf("EPR mode: Enter Failed, reason=0x%x\n", data);
                            break;
                        }
                        case 5: {  // Exit
                            pd_control_g.is_epr_ready = false;
                            pd_control_g.cable_epr_capable = 0;  // TODO: 还需要禁止后续收到 PSRDY 又重新进入 EPR
                            pd_control_g.pd_state = PD_STATE_SEND_SPR_REQUEST;
                            pd_printf("EPR mode: Exit\n");
                            break;
                        }
                        default: {
                            pd_printf("EPR mode: Unknown action=0x%x, data=0x%x\n", action, data);
                            break;
                        }
                    }
                    break;
                }
                case USBPD_DATA_MSG_VENDOR_DEFINED: {
                    USBPD_StructuredVDMHeader_t vdm_header = {0};
                    vdm_header.d32 = *(uint32_t *)&rx_buffer[2];

                    pd_printf("SOP0 VDM:\n");
                    pd_printf("  Header: 0x%08x\n", vdm_header.d32);
                    pd_printf("  SVID: 0x%04x\n", vdm_header.StructuredVDMHeader.SVID);
                    pd_printf("  Type: %d\n", vdm_header.StructuredVDMHeader.VDMType);
                    pd_printf("  Version: %d %d\n", vdm_header.StructuredVDMHeader.StructuredVDMVersionMajor, vdm_header.StructuredVDMHeader.StructuredVDMVersionMinor);
                    pd_printf("  Object Position: %d\n", vdm_header.StructuredVDMHeader.ObjectPosition);
                    pd_printf("  Command Type: %d\n", vdm_header.StructuredVDMHeader.CommandType);
                    pd_printf("  Command: %d\n", vdm_header.StructuredVDMHeader.Command);

                    if (vdm_header.StructuredVDMHeader.VDMType == 1) {             // 1 = Structured VDM
                        if (vdm_header.StructuredVDMHeader.CommandType == 0b00) {  // 00b = REQ
                            switch (vdm_header.StructuredVDMHeader.Command) {
                                case 1:  //  Discover Identity
                                    pd_control_g.pd_state = PD_STATE_SEND_VDM_ACK_DISCOVER_IDENTITY;
                                    break;
                                case 2:  // Discover SVIDs
                                    pd_control_g.pd_state = PD_STATE_SEND_VDM_ACK_DISCOVER_SVIDS;
                                    break;
                                default:  // 其他命令类型回复不支持
                                    pd_control_g.pd_state = PD_STATE_SEND_NOT_SUPPORTED;
                                    break;
                            }
                        }
                    }
                    break;
                }
                default: {
                    // pd_printf("USBPD_DataMessageType_t unhandled message type: %d\n", messageHeader->MessageHeader.MessageType);
                    break;
                }
            }
        }
    }

    // Extended message
    if (header.MessageHeader.Extended == 1u) {
        USBPD_ExtendedMessageHeader_t ext_header = {0};
        ext_header.d16 = *(uint16_t *)&rx_buffer[2];

        switch (header.MessageHeader.MessageType) {
            case ExtendedMessageType_EPRSourceCapabilities: {  // 收到 EPR Source Cap
                if (ext_header.ExtendedMessageHeader.Chunked) {
                    uint8_t current_chunk_number = ext_header.ExtendedMessageHeader.ChunkNumber;      // 当前分块号
                    uint8_t chunk_size = rx_length - (2 /*Header*/ + 2 /*ExtHeader*/ + 4 /*CRC32*/);  // 当前分块大小

                    // 收到分块号 0 时重置状态
                    if (current_chunk_number == 0) {
                        pd_control_g.epr_source_cap_buffer_size = 0;
                        pd_control_g.epr_source_cap_buffer_pdo_count = 0;
                        pd_control_g.epr_source_cap_buffer_chunk_number = 0;
                    }

                    // 保存
                    memcpy((uint8_t *)pd_control_g.epr_source_cap_buffer + pd_control_g.epr_source_cap_buffer_size, &rx_buffer[(2 /*Header*/ + 2 /*ExtHeader*/)], chunk_size);
                    pd_control_g.epr_source_cap_buffer_size += chunk_size;  // 更新 buffer size

                    if (chunk_size < 26) {
                        // 全部接收完成，在状态机中解析 EPR_SOURCE_CAP
                        pd_control_g.pd_state = PD_STATE_RECEIVED_EPR_SOURCE_CAP;
                        pd_control_g.epr_source_cap_buffer_chunk_number = 0;                                         // 重置当前分块号
                        pd_control_g.epr_source_cap_buffer_pdo_count = pd_control_g.epr_source_cap_buffer_size / 4;  // 更新 pdo count
                    } else {
                        // 请求下个分块
                        pd_control_g.pd_state = PD_STATE_SEND_EPR_SRC_CAP_REQ_CHUNK;
                        pd_control_g.epr_source_cap_buffer_chunk_number = current_chunk_number;  // 更新 chunk number
                    }
                }
                break;
            }
            case ExtendedMessageType_ExtendedControl: {
                // EPR_KeepAlive_Ack
                if (rx_buffer[4] == 0x04) {
                    pd_printf("EPR mode: Keep Alive Ack received\n");
                    pd_control_g.pd_state = PD_STATE_IDLE;
                }
                break;
            }
            default: {
                // pd_printf("Unhandled extended message type: %d\n", messageHeader->MessageHeader.MessageType);
                break;
            }
        }
    }
}

/**
 * @brief 解析 PD SOP1 数据包，并回复 GoodCRC
 * @note 需在 USBPD_IRQHandler 中调用
 */
static void usbpd_sink_protocol_analysis_sop1(const uint8_t *rx_buffer, const uint8_t rx_length) {
#ifdef E_MARKER_ENABLE
    USBPD_MessageHeader_t header = {0};
    header.d16 = *(uint16_t *)rx_buffer;

    bool is_goodcrc_msg = (header.MessageHeader.Extended == 0) &&
                          (header.MessageHeader.NumberOfDataObjects == 0) &&
                          (header.MessageHeader.MessageType == USBPD_CONTROL_MSG_GOODCRC);

    // 如果当前收到消息非 GoodCRC，则需回复 GoodCRC
    if (!is_goodcrc_msg) {
        usbpd_sink_send_goodcrc(header.MessageHeader.MessageID, true);
    }

    // Non-Extended message
    if (header.MessageHeader.Extended == 0) {

        // Control Message (A Message is defined as a Control Message when the Number of Data Objects field in the Message Header is set to zero)
        if (header.MessageHeader.NumberOfDataObjects == 0) {
            switch (header.MessageHeader.MessageType) {
                case USBPD_CONTROL_MSG_GOODCRC: {
                    is_goodcrc_msg = true;
                    pd_control_g.cable_message_id = (pd_control_g.cable_message_id + 1) & 0x07;  // 消息 ID 自增
                    break;
                }
                default: {
                    break;
                }
            }
        }

        // Data Message
        if (header.MessageHeader.NumberOfDataObjects > 0) {
            // MessageType
            switch (header.MessageHeader.MessageType) {
                case USBPD_DATA_MSG_VENDOR_DEFINED: {
                    USBPD_StructuredVDMHeader_t vdm_header = {0};
                    vdm_header.d32 = *(uint32_t *)&rx_buffer[2];

                    // 需要抢先在 E-Marker 前回复，开日志的话，会导致比 E-Marker 回复慢
                    // pd_printf("SOP1 VDM:\n");
                    // pd_printf("  Header: 0x%08x\n", vdm_header.d32);
                    // pd_printf("  SVID: 0x%04x\n", vdm_header.StructuredVDMHeader.SVID);
                    // pd_printf("  Type: %d\n", vdm_header.StructuredVDMHeader.VDMType);
                    // pd_printf("  Version: %d %d\n", vdm_header.StructuredVDMHeader.StructuredVDMVersionMajor, vdm_header.StructuredVDMHeader.StructuredVDMVersionMinor);
                    // pd_printf("  Object Position: %d\n", vdm_header.StructuredVDMHeader.ObjectPosition);
                    // pd_printf("  Command Type: %d\n", vdm_header.StructuredVDMHeader.CommandType);
                    // pd_printf("  Command: %d\n", vdm_header.StructuredVDMHeader.Command);

                    if ((vdm_header.StructuredVDMHeader.VDMType == 1 /* Structured_VDM */) && (vdm_header.StructuredVDMHeader.CommandType == 0b00 /* REQ */)) {
                        // VDM Command
                        switch (vdm_header.StructuredVDMHeader.Command) {
                            case 1: {  //  Discover Identity
                                USBPD_MessageHeader_t _header;
                                _header.MessageHeader.MessageType = USBPD_DATA_MSG_VENDOR_DEFINED;
                                _header.MessageHeader.PortDataRole = 0;
                                _header.MessageHeader.SpecificationRevision = USBPD_SPECIFICATION_REV3;
                                _header.MessageHeader.PortPowerRole_CablePlug = 1;
                                _header.MessageHeader.MessageID = pd_control_g.cable_message_id;
                                _header.MessageHeader.NumberOfDataObjects = 5;
                                _header.MessageHeader.Extended = 0;

                                *(uint16_t *)&usbpd_tx_buffer[0] = _header.d16;

                                // 酷态科磁吸短线 E-Marker
                                // *(uint32_t *)&usbpd_tx_buffer[2] = 0xFF00A041;
                                // *(uint32_t *)&usbpd_tx_buffer[6] = 0x18002B01;
                                // *(uint32_t *)&usbpd_tx_buffer[10] = 0x00000000;
                                // *(uint32_t *)&usbpd_tx_buffer[14] = 0x40916D14;
                                // *(uint32_t *)&usbpd_tx_buffer[18] = 0x000A0640;

                                // 联想 C135 E-Marker
                                *(uint32_t *)&usbpd_tx_buffer[2] = 0xFF00A041;   // VDM Header
                                *(uint32_t *)&usbpd_tx_buffer[6] = 0x180017EF;   // ID Header VDO
                                *(uint32_t *)&usbpd_tx_buffer[10] = 0x00000000;  // Cert Stat VDO
                                *(uint32_t *)&usbpd_tx_buffer[14] = 0xA4AA0000;  // Product VDO
                                *(uint32_t *)&usbpd_tx_buffer[18] = 0x110A2643;  // Product Type VDO(s)

                                // USBPD_PassiveCableVDO_t vdo = {0};
                                // vdo.PassiveCableVDO.USBHighestSpeed = 0b011;      // 000b = [USB 2.0] only, no SuperSpeed support
                                //                                                   // 001b = [USB 3.2] Gen1
                                //                                                   // 010b = [USB 3.2]/[USB4] Gen2
                                //                                                   // 011b = [USB4] Gen3
                                //                                                   // 100b = [USB4] Gen4
                                // vdo.PassiveCableVDO.VBUSCurrentHandling = 0b10;   // 01b = 3A
                                //                                                   // 10b = 5A
                                // vdo.PassiveCableVDO.MaxVBUSVoltage = 0b11;        // 00b – 20V
                                //                                                   // 01b – 30V (Deprecated)
                                //                                                   // 10b – 40V (Deprecated)
                                //                                                   // 11b – 50V
                                // vdo.PassiveCableVDO.CableTerminationType = 0b00;  // 00b = VCONN not required. Cable Plugs that only support Discover Identity Commands Shall set these bits to 00b.
                                //                                                   // 01b = VCONN required
                                // vdo.PassiveCableVDO.CableLatency = 0b0001;        // 0000b – Reserved and Shall Not be used
                                //                                                   // 0001b – <10ns (~1m)
                                //                                                   // 0010b – 10ns to 20ns (~2m)
                                //                                                   // 0011b – 20ns to 30ns (~3m)
                                //                                                   // 0100b – 30ns to 40ns (~4m)
                                //                                                   // 0101b – 40ns to 50ns (~5m)
                                //                                                   // 0110b – 50ns to 60ns (~6m)
                                //                                                   // 0111b – 60ns to 70ns (~7m)
                                //                                                   // 1000b – > 70ns (>~7m)
                                // vdo.PassiveCableVDO.EPRCapable = 1;               // 0b – Cable is not EPR Capable
                                //                                                   // 1b = Cable is EPR Capable
                                // vdo.PassiveCableVDO.USBTypeCPlug = 2;             // 00b = Reserved and Shall Not be used
                                //                                                   // 01b = Reserved and Shall Not be used
                                //                                                   // 10b = USB Type-C
                                //                                                   // 11b = Captive
                                // vdo.PassiveCableVDO.VDOVersion = 0;               // Version 1.0 = 000b
                                // vdo.PassiveCableVDO.FirmwareVersion = 1;
                                // vdo.PassiveCableVDO.HWVersion = 1;

                                // *(uint32_t *)&usbpd_tx_buffer[18] = vdo.d32;

                                // pd_printf("PassiveCableVDO:\n");
                                // pd_printf("  USBHighestSpeed: %d\n", vdo.PassiveCableVDO.USBHighestSpeed);
                                // pd_printf("  VBUSCurrentHandling: %d\n", vdo.PassiveCableVDO.VBUSCurrentHandling);
                                // pd_printf("  MaxVBUSVoltage: %d\n", vdo.PassiveCableVDO.MaxVBUSVoltage);
                                // pd_printf("  CableTerminationType: %d\n", vdo.PassiveCableVDO.CableTerminationType);
                                // pd_printf("  CableLatency: %d\n", vdo.PassiveCableVDO.CableLatency);
                                // pd_printf("  EPRCapable: %d\n", vdo.PassiveCableVDO.EPRCapable);
                                // pd_printf("  USBTypeCPlug: %d\n", vdo.PassiveCableVDO.USBTypeCPlug);
                                // pd_printf("  VDOVersion: %d\n", vdo.PassiveCableVDO.VDOVersion);
                                // pd_printf("  FirmwareVersion: %d\n", vdo.PassiveCableVDO.FirmwareVersion);
                                // pd_printf("  HWVersion: %d\n", vdo.PassiveCableVDO.HWVersion);

                                usbpd_sink_phy_send_data(usbpd_tx_buffer, (2 + 5 * 4), UPD_SOP1);
                                break;
                            }
                            case 2:  // Discover SVIDs
                            default: {
                                break;
                            }
                        }
                    }
                    break;
                }
                default: {
                    break;
                }
            }
        }
    }
#endif
}

/******************************************************************************
 * Interrupt Handler
 *****************************************************************************/

void USBPD_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USBPD_IRQHandler(void) {
    if (USBPD->STATUS & IF_RX_ACT) {
        USBPD->STATUS |= IF_RX_ACT;

        if (USBPD->BMC_BYTE_CNT >= (2 /*Header*/)) {
            if ((USBPD->STATUS & BMC_AUX_Mask) == BMC_AUX_SOP0) {  //  SOP0
                usbpd_sink_protocol_analysis_sop0(usbpd_rx_buffer, USBPD->BMC_BYTE_CNT);
            }
            if ((USBPD->STATUS & BMC_AUX_Mask) == BMC_AUX_SOP1_HRST) {  //  SOP1 用于实现模拟 E-Marker
                usbpd_sink_protocol_analysis_sop1(usbpd_rx_buffer, USBPD->BMC_BYTE_CNT);
            }
        }
    }

    if (USBPD->STATUS & IF_TX_END) {
        USBPD->STATUS |= IF_TX_END;
        usbpd_sink_rx_mode();
    }

    if (USBPD->STATUS & IF_RX_RESET) {
        USBPD->STATUS |= IF_RX_RESET;
        usbpd_sink_state_reset();
    }
}

void TIM3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM3_IRQHandler(void) {
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

    // tSinkEPRKeepAlive timeout 处理
    if (pd_control_g.is_epr_ready) {
        pd_control_g.epr_keepalive_timer++;

        // 在某些状态需要跳过 keep alive，在错误的时机发送可能 rx reset
        // if (pd_control_g.pd_state != PD_STATE_WAIT_ACCEPT &&
        //     pd_control_g.pd_state != PD_STATE_WAIT_PS_RDY &&
        //     pd_control_g.pd_state != PD_STATE_WAIT_EPR_MODE_ENTER_RESPONSE &&
        //     pd_control_g.pd_state != PD_STATE_WAIT_EPR_MODE_SOURCE_CAP &&
        //     pd_control_g.pd_state != PD_STATE_SEND_EPR_SRC_CAP_REQ_CHUNK &&
        //     pd_control_g.pd_state != PD_STATE_RECEIVED_EPR_SOURCE_CAP) {
        if (pd_control_g.pd_state == PD_STATE_IDLE) {
            if (pd_control_g.epr_keepalive_timer >= tSinkEPRKeepAlive) {
                pd_control_g.epr_keepalive_timer = 0;
                usbpd_sink_epr_keep_alive();
            }
        }
    }

    // tPPSRequest timeout 处理
    USBPD_PDO_Type_t pdo_type;
    USBPD_APDO_Subtype_t apdo_subtype;
    usbpd_sink_get_current_pdo_type(&pdo_type, &apdo_subtype);
    // 判断 spr pps
    if (pdo_type == PDO_TYPE_APDO && apdo_subtype == APDO_TYPE_SPR_PPS && !pd_control_g.is_epr_ready) {
        pd_control_g.pps_periodic_timer++;
        if (pd_control_g.pd_state == PD_STATE_IDLE) {
            if (pd_control_g.pps_periodic_timer >= tPPSRequest) {
                pd_control_g.pps_periodic_timer = 0;
                pd_control_g.pd_state = pd_control_g.is_epr_ready ? PD_STATE_SEND_EPR_REQUEST : PD_STATE_SEND_SPR_REQUEST;
            }
        }
    } else {
        pd_control_g.pps_periodic_timer = 0;
    }

    // 在 PD_STATE_CHECK_CONNECT 状态时检测 CC 连接
    if (pd_control_g.pd_state == PD_STATE_CHECK_CONNECT) {
        USBPD_CC_State_t cc_line = usbpd_sink_check_cc_connect();
        switch (cc_line) {
            case USBPD_CC1: {
                pd_control_g.cc2_connect_times = 0;
                pd_control_g.cc1_connect_times++;
                if (pd_control_g.cc1_connect_times >= 2) {
                    pd_control_g.cc1_connect_times = 0;
                    pd_control_g.pd_state = PD_STATE_CONNECT;
                    USBPD->CONFIG &= ~CC_SEL;
                    pd_printf("Attach: CC1\n");
                }
                break;
            }
            case USBPD_CC2: {
                pd_control_g.cc1_connect_times = 0;
                pd_control_g.cc2_connect_times++;
                if (pd_control_g.cc2_connect_times >= 2) {
                    pd_control_g.cc2_connect_times = 0;
                    pd_control_g.pd_state = PD_STATE_CONNECT;
                    USBPD->CONFIG |= CC_SEL;
                    pd_printf("Attach: CC2\n");
                }
                break;
            }
            case USBPD_CCNONE: {
                pd_control_g.cc1_connect_times = 0;
                pd_control_g.cc2_connect_times = 0;
                break;
            }
        }
    }

    // 处理状态机
    usbpd_sink_state_process();
}

/******************************************************************************
 * Helper Function
 *****************************************************************************/

void usbpd_sink_debug_available_pdos(void) {
    pd_printf("\nSource Capabilities Summary:\n");
    pd_printf("  Total PDOs: %d\n", pd_control_g.available_pdos.pdo_count);
    pd_printf("  EPR Status: %s\n", pd_control_g.is_epr_ready ? "Ready" : "Not Ready");
    pd_printf("  Source EPR Support: %s\n", pd_control_g.source_epr_capable ? "Yes" : "No");
    pd_printf("  Source Power Data Objects:\n");

    for (uint8_t i = 0; i < pd_control_g.available_pdos.pdo_count; i++) {
        const pd_pdo_t *pdo = &pd_control_g.available_pdos.pdo[i];
        pd_printf("  PDO #%d [RAW:0x%08x]: ", pdo->position, pdo->raw);

        switch (pdo->pdo_type) {
            case PDO_TYPE_FIXED_SUPPLY: {
                pd_printf("%s Fixed - %dmV, %dmA%s\n",
                          pdo->fixed.voltage <= 20000 ? "SPR" : "EPR",
                          pdo->fixed.voltage,
                          pdo->fixed.current,
                          pdo->fixed.epr_capable ? " (EPR Capable)" : "");
                break;
            }
            default: {
                // 不支持的 PDO 类型
                pd_printf("Unsupported Type\n");
                break;
            }
        }
    }
    pd_printf("\n");
}

const pd_available_pdos_t *usbpd_sink_get_available_pdos(void) {
    return &pd_control_g.available_pdos;
}

uint8_t usbpd_sink_get_position(void) {
    return pd_control_g.is_ready ? pd_control_g.pdo_pos : 0;
}

bool usbpd_sink_set_position(uint8_t position) {
    if (position == 0) {
        pd_printf("usbpd_sink_set_position(%d): invalid position\n", position);
        return false;
    }

    // 检查指定位置的 PDO 是
    // pd_pdo_t *pdo = NULL;
    // for (uint8_t i = 0; i < pd_control_g.available_pdos.pdo_count; i++) {
    //     if (pd_control_g.available_pdos.pdo[i].position == position) {
    //         pdo = &pd_control_g.available_pdos.pdo[i];
    //         break;
    //     }
    // }
    // if (pdo == NULL) {
    //     pd_printf("usbpd_sink_set_position: Invalid position\n");
    //     return false;
    // }

    // 如果请求的是 EPR PDO, 但 source 或 cable 不支持 EPR, 则忽略请求
    // if (pdo->fixed.voltage > 20000 && !(pd_control_g.source_epr_capable && pd_control_g.cable_epr_capable)) {
    //     pd_printf("usbpd_sink_set_position: EPR not supported\n");
    //     return false;
    // }

    if (!pd_control_g.is_ready) {
        pd_printf("usbpd_sink_set_position(%d): not ready\n", position);
        return false;
    }

    // 等待状态机进入空闲状态，超时后返回
    uint32_t start_time = millis();
    while (pd_control_g.pd_state != PD_STATE_IDLE) {
        if (millis() - start_time > 1000) {
            pd_printf("usbpd_sink_set_position(%d): wait for idle timeout\n", position);
            return false;
        }
    }

    // 设置 PDO Position
    pd_control_g.pdo_pos = position;
    // 判断 SPR/EPR，设置状态机进入对应的 SEND_REQUEST 状态
    pd_control_g.pd_state = pd_control_g.is_epr_ready ? PD_STATE_SEND_EPR_REQUEST : PD_STATE_SEND_SPR_REQUEST;

    pd_printf("usbpd_sink_set_position(%d): success\n", position);
    return true;
}

bool usbpd_sink_get_current_pdo_type(USBPD_PDO_Type_t *pdo_type, USBPD_APDO_Subtype_t *apdo_subtype) {
    // 获取当前 position
    uint8_t position = usbpd_sink_get_position();
    if (position == 0) {
        return false;
    }
    // 根据 position 查找 pdo
    for (uint8_t i = 0; i < pd_control_g.available_pdos.pdo_count; i++) {
        if (pd_control_g.available_pdos.pdo[i].position == position) {
            *pdo_type = pd_control_g.available_pdos.pdo[i].pdo_type;
            *apdo_subtype = pd_control_g.available_pdos.pdo[i].apdo_subtype;
            return true;
        }
    }
    return false;
}
