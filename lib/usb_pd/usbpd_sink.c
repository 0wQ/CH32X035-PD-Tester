#include "usbpd_sink.h"

#include <string.h>

#include "debug.h"

#if 1
#define pd_printf(format, ...) printf(format, ##__VA_ARGS__)
#else
#define pd_printf(x...)
#endif

static pd_control_t pdControl_g = {
    .cc_State = CC_IDLE,          // CC 状态机当前状态
    .cc1_ConnectTimes = 0,        // CC1 连接检测计数
    .cc2_ConnectTimes = 0,        // CC2 连接检测计数
    .cc_EPR_Ready = 0,            // EPR 模式就绪标志
    .cc_EPR_KeepAlive_Timer = 0,  // EPR keep alive 定时器
};

/* PD 接收和发送缓冲区 */
static __attribute__((aligned(4))) uint8_t usbpd_rx_buffer[34];  // PD 接收缓冲区
static __attribute__((aligned(4))) uint8_t usbpd_tx_buffer[34];  // PD 发送缓冲区

/******************************************************************************
 * 基础功能函数
 *****************************************************************************/

/**
 * @brief  获取 PD 通信就绪状态
 * @return bool
 *         @arg true:  就绪
 *         @arg false: 未就绪
 */
bool usbpd_sink_get_ready(void) {
    return pdControl_g.cc_USBPD_READY;
}

/**
 * @brief  清除 PD 通信就绪状态
 */
void usbpd_sink_clear_ready(void) {
    pdControl_g.cc_USBPD_READY = 0;
}

/******************************************************************************
 * 硬件初始化相关函数
 *****************************************************************************/

/**
 * @brief  初始化定时器
 */
static void timer3_init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_TimeBaseInitStructure.TIM_Period = 10000 - 1;
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

/**
 * @brief  初始化 USB PD
 */
void usbpd_sink_init(void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBPD, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    AFIO->CTLR |= USBPD_IN_HVT | USBPD_PHY_V33;

    USBPD->CONFIG = PD_DMA_EN;
    USBPD->STATUS = BUF_ERR | IF_RX_BIT | IF_RX_BYTE | IF_RX_ACT | IF_RX_RESET | IF_TX_END;

    timer3_init();

    NVIC_SetPriority(USBPD_IRQn, 0);
}

/******************************************************************************
 * USB PD 通信函数
 *****************************************************************************/

/**
 * @brief  配置为接收模式
 */
static void usbpd_sink_rx_mode(void) {
    USBPD->CONFIG |= PD_ALL_CLR;
    USBPD->CONFIG &= ~PD_ALL_CLR;
    USBPD->CONFIG |= IE_RX_ACT | IE_RX_RESET | PD_DMA_EN;
    USBPD->DMA = (uint32_t)usbpd_rx_buffer;
    USBPD->CONTROL &= ~PD_TX_EN;
    USBPD->BMC_CLK_CNT = UPD_TMR_RX_48M;
    USBPD->CONTROL |= BMC_START;
}

/**
 * @brief  复位 PD 通信状态
 */
static void usbpd_sink_reset(void) {
    USBPD->PORT_CC1 = CC_CMP_66;
    USBPD->PORT_CC2 = CC_CMP_66;

    // 重置 PDO
    pdControl_g.source_caps.pdo_count = 0;
    pdControl_g.cc_spr_source_cap_buffer_pdo_count = 0;

    pdControl_g.cc1_ConnectTimes = 0;
    pdControl_g.cc2_ConnectTimes = 0;
    pdControl_g.cc_NoneTimes = 0;
    pdControl_g.cc_State = CC_IDLE;
    pdControl_g.cc_LastState = CC_IDLE;
    pdControl_g.cc_SinkMessageID = 0;
    pdControl_g.cc_SinkGoodCRCOver = 0;
    pdControl_g.cc_SourceGoodCRCOver = 0;
    pdControl_g.cc_PD_Version = DEF_PD_REVISION_30;
    pdControl_g.cc_USBPD_READY = 0;

    pdControl_g.cc_PDO_Pos = 1;  // 改为 99 上电会请求最大档位
    pdControl_g.cc_Last_PDO_Pos = 0;

    pdControl_g.cc_Source_EPR_Capable = 0;  // 默认 source 不支持 EPR
    pdControl_g.cc_Cable_EPR_Capable = 1;   // 默认 cable 支持 EPR

    // 重置 EPR 相关变量
    pdControl_g.cc_EPR_Ready = 0;
    pdControl_g.cc_EPR_KeepAlive_Timer = 0;
    pdControl_g.cc_EPRSourceCapBufferSize = 0;
    pdControl_g.cc_CurrentChunkNumber = 0;
    pdControl_g.cc_ChunkRequestSent = 0;
}

/**
 * @brief  发送 PD 数据包
 * @param  pBuf        数据缓冲区指针
 * @param  length      数据长度
 * @param  sop         起始包类型
 */
static void usbpd_sink_phy_send_data(uint8_t* pBuf, uint8_t length, uint8_t sop) {
    USBPD->CONFIG |= IE_TX_END;

    if ((USBPD->CONFIG & CC_SEL) == CC_SEL) {
        USBPD->PORT_CC2 |= CC_LVE;
    } else {
        USBPD->PORT_CC1 |= CC_LVE;
    }

    USBPD->BMC_CLK_CNT = UPD_TMR_TX_48M;
    USBPD->DMA = (uint32_t)usbpd_tx_buffer;
    USBPD->TX_SEL = sop;
    USBPD->BMC_TX_SZ = length;
    USBPD->CONTROL |= PD_TX_EN;
    USBPD->STATUS &= BMC_AUX_INVALID;
    USBPD->CONTROL |= BMC_START;
}

/**
 * @brief  检查 CC 连接状态
 * @return USBPD_CC_State_t CC 连接状态
 *         @arg USBPD_CCNONE: 无连接
 *         @arg USBPD_CC1:    CC1 连接
 *         @arg USBPD_CC2:    CC2 连接
 */
static USBPD_CC_State_t usbpd_sink_check_cc_connect(void) {
    USBPD_CC_State_t ccLine = USBPD_CCNONE;

    USBPD->PORT_CC1 &= ~(CC_CMP_Mask | PA_CC_AI);
    USBPD->PORT_CC1 |= CC_CMP_22;
    if (USBPD->PORT_CC1 & PA_CC_AI) {
        ccLine = USBPD_CC1;
    }

    USBPD->PORT_CC2 &= ~(CC_CMP_Mask | PA_CC_AI);
    USBPD->PORT_CC2 |= CC_CMP_22;
    if (USBPD->PORT_CC2 & PA_CC_AI) {
        ccLine = USBPD_CC2;
    }

    return ccLine;
}

/******************************************************************************
 * SPR 相关函数 - SPR Functions
 *****************************************************************************/

/**
 * @brief  发送 SPR Fixed PDO 请求
 * @param  position   PDO 位置 (1-based), 如果超出范围则自动选择最大 fixed PDO
 * @param  pdControl  PD 控制结构体指针
 * @param  txData     发送缓冲区指针
 */
static void usbpd_sink_spr_fixed_pdo_request(uint8_t position, pd_control_t* pdControl, uint8_t* txData) {
    // 先找到最大的 fixed PDO
    uint8_t max_spr_fixed_pdo_pos = 0;
    uint16_t max_spr_fixed_voltage = 0;
    for (uint8_t i = 0; i < pdControl->source_caps.pdo_count; i++) {
        Source_PDO_t* current_pdo = &pdControl->source_caps.pdos[i];
        if (current_pdo->type == PDO_TYPE_FIXED_SUPPLY && current_pdo->fixed.voltage <= 20000) {
            if (current_pdo->fixed.voltage >= max_spr_fixed_voltage) {
                max_spr_fixed_voltage = current_pdo->fixed.voltage;
                max_spr_fixed_pdo_pos = current_pdo->position;
            }
        }
    }

    // 如果请求的位置超出范围, 使用最大的 SPR fixed PDO
    if (position > max_spr_fixed_pdo_pos && max_spr_fixed_pdo_pos > 0) {
        pd_printf("Position %d out of range, using max SPR fixed PDO at position %d\n", position, max_spr_fixed_pdo_pos);
        position = max_spr_fixed_pdo_pos;
    }

    // 查找对应位置的 PDO
    Source_PDO_t* pdo = NULL;
    for (uint8_t i = 0; i < pdControl->source_caps.pdo_count; i++) {
        if (pdControl->source_caps.pdos[i].position == position) {
            pdo = &pdControl->source_caps.pdos[i];
            break;
        }
    }
    // 未找到对应的 SPR Fixed PDO
    if (pdo == NULL || pdo->type != PDO_TYPE_FIXED_SUPPLY) {
        return;
    }
    // 检查 SPR Fixed 电压应小于等于 20V
    if (pdo->fixed.voltage > 20000) {
        return;
    }

    USBPD_MessageHeader_t messageHeader;
    messageHeader.d16 = 0u;
    messageHeader.MessageHeader.MessageID = pdControl->cc_SinkMessageID;
    messageHeader.MessageHeader.MessageType = USBPD_DATA_MSG_REQUEST;
    messageHeader.MessageHeader.NumberOfDataObjects = 1u;
    messageHeader.MessageHeader.SpecificationRevision = pdControl->cc_PD_Version;

    USBPD_RequestDataObject_t rdo;
    rdo.d32 = 0u;
    rdo.FixedAndVariableRequestDataObject.MaxOperatingCurrent10mAunits = pdo->fixed.current / 10;
    rdo.FixedAndVariableRequestDataObject.OperatingCurrentIn10mAunits = pdo->fixed.current / 10;
    rdo.FixedAndVariableRequestDataObject.ObjectPosition = position;
    rdo.FixedAndVariableRequestDataObject.USBCommunicationsCapable = 1u;
    rdo.FixedAndVariableRequestDataObject.NoUSBSuspend = 1u;

    // 在最后一个请求消息中, RDO 中必须正确设置 EPR Capable, 后续才能进入 EPR 模式
    if (pdControl->cc_Source_EPR_Capable) {
        rdo.FixedAndVariableRequestDataObject.EPRCapable = 1u;
    }

    *(uint16_t*)&txData[0] = messageHeader.d16;
    txData[2] = rdo.d32 & 0xff;
    txData[3] = (rdo.d32 >> 8) & 0xff;
    txData[4] = (rdo.d32 >> 16) & 0xff;
    txData[5] = (rdo.d32 >> 24) & 0xff;

    // 发送之后再打印日志有可能会漏 source 回复的 goodcrc, 导致 message id 未自增
    pd_printf("Sending SPR request:\n  Position: %d\n  Msg Header: 0x%04x\n  SPR FRDO: 0x%08lx\n", position, messageHeader.d16, rdo.d32);

    // pdControl_g.cc_Last_PDO_Pos = position;
    // pdControl_g.cc_PDO_Pos = position;

    usbpd_sink_phy_send_data(txData, 6, UPD_SOP0);
}

/******************************************************************************
 * EPR 相关函数
 *****************************************************************************/

/**
 * @brief  发送 EPR keep alive
 */
static void usbpd_sink_epr_keep_alive(void) {
    USBPD_MessageHeader_t messageHeader;
    messageHeader.d16 = 0;
    messageHeader.MessageHeader.MessageType = ExtendedMessageType_ExtendedControl;
    messageHeader.MessageHeader.SpecificationRevision = USBPD_SPECIFICATION_REV3;
    messageHeader.MessageHeader.NumberOfDataObjects = 1;
    messageHeader.MessageHeader.Extended = 1;
    messageHeader.MessageHeader.MessageID = pdControl_g.cc_SinkMessageID;

    // Extended header
    USBPD_ExtendedMessageHeader_t extHeader;
    extHeader.d16 = 0;
    extHeader.ExtendedMessageHeader.Chunked = 1;
    extHeader.ExtendedMessageHeader.ChunkNumber = 0;
    extHeader.ExtendedMessageHeader.RequestChunk = 0;
    extHeader.ExtendedMessageHeader.DataSize = 2;

    // Extended Control Message
    usbpd_tx_buffer[0] = messageHeader.d16 & 0xFF;
    usbpd_tx_buffer[1] = (messageHeader.d16 >> 8) & 0xFF;
    usbpd_tx_buffer[2] = extHeader.d16 & 0xFF;
    usbpd_tx_buffer[3] = (extHeader.d16 >> 8) & 0xFF;
    usbpd_tx_buffer[4] = 0x03;  // EPR_KeepAlive
    usbpd_tx_buffer[5] = 0x00;  // Reserved

    usbpd_sink_phy_send_data(usbpd_tx_buffer, 6, UPD_SOP0);
}

/**
 * @brief  发送 EPR Fixed PDO 请求
 * @param  position   PDO 位置 (1-based)
 * @param  pdControl  PD 控制结构体指针
 * @param  txData     发送缓冲区指针
 */
static void usbpd_sink_epr_fixed_pdo_request(uint8_t position, pd_control_t* pdControl, uint8_t* txData) {
    // 检查是否已进入 EPR 模式
    if (!pdControl->cc_EPR_Ready) {
        return;
    }

    // 查找最大 Fixed PDO, 电压小于等于 28V
    uint8_t max_fixed_pdo_pos = 0;
    uint16_t max_fixed_voltage = 0;
    for (uint8_t i = 0; i < pdControl->source_caps.pdo_count; i++) {
        Source_PDO_t* current_pdo = &pdControl->source_caps.pdos[i];
        if (current_pdo->type == PDO_TYPE_FIXED_SUPPLY && current_pdo->fixed.voltage <= 28000) {
            if (current_pdo->fixed.voltage >= max_fixed_voltage) {
                max_fixed_voltage = current_pdo->fixed.voltage;
                max_fixed_pdo_pos = current_pdo->position;
            }
        }
    }

    // 如果请求的位置超出范围, 使用最大的 Fixed PDO
    if (position > max_fixed_pdo_pos && max_fixed_pdo_pos > 0) {
        pd_printf("Position %d out of range, using max EPR fixed PDO at position %d\n", position, max_fixed_pdo_pos);
        position = max_fixed_pdo_pos;
    }

    // 构建消息头
    USBPD_MessageHeader_t header;
    header.d16 = 0;
    header.MessageHeader.MessageType = USBPD_DATA_MSG_EPR_REQUEST;
    header.MessageHeader.NumberOfDataObjects = 2;
    header.MessageHeader.MessageID = pdControl->cc_SinkMessageID;
    header.MessageHeader.SpecificationRevision = USBPD_SPECIFICATION_REV3;

    // 查找对应位置的 PDO
    Source_PDO_t* pdo = NULL;
    for (uint8_t i = 0; i < pdControl->source_caps.pdo_count; i++) {
        if (pdControl->source_caps.pdos[i].position == position) {
            pdo = &pdControl->source_caps.pdos[i];
            break;
        }
    }

    if (pdo == NULL) {
        pd_printf("No PDO found at position %d\n", position);
        return;
    }

    if (pdo->type != PDO_TYPE_FIXED_SUPPLY) {
        pd_printf("PDO at position %d is not Fixed Supply (type=%d)\n", position, pdo->type);
        return;
    }

    // 构建 RDO
    USBPD_RequestDataObject_t rdo;
    rdo.d32 = 0;
    rdo.FixedAndVariableRequestDataObject.ObjectPosition = position;
    rdo.FixedAndVariableRequestDataObject.EPRCapable = 1;
    rdo.FixedAndVariableRequestDataObject.NoUSBSuspend = 1;
    rdo.FixedAndVariableRequestDataObject.USBCommunicationsCapable = 1;
    rdo.FixedAndVariableRequestDataObject.MaxOperatingCurrent10mAunits = pdo->fixed.current / 10;
    rdo.FixedAndVariableRequestDataObject.OperatingCurrentIn10mAunits = pdo->fixed.current / 10;

    *(uint16_t*)&txData[0] = header.d16;
    *(uint32_t*)&txData[2] = rdo.d32;

    // EPR 请求需要包含 PDO 的副本, 使用原始 PDO 数据
    *(uint32_t*)&txData[6] = pdo->raw;

    pd_printf("Sending EPR request:\n  Position: %d\n  Msg Header: 0x%04x\n  EPR FRDO: 0x%08lx\n  Copy of PDO: 0x%08lx\n",
              position, header.d16, rdo.d32, pdo->raw);

    // pdControl_g.cc_Last_PDO_Pos = position;
    // pdControl_g.cc_PDO_Pos = position;

    usbpd_sink_phy_send_data(txData, 10, UPD_SOP0);
}

/******************************************************************************
 * USB PD 协议分析与状态机处理函数
 *****************************************************************************/

/**
 * @brief  PDO 分析
 * @param  pdo_data    PDO 数据指针
 * @param  pdo_num     PDO 数量
 * @param  is_epr      是否是 EPR
 * @param  pdControl   PD 控制结构体指针
 */
static void usbpd_sink_pdo_analyse(uint8_t* pdo_data, uint8_t pdo_num, bool is_epr, pd_control_t* pdControl) {
    if (pdo_num == 0 || pdo_data == NULL) {
        return;
    }

    if (!is_epr) {
        pdControl->cc_Source_EPR_Capable = 0;  // 重置 EPR 支持标志
        pd_printf("SPR Source Capabilities:\n");
    } else {
        pd_printf("EPR Source Capabilities:\n");
    }

    pdControl->source_caps.pdo_count = 0;  // 重置 PDO 计数

    USBPD_SourcePDO_t pdo_parse;
    for (uint8_t i = 0; i < pdo_num; i++) {
        pdo_parse.d32 = *(uint32_t*)(&pdo_data[i * 4]);
        Source_PDO_t* pdo = &pdControl->source_caps.pdos[pdControl->source_caps.pdo_count];

        pd_printf("  PDO[#%d][RAW:0x%08lx]: ", i + 1, pdo_parse.d32);

        // 保存原始PDO数据
        pdo->raw = pdo_parse.d32;
        pdo->position = i + 1;

        // 首先检查 PDO 是否为空
        if (pdo_parse.d32 == 0) {
            pd_printf("Empty\n");
            continue;
        }

        switch (pdo_parse.General.PDO_Type) {
            case PDO_TYPE_FIXED_SUPPLY: {
                uint16_t voltage = POWER_DECODE_50MV(pdo_parse.SourceFixedSupplyPDO.VoltageIn50mVunits);
                uint16_t current = POWER_DECODE_10MA(pdo_parse.SourceFixedSupplyPDO.MaxCurrentIn10mAunits);

                pdo->type = PDO_TYPE_FIXED_SUPPLY;
                pdo->fixed.voltage = voltage;
                pdo->fixed.current = current;
                pdo->fixed.epr_capable = pdo_parse.SourceFixedSupplyPDO.EPRCapable;

                pd_printf("%s Fixed: %dmV, %dmA", voltage <= 20000 ? "SPR" : "EPR", voltage, current);

                if (pdo_parse.SourceFixedSupplyPDO.EPRCapable) {
                    pdControl->cc_Source_EPR_Capable = 1;
                    pd_printf(" (EPR Capable)");
                }
                pd_printf("\n");

                pdControl->source_caps.pdo_count++;
                break;
            }
            case PDO_TYPE_BATTERY: {
                uint16_t max_voltage = POWER_DECODE_50MV(pdo_parse.SourceBatterySupplyPDO.MaxVoltageIn50mVunits);
                uint16_t min_voltage = POWER_DECODE_50MV(pdo_parse.SourceBatterySupplyPDO.MinVoltageIn50mVunits);
                uint16_t max_power = POWER_DECODE_250MW(pdo_parse.SourceBatterySupplyPDO.MaxAllowablePowerIn250mWunits);
                pd_printf("Battery: %d-%dmV, %dmW\n", min_voltage, max_voltage, max_power);
                break;
            }
            case PDO_TYPE_VARIABLE_SUPPLY: {
                uint16_t max_voltage = POWER_DECODE_50MV(pdo_parse.SourceVariableSupplyPDO.MaxVoltageIn50mVunits);
                uint16_t min_voltage = POWER_DECODE_50MV(pdo_parse.SourceVariableSupplyPDO.MinVoltageIn50mVunits);
                uint16_t current = POWER_DECODE_10MA(pdo_parse.SourceVariableSupplyPDO.MaxCurrentIn10mAunits);
                pd_printf("Variable Supply: %d-%dmV, %dmA\n", min_voltage, max_voltage, current);
                break;
            }
            case PDO_TYPE_APDO: {
                switch (pdo_parse.General.SubTypeOrOtherUsage) {
                    case APDO_TYPE_SPR_PPS: {
                        uint16_t min_voltage = POWER_DECODE_100MV(pdo_parse.SourceSPRProgrammablePowerAPDO.MinVoltageIn100mVunits);
                        uint16_t max_voltage = POWER_DECODE_100MV(pdo_parse.SourceSPRProgrammablePowerAPDO.MaxVoltageIn100mVunits);
                        uint16_t current = POWER_DECODE_50MA(pdo_parse.SourceSPRProgrammablePowerAPDO.MaxCurrentIn50mAunits);
                        pd_printf("SPR PPS: %d-%dmV, %dmA\n", min_voltage, max_voltage, current);
                        break;
                    }
                    case APDO_TYPE_EPR_AVS: {
                        uint16_t min_voltage = POWER_DECODE_100MV(pdo_parse.SourceEPRAdjustableVoltageSupplyAPDO.MinVoltageIn100mVunits);
                        uint16_t max_voltage = POWER_DECODE_100MV(pdo_parse.SourceEPRAdjustableVoltageSupplyAPDO.MaxVoltageIn100mVunits);
                        uint8_t pdp = pdo_parse.SourceEPRAdjustableVoltageSupplyAPDO.PDPIn1Wunits;
                        pd_printf("EPR AVS: %d-%dmV, %dW\n", min_voltage, max_voltage, pdp);
                        break;
                    }
                    case APDO_TYPE_SPR_AVS: {
                        uint16_t max_current_9v_15v = pdo_parse.SourceSPRAdjustableVoltageSupplyAPDO.MaxCurrentFor9V15VIn10mAunits;
                        uint16_t max_current_15v_20v = pdo_parse.SourceSPRAdjustableVoltageSupplyAPDO.MaxCurrentFor15V20VIn10mAunits;
                        pd_printf("SPR AVS: 9V-15V: %dmA, 15V-20V: %dmA\n", max_current_9v_15v, max_current_15v_20v);
                        break;
                    }
                    default:
                        pd_printf("Reserved APDO type\n");
                        break;
                }
                break;
            }
        }
    }
}

/**
 * @brief  PD 状态机处理函数
 * @note   由 TIM3_IRQHandler 调用
 */
static void usbpd_sink_process(void) {
    cc_state_t temp = pdControl_g.cc_State;
    USBPD_MessageHeader_t messageHeader;

    switch (pdControl_g.cc_State) {
        case CC_IDLE: {
            NVIC_DisableIRQ(USBPD_IRQn);
            usbpd_sink_reset();
            pdControl_g.cc_State = CC_CHECK_CONNECT;
            break;
        }
        case CC_CONNECT: {
            if (pdControl_g.cc_LastState != pdControl_g.cc_State) {
                usbpd_sink_rx_mode();
                NVIC_SetPriority(USBPD_IRQn, 0);
                NVIC_EnableIRQ(USBPD_IRQn);
            }
            break;
        }
        case CC_SOURCE_CAP: {
            if (pdControl_g.cc_SinkGoodCRCOver) {
                pdControl_g.cc_SinkGoodCRCOver = 0;
                // usbpd_sink_spr_pdo_analyse(usbpd_rx_buffer_pdo_count, &pdControl_g);
                usbpd_sink_pdo_analyse(pdControl_g.cc_spr_source_cap_buffer, pdControl_g.cc_spr_source_cap_buffer_pdo_count, false, &pdControl_g);
                pdControl_g.cc_State = CC_SEND_SPR_REQUEST;
            }
            break;
        }
        case CC_SEND_SPR_REQUEST: {
            if (pdControl_g.cc_LastState != pdControl_g.cc_State) {
                // pdControl_g.cc_Last_PDO_Pos = pdControl_g.cc_PDO_Pos;
                usbpd_sink_spr_fixed_pdo_request(pdControl_g.cc_PDO_Pos, &pdControl_g, usbpd_tx_buffer);
            }

            if (pdControl_g.cc_SourceGoodCRCOver) {
                pdControl_g.cc_SourceGoodCRCOver = 0;
                pdControl_g.cc_State = CC_WAIT_ACCEPT;
            }
            break;
        }
        case CC_PS_RDY: {
            if (pdControl_g.cc_SinkGoodCRCOver) {
                pdControl_g.cc_SinkGoodCRCOver = 0;

                pdControl_g.cc_USBPD_READY = 1;
                pdControl_g.cc_Last_PDO_Pos = pdControl_g.cc_PDO_Pos;

                // 如果已进入 EPR 模式, 则不发送
                if (pdControl_g.cc_EPR_Ready) {
                    pdControl_g.cc_State = CC_WAIT_EPR_RESPONSE;
                    break;
                }

                // 如果 source 和 cable 都支持 EPR, 则进入 EPR 模式
                if (pdControl_g.cc_Source_EPR_Capable && pdControl_g.cc_Cable_EPR_Capable) {
                    pd_printf("Found EPR capable PDO, entering EPR mode\n");
                    pdControl_g.cc_State = CC_SEND_EPR_MODE_ENTER;
                    break;
                }

                // 如果不支持 EPR, 则获取 Source Capability
                // pdControl_g.cc_State = CC_GET_SOURCE_CAP;

                pdControl_g.cc_State = CC_WAIT_PS_RDY;
            }
            break;
        }
        case CC_GET_SOURCE_CAP: {
            // 如果已进入 EPR 模式, 则不发送
            if (pdControl_g.cc_EPR_Ready) {
                pdControl_g.cc_State = CC_WAIT_EPR_RESPONSE;
                break;
            }

            if (pdControl_g.cc_LastState != pdControl_g.cc_State) {
                messageHeader.d16 = 0u;
                messageHeader.MessageHeader.MessageID = pdControl_g.cc_SinkMessageID;
                messageHeader.MessageHeader.MessageType = USBPD_CONTROL_MSG_GET_SRC_CAP;
                messageHeader.MessageHeader.NumberOfDataObjects = 0u;
                messageHeader.MessageHeader.SpecificationRevision = pdControl_g.cc_PD_Version;
                *(uint16_t*)&usbpd_tx_buffer[0] = messageHeader.d16;

                usbpd_sink_phy_send_data(usbpd_tx_buffer, 2, UPD_SOP0);

                pdControl_g.cc_State = CC_WAIT_SOURCE_CAP;
            }
            break;
        }
        case CC_SEND_EPR_MODE_ENTER: {
            if (pdControl_g.cc_LastState != pdControl_g.cc_State) {
                messageHeader.d16 = 0u;
                messageHeader.MessageHeader.MessageType = USBPD_DATA_MSG_EPR_MODE;
                messageHeader.MessageHeader.SpecificationRevision = USBPD_SPECIFICATION_REV3;
                messageHeader.MessageHeader.NumberOfDataObjects = 1u;
                messageHeader.MessageHeader.MessageID = pdControl_g.cc_SinkMessageID;

                uint32_t eprmdo = 0;
                eprmdo |= (1u << 24);  // Action = Enter (0x01) 31, 24;
                eprmdo |= (0u << 16);

                *(uint16_t*)&usbpd_tx_buffer[0] = messageHeader.d16;
                *(uint32_t*)&usbpd_tx_buffer[2] = eprmdo;

                pd_printf("EPR mode: send Enter\n");

                usbpd_sink_phy_send_data(usbpd_tx_buffer, 6, UPD_SOP0);

                pdControl_g.cc_EPR_Ready = 0;
            }

            if (pdControl_g.cc_SourceGoodCRCOver) {
                pdControl_g.cc_SourceGoodCRCOver = 0;
                pdControl_g.cc_State = CC_WAIT_EPR_RESPONSE;
            }
            break;
        }
        case CC_REQUEST_EPR_CHUNK: {
            if (pdControl_g.cc_LastState != pdControl_g.cc_State) {
                pdControl_g.cc_ChunkRequestSent = 0;  // 重置标志位
            }

            if (!pdControl_g.cc_ChunkRequestSent) {
                USBPD_MessageHeader_t reqHeader;
                reqHeader.d16 = 0;
                reqHeader.MessageHeader.MessageType = ExtendedMessageType_EPRSourceCapabilities;
                reqHeader.MessageHeader.SpecificationRevision = USBPD_SPECIFICATION_REV3;
                reqHeader.MessageHeader.MessageID = pdControl_g.cc_SinkMessageID;
                reqHeader.MessageHeader.NumberOfDataObjects = 1;
                reqHeader.MessageHeader.Extended = 1;

                USBPD_ExtendedMessageHeader_t reqExtHeader;
                reqExtHeader.d16 = 0;
                reqExtHeader.ExtendedMessageHeader.ChunkNumber = pdControl_g.cc_CurrentChunkNumber + 1;
                reqExtHeader.ExtendedMessageHeader.RequestChunk = 1;
                reqExtHeader.ExtendedMessageHeader.Chunked = 1;
                reqExtHeader.ExtendedMessageHeader.DataSize = 0;

                *(uint16_t*)&usbpd_tx_buffer[0] = reqHeader.d16;
                *(uint16_t*)&usbpd_tx_buffer[2] = reqExtHeader.d16;
                usbpd_tx_buffer[4] = 0;
                usbpd_tx_buffer[5] = 0;

                usbpd_sink_phy_send_data(usbpd_tx_buffer, 6, UPD_SOP0);
                pdControl_g.cc_ChunkRequestSent = 1;
            }

            if (pdControl_g.cc_SourceGoodCRCOver) {
                pdControl_g.cc_SourceGoodCRCOver = 0;
                pdControl_g.cc_State = CC_WAIT_EPR_RESPONSE;
            }
            break;
        }
        case CC_SEND_EPR_REQUEST: {
            if (pdControl_g.cc_LastState != pdControl_g.cc_State) {
                usbpd_sink_epr_fixed_pdo_request(pdControl_g.cc_PDO_Pos, &pdControl_g, usbpd_tx_buffer);
            }

            if (pdControl_g.cc_SourceGoodCRCOver) {
                pdControl_g.cc_SourceGoodCRCOver = 0;
                pdControl_g.cc_State = CC_WAIT_ACCEPT;
            }
            break;
        }
        default: {
            // pd_printf("cc state: unhandled %d\n", pdControl_g.cc_State);
            break;
        }
    }

    pdControl_g.cc_LastState = temp;
}

/**
 * @brief  分析和处理 PD 协议消息, 并回复 GoodCRC
 * @param  messageHeader  消息头指针
 * @param  pdControl      PD 控制结构体指针
 * @note   由 USBPD_IRQHandler 调用
 */
static void usbpd_sink_protocol_analysis(USBPD_MessageHeader_t* messageHeader, pd_control_t* pdControl) {
    uint8_t sendGoodCRCFlag = 1;

    if (messageHeader->MessageHeader.Extended == 0u) {
        if (messageHeader->MessageHeader.NumberOfDataObjects == 0u) {
            // control message
            switch (messageHeader->MessageHeader.MessageType) {
                case USBPD_CONTROL_MSG_GOODCRC: {
                    sendGoodCRCFlag = 0;
                    pdControl->cc_EPR_KeepAlive_Timer = 0;
                    pdControl->cc_SourceGoodCRCOver = 1;
                    pdControl->cc_SinkMessageID = (pdControl->cc_SinkMessageID + 1) & 0x07;
                    break;
                }
                case USBPD_CONTROL_MSG_ACCEPT: {
                    pdControl->cc_State = CC_WAIT_PS_RDY;
                    break;
                }
                case USBPD_CONTROL_MSG_PS_RDY: {
                    pdControl->cc_State = CC_PS_RDY;
                    break;
                }
                case USBPD_CONTROL_MSG_SOFT_RESET: {
                    pdControl->cc_State = CC_IDLE;
                    usbpd_sink_reset();
                    pd_printf("USBPD_CONTROL_MSG_SOFT_RESET\n");
                    break;
                }
                default: {
                    // pd_printf("USBPD_ControlMessage_t unhandled message type: %d\n", messageHeader->MessageHeader.MessageType);
                    break;
                }
            }
        } else {
            // data message
            switch (messageHeader->MessageHeader.MessageType) {
                case USBPD_DATA_MSG_SRC_CAP: {
                    pdControl->cc_State = CC_SOURCE_CAP;
                    pdControl->cc_PD_Version = messageHeader->MessageHeader.SpecificationRevision;
                    // 先保存原始数据
                    memcpy(pdControl->cc_spr_source_cap_buffer, &usbpd_rx_buffer[2], 28);
                    // 更新 SPR PDO 数量
                    pdControl->cc_spr_source_cap_buffer_pdo_count = messageHeader->MessageHeader.NumberOfDataObjects;
                    // 解析 SPR PDO (直接在这里解析可能会导致收到 SourceCap 后, 未及时回复 Request, 导致 SoftReset)
                    // usbpd_sink_pdo_analyse(pdControl->spr_source_cap_buffer, messageHeader->MessageHeader.NumberOfDataObjects, false, pdControl);
                    break;
                }
                case USBPD_DATA_MSG_EPR_MODE: {
                    uint32_t eprmdo = *(uint32_t*)&usbpd_rx_buffer[2];
                    uint8_t action = (eprmdo >> 24) & 0xF;
                    uint8_t data = (eprmdo >> 16) & 0xFF;

                    switch (action) {
                        case 2:  // Enter Acknowledged
                            pdControl->cc_EPR_Ready = 0;
                            pd_printf("EPR mode: Enter Acknowledged\n");
                            break;
                        case 3:  // Enter Succeeded
                            pdControl->cc_EPR_Ready = 1;
                            pdControl->cc_EPR_KeepAlive_Timer = 0;  // 开始计时
                            pd_printf("EPR mode: Enter Succeeded\n");
                            break;
                        case 4:  // Enter Failed
                            pdControl->cc_EPR_Ready = 0;
                            pdControl->cc_Cable_EPR_Capable = 0;  // Cable 不支持 EPR, TODO: 可能要判断具体识别原因
                            pdControl->cc_State = CC_SEND_SPR_REQUEST;
                            pd_printf("EPR mode: Enter Failed, reason=0x%x\n", data);
                            break;
                        case 5:  // Exit
                            pdControl->cc_EPR_Ready = 0;
                            pdControl->cc_Cable_EPR_Capable = 0;  // TODO: 还需要禁止后续收到 PSRDY 又重新进入 EPR
                            pdControl->cc_State = CC_SEND_SPR_REQUEST;
                            pd_printf("EPR mode: Exit\n");
                            break;
                        default:
                            pd_printf("EPR mode: Unknown action=0x%x, data=0x%x\n", action, data);
                            break;
                    }
                    break;
                }
                default: {
                    // pd_printf("USBPD_DataMessage_t unhandled message type: %d\n", messageHeader->MessageHeader.MessageType);
                    break;
                }
            }
        }
    }

    // 回复 GoodCRC
    if (sendGoodCRCFlag) {
        Delay_Us(90);

        pdControl_g.cc_SinkGoodCRCOver = 0;
        pdControl_g.cc_EPR_KeepAlive_Timer = 0;

        USBPD_MessageHeader_t my_messageHeader;
        my_messageHeader.d16 = 0u;
        my_messageHeader.MessageHeader.MessageID = messageHeader->MessageHeader.MessageID;  // GoodCRC 回复相同的 MessageID
        my_messageHeader.MessageHeader.MessageType = USBPD_CONTROL_MSG_GOODCRC;
        my_messageHeader.MessageHeader.SpecificationRevision = pdControl->cc_PD_Version;

        *(uint16_t*)&usbpd_tx_buffer[0] = my_messageHeader.d16;
        usbpd_sink_phy_send_data(usbpd_tx_buffer, 2, UPD_SOP0);
    }

    // Extended message
    if (messageHeader->MessageHeader.Extended == 1u) {
        USBPD_ExtendedMessageHeader_t extHeader;
        extHeader.d16 = *(uint16_t*)&usbpd_rx_buffer[2];

        switch (messageHeader->MessageHeader.MessageType) {
            case ExtendedMessageType_EPRSourceCapabilities: {
                if (extHeader.ExtendedMessageHeader.Chunked) {
                    uint8_t chunk_number = extHeader.ExtendedMessageHeader.ChunkNumber;

                    if (chunk_number == 0) {
                        pdControl->cc_EPRSourceCapBufferSize = 0;
                    }
                    uint8_t bytes_in_chunk = messageHeader->MessageHeader.NumberOfDataObjects * 4 - 2;
                    memcpy(pdControl->cc_EPRSourceCapBuffer + pdControl->cc_EPRSourceCapBufferSize, &usbpd_rx_buffer[4], bytes_in_chunk);
                    pdControl->cc_EPRSourceCapBufferSize += bytes_in_chunk;

                    if (bytes_in_chunk < 26) {
                        NVIC_DisableIRQ(USBPD_IRQn);
                        usbpd_sink_pdo_analyse(pdControl->cc_EPRSourceCapBuffer, pdControl->cc_EPRSourceCapBufferSize / 4, true, pdControl);
                        NVIC_EnableIRQ(USBPD_IRQn);
                        pdControl->cc_State = CC_SEND_EPR_REQUEST;
                    } else {
                        pdControl->cc_State = CC_REQUEST_EPR_CHUNK;
                        pdControl->cc_CurrentChunkNumber = chunk_number;
                    }
                }
                break;
            }
            case ExtendedMessageType_ExtendedControl: {
                // EPR_KeepAlive_Ack
                // if (usbpd_rx_buffer[4] == 0x04) {
                //     pd_printf("EPR mode: Keep Alive Ack received\n");
                // }
                break;
            }
            default: {
                // pd_printf("Unhandled extended message type: %d\n", messageHeader->MessageHeader.MessageType);
                break;
            }
        }
    }
}

/******************************************************************************
 * 辅助函数
 *****************************************************************************/

/**
 * @brief  打印 pdControl_g.source_caps
 */
void usbpd_sink_print_source_caps(void) {
    printf("\nSource Capabilities Summary:\n");
    printf("  Total PDOs: %d\n", pdControl_g.source_caps.pdo_count);
    printf("  EPR Status: %s\n", pdControl_g.cc_EPR_Ready ? "Ready" : "Not Ready");
    printf("  Source EPR Support: %s\n", pdControl_g.cc_Source_EPR_Capable ? "Yes" : "No");
    printf("  Source Power Data Objects:\n");

    for (uint8_t i = 0; i < pdControl_g.source_caps.pdo_count; i++) {
        const Source_PDO_t* pdo = &pdControl_g.source_caps.pdos[i];
        printf("  PDO #%d [RAW:0x%08lx]: ", pdo->position, pdo->raw);

        switch (pdo->type) {
            case PDO_TYPE_FIXED_SUPPLY: {
                printf("%s Fixed - %dmV, %dmA%s\n",
                       pdo->fixed.voltage <= 20000 ? "SPR" : "EPR",
                       pdo->fixed.voltage,
                       pdo->fixed.current,
                       pdo->fixed.epr_capable ? " (EPR Capable)" : "");
                break;
            }
            default: {
                // 不支持的 PDO 类型
                printf("Unsupported Type\n");
                break;
            }
        }
    }
    printf("\n");
}

/**
 * @brief  获取 Source Capabilities
 * @return Source Capabilities
 */
Source_PDO_Storage_t usbpd_sink_get_source_caps(void) {
    return pdControl_g.source_caps;
}

/******************************************************************************
 * 中断处理函数
 *****************************************************************************/

/**
 * @brief  USB PD 中断处理函数
 */
void USBPD_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USBPD_IRQHandler(void) {
    USBPD_MessageHeader_t messageHeader;

    if (USBPD->STATUS & IF_RX_ACT) {
        messageHeader.d16 = *(uint16_t*)usbpd_rx_buffer;
        if ((USBPD->STATUS & MASK_PD_STAT) == PD_RX_SOP0) {
            if (USBPD->BMC_BYTE_CNT >= 6) {
                usbpd_sink_protocol_analysis(&messageHeader, &pdControl_g);
            }
        }
        USBPD->STATUS |= IF_RX_ACT;
    }
    if (USBPD->STATUS & IF_TX_END) {
        /* Packet send completion interrupt (GoodCRC send completion interrupt only) */
        USBPD->PORT_CC1 &= ~CC_LVE;
        USBPD->PORT_CC2 &= ~CC_LVE;

        usbpd_sink_rx_mode();
        pdControl_g.cc_SinkGoodCRCOver = 1;

        USBPD->STATUS |= IF_TX_END;
    }
    if (USBPD->STATUS & IF_RX_RESET) {
        USBPD->STATUS |= IF_RX_RESET;
        usbpd_sink_reset();
    }
}

/**
 * @brief  定时器中断处理函数
 */
void TIM3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM3_IRQHandler(void) {
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

    // EPR keep alive 处理
    if (pdControl_g.cc_EPR_Ready) {
        pdControl_g.cc_EPR_KeepAlive_Timer++;
        if (pdControl_g.cc_EPR_KeepAlive_Timer >= EPR_KEEP_ALIVE_TIMEOUT) {
            pdControl_g.cc_EPR_KeepAlive_Timer = 0;
            usbpd_sink_epr_keep_alive();
        }
    }

    // 检测 CC 状态
    USBPD_CC_State_t ccLine;
    if (pdControl_g.cc_State == CC_CHECK_CONNECT) {
        ccLine = usbpd_sink_check_cc_connect();
        switch (ccLine) {
            case USBPD_CC1:
                pdControl_g.cc2_ConnectTimes = 0;
                pdControl_g.cc1_ConnectTimes++;
                if (pdControl_g.cc1_ConnectTimes >= 5) {
                    pdControl_g.cc1_ConnectTimes = 0;
                    pdControl_g.cc_State = CC_CONNECT;
                    USBPD->CONFIG &= ~CC_SEL;
                    pd_printf("Attach: CC1\n");
                }
                break;
            case USBPD_CC2:
                pdControl_g.cc1_ConnectTimes = 0;
                pdControl_g.cc2_ConnectTimes++;
                if (pdControl_g.cc2_ConnectTimes >= 5) {
                    pdControl_g.cc2_ConnectTimes = 0;
                    pdControl_g.cc_State = CC_CONNECT;
                    USBPD->CONFIG |= CC_SEL;
                    pd_printf("Attach: CC2\n");
                }
                break;
            default:
                pdControl_g.cc1_ConnectTimes = 0;
                pdControl_g.cc2_ConnectTimes = 0;
                break;
        }
    }

    // 处理状态机
    usbpd_sink_process();
}

/**
 * @brief  设置 PDO Position
 * @param  position PDO Position (1-based)
 * @return 成功返回 true, 失败返回 false
 */
bool usbpd_sink_set_pdo_position(uint8_t position) {
    // if (position == 0 || position > pdControl_g.source_caps.pdo_count) {
    if (position == 0) {
        pd_printf("usbpd_sink_set_pdo_position: Invalid position\n");
        return false;
    }

    // 检查指定位置的 PDO 是否有效
    Source_PDO_t* pdo = NULL;
    for (uint8_t i = 0; i < pdControl_g.source_caps.pdo_count; i++) {
        if (pdControl_g.source_caps.pdos[i].position == position) {
            pdo = &pdControl_g.source_caps.pdos[i];
            break;
        }
    }
    if (pdo == NULL) {
        pd_printf("usbpd_sink_set_pdo_position: Invalid position\n");
        return false;
    }

    // 如果请求的是 EPR PDO, 但 source 或 cable 不支持 EPR, 则忽略请求
    if (pdo->fixed.voltage > 20000 && !(pdControl_g.cc_Source_EPR_Capable && pdControl_g.cc_Cable_EPR_Capable)) {
        pd_printf("usbpd_sink_set_pdo_position: EPR not supported\n");
        return false;
    }

    pdControl_g.cc_PDO_Pos = position;

    // 如果当前已经建立连接, 则发起新的请求
    if (pdControl_g.cc_USBPD_READY) {
        if (pdControl_g.cc_EPR_Ready) {
            // 在 EPR 模式下, 直接发送 EPR 请求
            pdControl_g.cc_State = CC_SEND_EPR_REQUEST;
        } else {
            // 在 SPR 模式下, 直接发送 SPR 请求
            pdControl_g.cc_State = CC_SEND_SPR_REQUEST;
        }
    }

    pd_printf("usbpd_sink_set_pdo_position: position=%d\n", position);
    return true;
}

/**
 * @brief  获取当前 PDO Position
 * @return uint8_t PDO Position (1-based), 如果未连接或无效则返回 0
 */
uint8_t usbpd_sink_get_pdo_position(void) {
    if (!pdControl_g.cc_USBPD_READY) {
        return 0;
    }
    return pdControl_g.cc_PDO_Pos;
}
