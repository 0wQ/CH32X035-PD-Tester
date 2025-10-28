#include "usbpd_sink.h"
#include "usbpd_def.h"
#include "usbpd_rdo.h"

#include <stdint.h>
#include "ch32x035.h"
#include "utils_print.h"
#include "utils_delay.h"

/******************************************************************************
 * Static Variables
 *****************************************************************************/

static pd_control_t pd_control_g;
static uint8_t usbpd_rx_buffer[USBPD_DATA_MAX_LEN] __attribute__((aligned(4)));
static uint8_t usbpd_tx_buffer[USBPD_DATA_MAX_LEN] __attribute__((aligned(4)));

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
    // 设置 CC 以正常 VDD 电压驱动输出
    USBPD->PORT_CC1 &= ~CC_LVE;
    USBPD->PORT_CC2 &= ~CC_LVE;

    // 清除所有中断标志位
    USBPD->CONFIG |= PD_ALL_CLR;
    USBPD->CONFIG &= ~PD_ALL_CLR;

    // 中断使能
    USBPD->CONFIG |= IE_TX_END | IE_RX_ACT | IE_RX_RESET;
    // PD 引脚输入滤波使能？
    USBPD->CONFIG |= PD_FILT_ED;
    // DMA 使能
    USBPD->CONFIG |= PD_DMA_EN;

    // 设置为接收模式
    USBPD->BMC_CLK_CNT = UPD_TMR_RX_48M;     // BMC 接收采样时钟计数器
    USBPD->DMA = (uint32_t)usbpd_rx_buffer;  // DMA Buffer

    // 开始接收
    USBPD->CONTROL &= ~PD_TX_EN;  // PD 接收使能
    USBPD->CONTROL |= BMC_START;  // BMC 开始信号

    NVIC_EnableIRQ(USBPD_IRQn);
}

static void usbpd_sink_state_reset(void) {
    NVIC_DisableIRQ(USBPD_IRQn);

    // 重置就绪状态
    pd_control_g.is_ready = false;

    // 重置 PD 版本
    pd_control_g.pd_version = DEF_PD_REVISION_30;

    // 重置状态机
    pd_control_g.pd_state = PD_STATE_CHECK_CONNECT;
    pd_control_g.pd_last_state = PD_STATE_CHECK_CONNECT;

    // 重置 PDO
    pd_control_g.pdo_pos = 1;
    pd_control_g.pdo_voltage_mv = 0;
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

    // 重置 EPR 相关变量
    pd_control_g.is_epr_ready = false;
    pd_control_g.source_epr_capable = false;  // 默认 source 不支持 EPR，后续根据判断
    pd_control_g.cable_epr_capable = true;    // 默认 cable 支持 EPR

    // 重置定时器
    pd_control_g.epr_keepalive_timer = 0;
    pd_control_g.pps_periodic_timer = 0;
}

static void usbpd_sink_phy_send_data(const uint8_t *tx_buffer, uint8_t tx_length, uint8_t sop) {
    delay_us(90);  // 确保 GoodCRC 已发送

    // 设置 CC 以低电压驱动输出
    if ((USBPD->CONFIG & CC_SEL) == CC_SEL) {
        USBPD->PORT_CC2 |= CC_LVE;
    } else {
        USBPD->PORT_CC1 |= CC_LVE;
    }

    USBPD->BMC_CLK_CNT = UPD_TMR_TX_48M;
    USBPD->TX_SEL = sop;
    USBPD->DMA = (uint32_t)tx_buffer;
    USBPD->BMC_TX_SZ = tx_length;

    USBPD->STATUS |= IF_TX_END;
    USBPD->CONTROL |= PD_TX_EN;
    USBPD->CONTROL |= BMC_START;

    // 等待发送完成
    while ((USBPD->STATUS & IF_TX_END) == 0);
    USBPD->STATUS |= IF_TX_END;

    usbpd_sink_rx_mode();
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

    // 清除全部状态
    USBPD->STATUS = BUF_ERR | IF_RX_BIT | IF_RX_BYTE | IF_RX_ACT | IF_RX_RESET | IF_TX_END;

    // PD Sink
    USBPD->PORT_CC1 = CC_CMP_66 | CC_PD;
    USBPD->PORT_CC2 = CC_CMP_66 | CC_PD;

    usbpd_sink_rx_mode();

    timer_init();
}

/******************************************************************************
 * SPR Mode Function
 * EPR Mode Function
 *****************************************************************************/

// TODO: 构建发送请求分散到多个函数
// NOT_SUPPORTED
// REJECT
// VDM_ACK_DISCOVER_IDENTITY
// VDM_ACK_DISCOVER_SVIDS
// SEND_EPR_ENTER
// SEND_EPR_SRC_CAP_REQ_CHUNK

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

static bool usbpd_sink_request(uint8_t position) {
    // TODO: 此函数可能需要删除多余的电压范围验证

    // 查找对应位置的 PDO
    pd_pdo_t *pdo = NULL;
    for (uint8_t i = 0; i < pd_control_g.available_pdos.pdo_count; i++) {
        if (pd_control_g.available_pdos.pdo[i].position == position) {
            pdo = &pd_control_g.available_pdos.pdo[i];
            break;
        }
    }

    if (pdo == NULL) {
        pd_printf("usbpd_sink_request: PDO position %d not found\n", position);
        return false;
    }

    // 构建 RDO
    USBPD_RDO_t rdo = {0};

    switch (pdo->pdo_type) {
        case PDO_TYPE_FIXED_SUPPLY: {
            // 检查硬件限制电压
            if (pdo->fixed.voltage > CONFIG_HW_MAX_VOLTAGE) {
                pd_printf("usbpd_sink_request: voltage %dmV > %dmV\n", pdo->fixed.voltage, CONFIG_HW_MAX_VOLTAGE);
                return false;
            }
            usbpd_rdo_build_fixed(&rdo, position, pdo->fixed.voltage, pdo->fixed.current);
            break;
        }

        case PDO_TYPE_APDO: {
            if (pdo->apdo_subtype == APDO_TYPE_SPR_PPS) {
                // 使用指定电压或最小电压
                uint16_t voltage = (pd_control_g.pdo_voltage_mv > 0) ? pd_control_g.pdo_voltage_mv : pdo->pps.min_voltage;
                // 检查档位电压范围
                if (voltage < pdo->pps.min_voltage || voltage > pdo->pps.max_voltage) {
                    pd_printf("usbpd_sink_request: voltage %dmV not in [%dmV, %dmV]\n", voltage, pdo->pps.min_voltage, pdo->pps.max_voltage);
                    return false;
                }
                // 检查硬件限制电压
                if (voltage > CONFIG_HW_MAX_VOLTAGE) {
                    pd_printf("usbpd_sink_request: voltage %dmV > %dmV\n", voltage, CONFIG_HW_MAX_VOLTAGE);
                    return false;
                }
                usbpd_rdo_build_pps(&rdo, position, voltage, pdo->pps.current);

            } else if (pdo->apdo_subtype == APDO_TYPE_EPR_AVS) {
                // 使用指定电压或最小电压
                uint16_t voltage = (pd_control_g.pdo_voltage_mv > 0) ? pd_control_g.pdo_voltage_mv : pdo->epr_avs.min_voltage;
                // 检查档位电压范围
                if (voltage < pdo->epr_avs.min_voltage || voltage > pdo->epr_avs.max_voltage) {
                    pd_printf("usbpd_sink_request: voltage %dmV not in [%dmV, %dmV]\n", voltage, pdo->epr_avs.min_voltage, pdo->epr_avs.max_voltage);
                    return false;
                }
                // 检查硬件限制电压
                if (voltage > CONFIG_HW_MAX_VOLTAGE) {
                    pd_printf("usbpd_sink_request: voltage %dmV > %dmV\n", voltage, CONFIG_HW_MAX_VOLTAGE);
                    return false;
                }
                // 根据 PDP 和电压计算电流
                uint16_t current = ((uint32_t)pdo->epr_avs.pdp * 1000000U) / voltage;
                current = current > 5000 ? 5000 : current;
                usbpd_rdo_build_avs(&rdo, position, voltage, current);

            } else if (pdo->apdo_subtype == APDO_TYPE_SPR_AVS) {
                // 使用指定电压或最小电压
                uint16_t voltage = (pd_control_g.pdo_voltage_mv > 0) ? pd_control_g.pdo_voltage_mv : pdo->spr_avs.min_voltage;
                // 检查档位电压范围
                if (voltage < pdo->spr_avs.min_voltage || voltage > pdo->spr_avs.max_voltage) {
                    pd_printf("usbpd_sink_request: voltage %dmV not in [%dmV, %dmV]\n", voltage, pdo->spr_avs.min_voltage, pdo->spr_avs.max_voltage);
                    return false;
                }
                // 检查硬件限制电压
                if (voltage > CONFIG_HW_MAX_VOLTAGE) {
                    pd_printf("usbpd_sink_request: voltage %dmV > %dmV\n", voltage, CONFIG_HW_MAX_VOLTAGE);
                    return false;
                }
                // 判断最大电流
                uint16_t current = voltage > 15000 ? pdo->spr_avs.max_current_15v_20v : pdo->spr_avs.max_current_9v_15v;
                usbpd_rdo_build_avs(&rdo, position, voltage, current);

            } else {
                pd_printf("usbpd_sink_request: Unsupported APDO subtype %d\n", pdo->apdo_subtype);
                return false;
            }
            break;
        }

        default: {
            pd_printf("usbpd_sink_request: Unsupported PDO type %d\n", pdo->pdo_type);
            return false;
        }
    }

    // 格式化请求
    uint8_t length;
    if (pd_control_g.is_epr_ready) {
        length = usbpd_rdo_format_epr_request(usbpd_tx_buffer, &rdo, pdo->raw, pd_control_g.sink_message_id);
        pd_printf("Sending EPR request: Pos=%d, RDO=0x%08x, PDO=0x%08x\n", position, rdo.d32, pdo->raw);
    } else {
        length = usbpd_rdo_format_spr_request(usbpd_tx_buffer, &rdo, pd_control_g.sink_message_id, pd_control_g.pd_version);
        pd_printf("Sending SPR request: Pos=%d, RDO=0x%08x\n", position, rdo.d32);
    }

    // 发送请求
    usbpd_sink_phy_send_data(usbpd_tx_buffer, length, UPD_SOP0);
    return true;
}

bool usbpd_sink_set_fpdo_position(uint8_t position) {
    // 查找对应位置的 PDO，验证是否为 Fixed Supply
    pd_pdo_t *pdo = NULL;
    for (uint8_t i = 0; i < pd_control_g.available_pdos.pdo_count; i++) {
        if (pd_control_g.available_pdos.pdo[i].position == position) {
            pdo = &pd_control_g.available_pdos.pdo[i];
            break;
        }
    }

    if (pdo == NULL) {
        pd_printf("usbpd_sink_set_fpdo_position(%d): position not found\n", position);
        return false;
    }

    if (pdo->pdo_type != PDO_TYPE_FIXED_SUPPLY) {
        pd_printf("usbpd_sink_set_fpdo_position(%d): not a Fixed Supply PDO\n", position);
        return false;
    }

    if (!pd_control_g.is_ready) {
        pd_printf("usbpd_sink_set_fpdo_position(%d): not ready\n", position);
        return false;
    }

    // 检查硬件限制电压
    if (pdo->fixed.voltage > CONFIG_HW_MAX_VOLTAGE) {
        pd_printf("usbpd_sink_set_fpdo_position(%d): voltage %dmV > %dmV\n", position, pdo->fixed.voltage, CONFIG_HW_MAX_VOLTAGE);
        return false;
    }

    // 等待状态机进入空闲状态，超时后返回
    uint32_t start_time = millis();
    while (pd_control_g.pd_state != PD_STATE_IDLE) {
        if (millis() - start_time > 1000) {
            pd_printf("usbpd_sink_set_fpdo_position(%d): wait for idle timeout\n", position);
            return false;
        }
    }

    // 设置 PDO Position 和电压为 0（表示使用 PDO 默认值）
    pd_control_g.pdo_pos = position;
    pd_control_g.pdo_voltage_mv = 0;

    // 判断 SPR/EPR，设置状态机进入对应的 SEND_REQUEST 状态
    pd_control_g.pd_state = pd_control_g.is_epr_ready ? PD_STATE_SEND_EPR_REQUEST : PD_STATE_SEND_SPR_REQUEST;

    pd_printf("usbpd_sink_set_fpdo_position(%d): success\n", position);
    return true;
}

bool usbpd_sink_set_apdo_position_with_voltage(uint8_t position, uint16_t voltage_mv) {
    // 查找对应位置的 PDO
    pd_pdo_t *pdo = NULL;
    for (uint8_t i = 0; i < pd_control_g.available_pdos.pdo_count; i++) {
        if (pd_control_g.available_pdos.pdo[i].position == position) {
            pdo = &pd_control_g.available_pdos.pdo[i];
            break;
        }
    }

    if (pdo == NULL) {
        pd_printf("usbpd_sink_set_apdo_position_with_voltage(%d, %d): position not found\n", position, voltage_mv);
        return false;
    }

    if (pdo->pdo_type != PDO_TYPE_APDO) {
        pd_printf("usbpd_sink_set_apdo_position_with_voltage(%d, %d): not an APDO\n", position, voltage_mv);
        return false;
    }

    // 检查硬件限制电压
    if (pdo->fixed.voltage > CONFIG_HW_MAX_VOLTAGE) {
        pd_printf("usbpd_sink_set_apdo_position_with_voltage(%d, %d): voltage %dmV > %dmV\n", position, voltage_mv, voltage_mv, CONFIG_HW_MAX_VOLTAGE);
        return false;
    }

    // 验证电压范围和步进
    uint16_t min_voltage = 0;
    uint16_t max_voltage = 0;
    uint16_t step_mv = 0;

    if (pdo->apdo_subtype == APDO_TYPE_SPR_PPS) {
        min_voltage = pdo->pps.min_voltage;
        max_voltage = pdo->pps.max_voltage;
        step_mv = 20;  // PPS 步进 20mV
    } else if (pdo->apdo_subtype == APDO_TYPE_EPR_AVS) {
        min_voltage = pdo->epr_avs.min_voltage;
        max_voltage = pdo->epr_avs.max_voltage;
        step_mv = 100;  // EPR AVS 步进 100mV
    } else if (pdo->apdo_subtype == APDO_TYPE_SPR_AVS) {
        min_voltage = pdo->spr_avs.min_voltage;
        max_voltage = pdo->spr_avs.max_voltage;
        step_mv = 100;  // SPR AVS 步进 100mV
    } else {
        pd_printf("usbpd_sink_set_apdo_position_with_voltage(%d, %d): unsupported APDO subtype\n", position, voltage_mv);
        return false;
    }

    // 检查电压范围
    if (voltage_mv < min_voltage || voltage_mv > max_voltage) {
        pd_printf("usbpd_sink_set_apdo_position_with_voltage(%d, %d): voltage out of range [%d-%d]\n", position, voltage_mv, min_voltage, max_voltage);
        return false;
    }

    // 检查步进对齐
    if ((voltage_mv % step_mv) != 0) {
        pd_printf("usbpd_sink_set_apdo_position_with_voltage(%d, %d): voltage not aligned to %dmV step\n", position, voltage_mv, step_mv);
        return false;
    }

    if (!pd_control_g.is_ready) {
        pd_printf("usbpd_sink_set_apdo_position_with_voltage(%d, %d): not ready\n", position, voltage_mv);
        return false;
    }

    // 等待状态机进入空闲状态，超时后返回
    uint32_t start_time = millis();
    while (pd_control_g.pd_state != PD_STATE_IDLE) {
        if (millis() - start_time > 1000) {
            pd_printf("usbpd_sink_set_apdo_position_with_voltage(%d, %d): wait for idle timeout\n", position, voltage_mv);
            return false;
        }
    }

    // 设置 PDO Position 和电压
    pd_control_g.pdo_pos = position;
    pd_control_g.pdo_voltage_mv = voltage_mv;

    // 判断 SPR/EPR，设置状态机进入对应的 SEND_REQUEST 状态
    pd_control_g.pd_state = pd_control_g.is_epr_ready ? PD_STATE_SEND_EPR_REQUEST : PD_STATE_SEND_SPR_REQUEST;

    pd_printf("usbpd_sink_set_apdo_position_with_voltage(%d, %d): success\n", position, voltage_mv);
    return true;
}

bool usbpd_sink_set_position(uint8_t position) {
    // 查找对应位置的 PDO，验证是否为 Fixed Supply
    pd_pdo_t *pdo = NULL;
    for (uint8_t i = 0; i < pd_control_g.available_pdos.pdo_count; i++) {
        if (pd_control_g.available_pdos.pdo[i].position == position) {
            pdo = &pd_control_g.available_pdos.pdo[i];
            break;
        }
    }

    if (pdo == NULL) {
        pd_printf("usbpd_sink_set_position(%d): position not found\n", position);
        return false;
    }

    // if (pdo->pdo_type == PDO_TYPE_FIXED_SUPPLY) {
    //     usbpd_sink_set_fpdo_position(position);
    // }
    // if (pdo->pdo_type == PDO_TYPE_APDO) {
    //     usbpd_sink_set_apdo_position_with_voltage(position, 0);
    // }

    return true;
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
static void usbpd_sink_pdos_analyse(const uint32_t *pdos, uint8_t pdo_count, bool is_epr) {
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
                        uint16_t max_current_9v_15v = POWER_DECODE_10MA(pdo_parser.SPR_AVS.MaxCurrentFor9V15VIn10mAunits);
                        uint16_t max_current_15v_20v = POWER_DECODE_10MA(pdo_parser.SPR_AVS.MaxCurrentFor15V20VIn10mAunits);

                        pd_control_g.available_pdos.pdo_count++;
                        pdo->pdo_type = PDO_TYPE_APDO;
                        pdo->apdo_subtype = APDO_TYPE_SPR_AVS;
                        pdo->spr_avs.max_current_9v_15v = max_current_9v_15v;
                        pdo->spr_avs.max_current_15v_20v = max_current_15v_20v;
                        pdo->spr_avs.min_voltage = max_current_9v_15v > 0 ? 9000 : 15000;
                        pdo->spr_avs.max_voltage = max_current_15v_20v > 0 ? 20000 : 15000;

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
            }
            break;
        }
        case PD_STATE_RECEIVED_SPR_SOURCE_CAP: {
            usbpd_sink_pdos_analyse(pd_control_g.spr_source_cap_buffer, pd_control_g.spr_source_cap_buffer_pdo_count, false);
            pd_control_g.pd_state = PD_STATE_SEND_SPR_REQUEST;
            break;
        }
        case PD_STATE_SEND_SPR_REQUEST:
        case PD_STATE_SEND_EPR_REQUEST: {
            bool req_ok = usbpd_sink_request(pd_control_g.pdo_pos);
            pd_control_g.pd_state = req_ok ? PD_STATE_WAIT_ACCEPT : PD_STATE_IDLE;
            break;
        }
        case PD_STATE_RECEIVED_PS_RDY: {
            pd_control_g.is_ready = true;

            // MIPPS
            if (!pd_control_g.mipps_is_drswap_requested) {
                pd_control_g.mipps_is_drswap_requested = true;
                pd_control_g.pd_state = MIPPS_STATE_SEND_DRSWAP;
                break;
            }

#ifdef CONFIG_EPR_MODE_ENABLE
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
        case MIPPS_STATE_SEND_DRSWAP: {
            USBPD_MessageHeader_t header = {0};
            header.MessageHeader.MessageID = pd_control_g.sink_message_id;
            header.MessageHeader.MessageType = USBPD_CONTROL_MSG_DR_SWAP;
            header.MessageHeader.SpecificationRevision = pd_control_g.pd_version;

            *(uint16_t *)&usbpd_tx_buffer[0] = header.d16;

            usbpd_sink_phy_send_data(usbpd_tx_buffer, 2, UPD_SOP0);
            pd_control_g.pd_state = MIPPS_STATE_WAIT_DRSWAP_RESPONSE;
            break;
        }
        case MIPPS_STATE_SEND_VDM_REQ_DISCOVER_IDENTITY: {
            USBPD_MessageHeader_t header = {0};
            header.MessageHeader.MessageID = pd_control_g.sink_message_id;
            header.MessageHeader.MessageType = USBPD_DATA_MSG_VENDOR_DEFINED;
            header.MessageHeader.SpecificationRevision = pd_control_g.pd_version;
            header.MessageHeader.NumberOfDataObjects = 1;

            USBPD_StructuredVDMHeader_t vdm_header = {0};
            vdm_header.StructuredVDMHeader.Command = 1;
            vdm_header.StructuredVDMHeader.CommandType = 0;
            vdm_header.StructuredVDMHeader.ObjectPosition = 0;
            vdm_header.StructuredVDMHeader.StructuredVDMVersionMinor = 0;
            vdm_header.StructuredVDMHeader.StructuredVDMVersionMajor = 0;
            vdm_header.StructuredVDMHeader.VDMType = 1;
            vdm_header.StructuredVDMHeader.SVID = 0xFF00;

            *(uint16_t *)&usbpd_tx_buffer[0] = header.d16;
            *(uint32_t *)&usbpd_tx_buffer[2] = vdm_header.d32;

            usbpd_sink_phy_send_data(usbpd_tx_buffer, (2 + 4), UPD_SOP0);
            pd_control_g.pd_state = MIPPS_STATE_WAIT_VDM_ACK_DISCOVER_IDENTITY;
            break;
        }
        case MIPPS_STATE_SEND_VDM_REQ_DISCOVER_SVIDS: {
            USBPD_MessageHeader_t header = {0};
            header.MessageHeader.MessageID = pd_control_g.sink_message_id;
            header.MessageHeader.MessageType = USBPD_DATA_MSG_VENDOR_DEFINED;
            header.MessageHeader.SpecificationRevision = pd_control_g.pd_version;
            header.MessageHeader.NumberOfDataObjects = 1;

            USBPD_StructuredVDMHeader_t vdm_header = {0};
            vdm_header.StructuredVDMHeader.Command = 2;
            vdm_header.StructuredVDMHeader.CommandType = 0;
            vdm_header.StructuredVDMHeader.ObjectPosition = 0;
            vdm_header.StructuredVDMHeader.StructuredVDMVersionMinor = 0;
            vdm_header.StructuredVDMHeader.StructuredVDMVersionMajor = 0;
            vdm_header.StructuredVDMHeader.VDMType = 1;
            vdm_header.StructuredVDMHeader.SVID = 0xFF00;

            *(uint16_t *)&usbpd_tx_buffer[0] = header.d16;
            *(uint32_t *)&usbpd_tx_buffer[2] = vdm_header.d32;

            usbpd_sink_phy_send_data(usbpd_tx_buffer, (2 + 4), UPD_SOP0);
            pd_control_g.pd_state = MIPPS_STATE_WAIT_VDM_ACK_DISCOVER_SVIDS;
            break;
        }
        case MIPPS_STATE_SEND_VDM_1: {
            USBPD_MessageHeader_t header = {0};
            header.MessageHeader.MessageID = pd_control_g.sink_message_id;
            header.MessageHeader.MessageType = USBPD_DATA_MSG_VENDOR_DEFINED;
            header.MessageHeader.SpecificationRevision = pd_control_g.pd_version;
            header.MessageHeader.NumberOfDataObjects = 1;

            *(uint16_t *)&usbpd_tx_buffer[0] = header.d16;
            *(uint32_t *)&usbpd_tx_buffer[2] = 0x27170101;

            usbpd_sink_phy_send_data(usbpd_tx_buffer, (2 + 4), UPD_SOP0);
            pd_control_g.pd_state = MIPPS_STATE_WAIT_VDM_1;
            break;
        }
        case MIPPS_STATE_SEND_VDM_2: {
            USBPD_MessageHeader_t header = {0};
            header.MessageHeader.MessageID = pd_control_g.sink_message_id;
            header.MessageHeader.MessageType = USBPD_DATA_MSG_VENDOR_DEFINED;
            header.MessageHeader.SpecificationRevision = pd_control_g.pd_version;
            header.MessageHeader.NumberOfDataObjects = 1;

            *(uint16_t *)&usbpd_tx_buffer[0] = header.d16;
            *(uint32_t *)&usbpd_tx_buffer[2] = 0x27170102;

            usbpd_sink_phy_send_data(usbpd_tx_buffer, (2 + 4), UPD_SOP0);
            pd_control_g.pd_state = MIPPS_STATE_WAIT_VDM_2;
            break;
        }
        case MIPPS_STATE_SEND_VDM_3: {
            USBPD_MessageHeader_t header = {0};
            header.MessageHeader.MessageID = pd_control_g.sink_message_id;
            header.MessageHeader.MessageType = USBPD_DATA_MSG_VENDOR_DEFINED;
            header.MessageHeader.SpecificationRevision = pd_control_g.pd_version;
            header.MessageHeader.NumberOfDataObjects = 1;

            *(uint16_t *)&usbpd_tx_buffer[0] = header.d16;
            *(uint32_t *)&usbpd_tx_buffer[2] = 0x27170103;

            usbpd_sink_phy_send_data(usbpd_tx_buffer, (2 + 4), UPD_SOP0);
            pd_control_g.pd_state = MIPPS_STATE_WAIT_VDM_3;
            break;
        }
        case MIPPS_STATE_SEND_VDM_4: {
            USBPD_MessageHeader_t header = {0};
            header.MessageHeader.MessageID = pd_control_g.sink_message_id;
            header.MessageHeader.MessageType = USBPD_DATA_MSG_VENDOR_DEFINED;
            header.MessageHeader.SpecificationRevision = pd_control_g.pd_version;
            header.MessageHeader.NumberOfDataObjects = 5;

            *(uint16_t *)&usbpd_tx_buffer[0] = header.d16;
            *(uint32_t *)&usbpd_tx_buffer[2] = 0x27170104;
            *(uint32_t *)&usbpd_tx_buffer[6] = 0xD6E97705;
            *(uint32_t *)&usbpd_tx_buffer[10] = 0x06C7F5F4;
            *(uint32_t *)&usbpd_tx_buffer[14] = 0xDF7A4000;
            *(uint32_t *)&usbpd_tx_buffer[18] = 0x8C7467B8;

            usbpd_sink_phy_send_data(usbpd_tx_buffer, (2 + 4 * 5), UPD_SOP0);
            pd_control_g.pd_state = MIPPS_STATE_WAIT_VDM_4;
            break;
        }
        case MIPPS_STATE_SEND_VDM_5: {
            USBPD_MessageHeader_t header = {0};
            header.MessageHeader.MessageID = pd_control_g.sink_message_id;
            header.MessageHeader.MessageType = USBPD_DATA_MSG_VENDOR_DEFINED;
            header.MessageHeader.SpecificationRevision = pd_control_g.pd_version;
            header.MessageHeader.NumberOfDataObjects = 5;

            *(uint16_t *)&usbpd_tx_buffer[0] = header.d16;
            *(uint32_t *)&usbpd_tx_buffer[2] = 0x27170105;
            *(uint32_t *)&usbpd_tx_buffer[6] = 0x464E9E0C;
            *(uint32_t *)&usbpd_tx_buffer[10] = 0xEE13382B;
            *(uint32_t *)&usbpd_tx_buffer[14] = 0x4124E2DA;
            *(uint32_t *)&usbpd_tx_buffer[18] = 0x43DDF86F;

            usbpd_sink_phy_send_data(usbpd_tx_buffer, (2 + 4 * 5), UPD_SOP0);
            pd_control_g.pd_state = MIPPS_STATE_WAIT_VDM_5;
            break;
        }
        case MIPPS_STATE_SEND_VDM_6: {
            USBPD_MessageHeader_t header = {0};
            header.MessageHeader.MessageID = pd_control_g.sink_message_id;
            header.MessageHeader.MessageType = USBPD_DATA_MSG_VENDOR_DEFINED;
            header.MessageHeader.SpecificationRevision = pd_control_g.pd_version;
            header.MessageHeader.NumberOfDataObjects = 2;

            *(uint16_t *)&usbpd_tx_buffer[0] = header.d16;
            *(uint32_t *)&usbpd_tx_buffer[2] = 0x27170106;
            *(uint32_t *)&usbpd_tx_buffer[6] = 0x00000001;

            usbpd_sink_phy_send_data(usbpd_tx_buffer, (2 + 4 * 2), UPD_SOP0);
            pd_control_g.pd_state = MIPPS_STATE_WAIT_VDM_6;
            break;
        }
        case MIPPS_STATE_SEND_VDM_7: {
            USBPD_MessageHeader_t header = {0};
            header.MessageHeader.MessageID = pd_control_g.sink_message_id;
            header.MessageHeader.MessageType = USBPD_DATA_MSG_VENDOR_DEFINED;
            header.MessageHeader.SpecificationRevision = pd_control_g.pd_version;
            header.MessageHeader.NumberOfDataObjects = 2;

            *(uint16_t *)&usbpd_tx_buffer[0] = header.d16;
            *(uint32_t *)&usbpd_tx_buffer[2] = 0x27170107;
            *(uint32_t *)&usbpd_tx_buffer[6] = 0x00000001;

            usbpd_sink_phy_send_data(usbpd_tx_buffer, (2 + 4 * 2), UPD_SOP0);
            pd_control_g.pd_state = MIPPS_STATE_WAIT_VDM_7;
            break;
        }
        case MIPPS_STATE_SEND_GET_SRC_CAP: {
            USBPD_MessageHeader_t header = {0};
            header.MessageHeader.MessageID = pd_control_g.sink_message_id;
            header.MessageHeader.MessageType = USBPD_CONTROL_MSG_GET_SRC_CAP;
            header.MessageHeader.SpecificationRevision = pd_control_g.pd_version;

            *(uint16_t *)&usbpd_tx_buffer[0] = header.d16;

            usbpd_sink_phy_send_data(usbpd_tx_buffer, 2, UPD_SOP0);
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
static void usbpd_sink_protocol_analysis_sop0(const uint8_t *rx_buffer, uint8_t rx_length) {
    // Header
    USBPD_MessageHeader_t header = {0};
    header.d16 = *(uint16_t *)rx_buffer;

    // 如果当前收到消息非 GoodCRC，则需先回复 GoodCRC
    bool is_goodcrc_msg = (header.MessageHeader.Extended == 0) &&
                          (header.MessageHeader.NumberOfDataObjects == 0) &&
                          (header.MessageHeader.MessageType == USBPD_CONTROL_MSG_GOODCRC);
    if (!is_goodcrc_msg) {
        usbpd_sink_send_goodcrc(header.MessageHeader.MessageID, false);
    }

    // Non-Extended message
    if (header.MessageHeader.Extended == 0) {

        // Control Message (A Message is defined as a Control Message when the Number of Data Objects field in the Message Header is set to zero)
        if (header.MessageHeader.NumberOfDataObjects == 0) {
            switch (header.MessageHeader.MessageType) {
                case USBPD_CONTROL_MSG_GOODCRC: {
                    // 消息 ID 自增
                    pd_control_g.sink_message_id = (pd_control_g.sink_message_id + 1) & 0x07;
                    // 重置定时器
                    pd_control_g.epr_keepalive_timer = 0;
                    pd_control_g.pps_periodic_timer = 0;
                    break;
                }
                case USBPD_CONTROL_MSG_ACCEPT: {
                    // MIPPS
                    if (pd_control_g.pd_state == MIPPS_STATE_WAIT_DRSWAP_RESPONSE) {
                        pd_control_g.pd_state = MIPPS_STATE_SEND_VDM_REQ_DISCOVER_IDENTITY;
                        break;
                    }

                    // TODO: 这里要判断上一状态才能 PD_STATE_WAIT_PS_RDY
                    pd_control_g.pd_state = PD_STATE_WAIT_PS_RDY;
                    break;
                }
                case USBPD_CONTROL_MSG_PS_RDY: {
                    pd_control_g.pd_state = PD_STATE_RECEIVED_PS_RDY;
                    break;
                }
                case USBPD_CONTROL_MSG_SOFT_RESET: {
                    usbpd_sink_state_reset();
                    break;
                }
                case USBPD_CONTROL_MSG_NOT_SUPPORTED: {
                    // 在发送 EPR MODE Enter 后，如果收到 NOT_SUPPORTED 回复，则认为不支持 EPR
                    if (pd_control_g.pd_state == PD_STATE_WAIT_EPR_ENTER_RESPONSE) {
                        pd_control_g.cable_epr_capable = 0;
                        pd_control_g.source_epr_capable = 0;
                        pd_control_g.pd_state = PD_STATE_IDLE;
                        break;
                    }

                    pd_control_g.pd_state = PD_STATE_IDLE;
                    break;
                }
                case USBPD_CONTROL_MSG_GET_SNK_CAP:           // 待实现
                case USBPD_CONTROL_MSG_GET_SNK_CAP_EXTENDED:  // 待实现
                case USBPD_CONTROL_MSG_VCONN_SWAP:            // 待实现
                default: {
                    // pd_printf("USBPD_ControlMessageType_t unhandled message type: %d\n", messageHeader->MessageHeader.MessageType);
                    pd_control_g.pd_state = PD_STATE_SEND_NOT_SUPPORTED;
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

                    if (vdm_header.StructuredVDMHeader.VDMType == 1) {  // 1 = Structured VDM
                        // MIPPS
                        if (vdm_header.StructuredVDMHeader.Command == 1 && vdm_header.StructuredVDMHeader.CommandType == 1) {  // ACK DISCOVER_IDENTITY
                            // 判断 USB Vendor ID
                            USBPD_IDHeaderVDO_t id_header_vdo = {0};
                            id_header_vdo.d32 = *(uint32_t *)&rx_buffer[6];
                            pd_printf("  ID Header VDO: 0x%08x\n", id_header_vdo.d32);
                            pd_printf("  Vendor ID: 0x%04x\n", id_header_vdo.IDHeaderVDO.USBVendorID);
                            if (id_header_vdo.IDHeaderVDO.USBVendorID == 0x2B01 ||  // ZMI
                                id_header_vdo.IDHeaderVDO.USBVendorID == 0x2717) {  // XIAOMI
                                pd_control_g.pd_state = MIPPS_STATE_SEND_VDM_REQ_DISCOVER_SVIDS;
                                break;
                            }
                            // 其他 USB Vendor ID，不支持 MIPPS
                            pd_control_g.pd_state = PD_STATE_IDLE;
                            break;
                        }
                        if (vdm_header.StructuredVDMHeader.Command == 2 && vdm_header.StructuredVDMHeader.CommandType == 1) {  // ACK DISCOVER_SVIDS
                            // TODO: 判断 SVID
                            pd_control_g.pd_state = MIPPS_STATE_SEND_VDM_1;
                            break;
                        }
                    } else {
                        // MIPPS
                        if (pd_control_g.pd_state == MIPPS_STATE_WAIT_VDM_1) {
                            pd_control_g.pd_state = MIPPS_STATE_SEND_VDM_2;
                            break;
                        }
                        if (pd_control_g.pd_state == MIPPS_STATE_WAIT_VDM_2) {
                            pd_control_g.pd_state = MIPPS_STATE_SEND_VDM_3;
                            break;
                        }
                        if (pd_control_g.pd_state == MIPPS_STATE_WAIT_VDM_3) {
                            pd_control_g.pd_state = MIPPS_STATE_SEND_VDM_4;
                            break;
                        }
                        if (pd_control_g.pd_state == MIPPS_STATE_WAIT_VDM_4) {
                            pd_control_g.pd_state = MIPPS_STATE_SEND_VDM_5;
                            break;
                        }
                        if (pd_control_g.pd_state == MIPPS_STATE_WAIT_VDM_5) {
                            pd_control_g.pd_state = MIPPS_STATE_SEND_VDM_6;
                            break;
                        }
                        if (pd_control_g.pd_state == MIPPS_STATE_WAIT_VDM_6) {
                            pd_control_g.pd_state = MIPPS_STATE_SEND_VDM_7;
                            break;
                        }
                        if (pd_control_g.pd_state == MIPPS_STATE_WAIT_VDM_7) {
                            pd_control_g.pd_state = MIPPS_STATE_SEND_GET_SRC_CAP;
                            break;
                        }
                    }

                    pd_control_g.pd_state = PD_STATE_SEND_NOT_SUPPORTED;
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
                    // 当前分块号
                    uint8_t current_chunk_number = ext_header.ExtendedMessageHeader.ChunkNumber;
                    // 当前分块大小
                    uint8_t chunk_size = rx_length - (2 /*Header*/ + 2 /*ExtHeader*/ + 4 /*CRC32*/);

                    // 收到分块号 0 时重置状态
                    if (current_chunk_number == 0) {
                        pd_control_g.epr_source_cap_buffer_size = 0;
                        pd_control_g.epr_source_cap_buffer_pdo_count = 0;
                        pd_control_g.epr_source_cap_buffer_chunk_number = 0;
                    }

                    // 保存
                    memcpy((uint8_t *)pd_control_g.epr_source_cap_buffer + pd_control_g.epr_source_cap_buffer_size, &rx_buffer[(2 /*Header*/ + 2 /*ExtHeader*/)], chunk_size);
                    // 更新 buffer size
                    pd_control_g.epr_source_cap_buffer_size += chunk_size;

                    if (chunk_size < 26) {
                        // 全部接收完成，在状态机中解析 EPR_SOURCE_CAP
                        pd_control_g.pd_state = PD_STATE_RECEIVED_EPR_SOURCE_CAP;
                        // 重置当前分块号
                        pd_control_g.epr_source_cap_buffer_chunk_number = 0;
                        // 更新 pdo count
                        pd_control_g.epr_source_cap_buffer_pdo_count = pd_control_g.epr_source_cap_buffer_size / 4;
                    } else {
                        // 请求下个分块
                        pd_control_g.pd_state = PD_STATE_SEND_EPR_SRC_CAP_REQ_CHUNK;
                        // 更新 chunk number
                        pd_control_g.epr_source_cap_buffer_chunk_number = current_chunk_number;
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
static void usbpd_sink_protocol_analysis_sop1(const uint8_t *rx_buffer, uint8_t rx_length) {
#ifdef CONFIG_EMARKER_ENABLE
    // Header
    USBPD_MessageHeader_t header = {0};
    header.d16 = *(uint16_t *)rx_buffer;

    // 如果当前收到消息非 GoodCRC，则需先回复 GoodCRC
    bool is_goodcrc_msg = (header.MessageHeader.Extended == 0) &&
                          (header.MessageHeader.NumberOfDataObjects == 0) &&
                          (header.MessageHeader.MessageType == USBPD_CONTROL_MSG_GOODCRC);
    if (!is_goodcrc_msg) {
        usbpd_sink_send_goodcrc(header.MessageHeader.MessageID, false);
    }

    // Non-Extended message
    if (header.MessageHeader.Extended == 0) {

        // Control Message (A Message is defined as a Control Message when the Number of Data Objects field in the Message Header is set to zero)
        if (header.MessageHeader.NumberOfDataObjects == 0) {
            switch (header.MessageHeader.MessageType) {
                case USBPD_CONTROL_MSG_GOODCRC: {
                    // 消息 ID 自增
                    pd_control_g.cable_message_id = (pd_control_g.cable_message_id + 1) & 0x07;
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
                                // *(uint32_t *)&usbpd_tx_buffer[18] = 0x110A2643;  // Product Type VDO(s)

                                USBPD_PassiveCableVDO_t vdo = {0};
                                vdo.PassiveCableVDO.USBHighestSpeed = 0b100;      // 000b = [USB 2.0] only, no SuperSpeed support
                                                                                  // 001b = [USB 3.2] Gen1
                                                                                  // 010b = [USB 3.2]/[USB4] Gen2
                                                                                  // 011b = [USB4] Gen3
                                                                                  // 100b = [USB4] Gen4
                                vdo.PassiveCableVDO.VBUSCurrentHandling = 0b10;   // 01b = 3A
                                                                                  // 10b = 5A
                                vdo.PassiveCableVDO.MaxVBUSVoltage = 0b11;        // 00b – 20V
                                                                                  // 01b – 30V (Deprecated)
                                                                                  // 10b – 40V (Deprecated)
                                                                                  // 11b – 50V
                                vdo.PassiveCableVDO.CableTerminationType = 0b00;  // 00b = VCONN not required. Cable Plugs that only support Discover Identity Commands Shall set these bits to 00b.
                                                                                  // 01b = VCONN required
                                vdo.PassiveCableVDO.CableLatency = 0b1000;        // 0001b – <10ns (~1m)
                                                                                  // 0010b – 10ns to 20ns (~2m)
                                                                                  // 0011b – 20ns to 30ns (~3m)
                                                                                  // 0100b – 30ns to 40ns (~4m)
                                                                                  // 0101b – 40ns to 50ns (~5m)
                                                                                  // 0110b – 50ns to 60ns (~6m)
                                                                                  // 0111b – 60ns to 70ns (~7m)
                                                                                  // 1000b – > 70ns (>~7m)
                                vdo.PassiveCableVDO.EPRCapable = 0b1;             // 0b – Cable is not EPR Capable
                                                                                  // 1b = Cable is EPR Capable
                                vdo.PassiveCableVDO.USBTypeCPlug = 0b10;          // 10b = USB Type-C
                                                                                  // 11b = Captive
                                vdo.PassiveCableVDO.VDOVersion = 0b000;           // Version 1.0 = 000b
                                vdo.PassiveCableVDO.FirmwareVersion = 1;
                                vdo.PassiveCableVDO.HWVersion = 1;

                                *(uint32_t *)&usbpd_tx_buffer[18] = vdo.d32;

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

        // 在某些状态需要跳过 keep alive，在错误的时机发送可能 reset
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