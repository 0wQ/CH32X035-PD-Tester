#pragma once

#include <stdbool.h>
#include <string.h>
#include "usbpd_def.h"

// USB_PD_R3_2 V1.1 2024-10.pdf
// Page 260: SinkEPRKeepAlive Timer
// tSinkEPRKeepAlive   min:0.250s nom:0.375s max:0.500s
#define EPR_KEEP_ALIVE_TIMEOUT 40  // 400ms 单位：10ms
// #define EPR_KEEP_ALIVE_TIMEOUT 1  // 速度测试

// 用于存储可用的 PDO 以供后续请求使用，目前只支持 Fixed Supply PDO
typedef struct {
    PDO_Type_t type;   // PDO 类型
    uint8_t position;  // PDO Position
    uint32_t raw;      // PDO 原始数据
    union {
        // Fixed Supply PDO
        struct {
            uint16_t voltage;  // 电压 (mV)
            uint16_t current;  // 电流 (mA)
            bool epr_capable;  // 是否支持 EPR
        } fixed;

        // Battery PDO
        struct {
            uint16_t min_voltage;  // 最小电压 (mV)
            uint16_t max_voltage;  // 最大电压 (mV)
            uint16_t power;        // 功率 (mW)
        } battery;

        // Variable PDO
        struct {
            uint16_t min_voltage;  // 最小电压 (mV)
            uint16_t max_voltage;  // 最大电压 (mV)
            uint16_t current;      // 电流 (mA)
        } variable;

        // SPR PPS APDO
        struct {
            uint16_t min_voltage;  // 最小电压 (mV)
            uint16_t max_voltage;  // 最大电压 (mV)
            uint16_t current;      // 电流 (mA)
        } pps;

        // SPR AVS APDO
        struct {
            uint16_t max_current_15v_20v;  // 最大电流 (mA)
            uint16_t max_current_9v_15v;   // 最大电流 (mA)
        } spr_avs;

        // EPR AVS APDO
        struct {
            uint16_t min_voltage;  // 最小电压 (mV)
            uint16_t max_voltage;  // 最大电压 (mV)
            uint8_t pdp;           // 功率 (W)
        } epr_avs;
    };
} Source_PDO_t;

typedef struct {
    Source_PDO_t pdos[32];
    uint8_t pdo_count;  // PDO 总数
} Source_PDO_Storage_t;

// 状态机
typedef enum {
    CC_DISCONNECTED,
    CC_CHECK_CONNECT,
    CC_CONNECT,
    CC_SPR_SOURCE_CAP_RECEIVED,
    CC_SEND_SPR_REQUEST,
    CC_WAIT_ACCEPT,
    CC_ACCEPT,
    CC_WAIT_PS_RDY,
    CC_PS_RDY,
    CC_GET_SOURCE_CAP,
    CC_SEND_EPR_MODE_ENTER,
    CC_WAIT_EPR_MODE_ENTER_RESPONSE,
    CC_WAIT_EPR_MODE_SOURCE_CAP,
    CC_EPR_SOURCE_CAP_RECEIVED,
    CC_REQUEST_EPR_CHUNK,
    CC_SEND_EPR_REQUEST,
    CC_IDLE,
} cc_state_t;

typedef struct {
    volatile cc_state_t cc_State;
    volatile cc_state_t cc_LastState;

    Source_PDO_Storage_t source_caps;

    volatile uint8_t cc1_ConnectTimes;
    volatile uint8_t cc2_ConnectTimes;
    volatile uint8_t cc_NoneTimes;
    volatile uint8_t cc_PD_Version;

    volatile uint8_t cc_PDO_Pos;             // 当前选择的 PDO 位置
    volatile uint8_t cc_USBPD_READY;         // USBPD 是否准备好
    volatile uint8_t cc_Source_EPR_Capable;  // source 是否支持 EPR
    volatile uint8_t cc_Cable_EPR_Capable;   // 线缆是否支持 EPR

    volatile uint8_t cc_SourceMessageID;
    volatile uint8_t cc_SinkMessageID;

    volatile uint8_t cc_SinkGoodCRCOver;
    volatile uint8_t cc_SourceGoodCRCOver;

    __attribute__((aligned(4))) uint8_t cc_spr_source_cap_buffer[28];  // SPR Source Capability 存储缓冲区
    volatile uint8_t cc_spr_source_cap_buffer_pdo_count;

    volatile uint8_t cc_EPR_Ready;             // EPR 是否已进入成功
    volatile uint16_t cc_EPR_KeepAlive_Timer;  // EPR Keep Alive 计时器

    volatile uint32_t cc_EPRSourceCap[11];                           // EPR Source Capabilities, 最多 11 个
    volatile uint8_t cc_CurrentChunkNumber;                          // 跟踪当前接收的块号
    volatile uint8_t cc_ChunkRequestSent;                            // 标记是否已发送分块请求
    __attribute__((aligned(4))) uint8_t cc_EPRSourceCapBuffer[128];  // 用于存储分块数据的缓冲区
    volatile uint16_t cc_EPRSourceCapBufferSize;                     // 当前缓冲区中的数据大小

} pd_control_t;

/******************************************************************************
 * Public Interface Functions
 *****************************************************************************/
bool usbpd_sink_get_ready(void);
void usbpd_sink_clear_ready(void);

void usbpd_sink_init(void);

void usbpd_sink_print_source_caps(void);
Source_PDO_Storage_t usbpd_sink_get_source_caps(void);

bool usbpd_sink_set_pdo_position(uint8_t position);
uint8_t usbpd_sink_get_pdo_position(void);
