#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include "usbpd_def.h"

// Page 260: SinkEPRKeepAlive Timer
// tSinkEPRKeepAlive   min:0.250s nom:0.375s max:0.500s
#define EPR_KEEP_ALIVE_TIMEOUT 40  // 单位: 10ms
// #define EPR_KEEP_ALIVE_TIMEOUT 5  // 单位: 10ms

typedef struct {
    PDO_Type_t type;        // PDO 类型
    uint8_t position;       // PDO position
    uint32_t raw;           // 原始 PDO 数据
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

// PDO 存储结构体
typedef struct {
    Source_PDO_t pdos[32];  // 最多支持 32 个 PDO
    uint8_t pdo_count;      // PDO 总数
} Source_PDO_Storage_t;

// 状态机
typedef enum {
    CC_IDLE,
    CC_CHECK_CONNECT,
    CC_CONNECT,
    CC_SOURCE_CAP,
    CC_SEND_SPR_REQUEST,
    CC_WAIT_ACCEPT,
    CC_ACCEPT,
    CC_WAIT_PS_RDY,
    CC_PS_RDY,
    CC_GET_SOURCE_CAP,
    CC_WAIT_SOURCE_CAP,
    CC_SEND_EPR_MODE_ENTER,
    CC_WAIT_EPR_RESPONSE,
    CC_GET_EPR_CAP,
    CC_WAIT_EPR_CAP,
    CC_REQUEST_EPR_CHUNK,
    CC_SEND_EPR_REQUEST,
} cc_state_t;

typedef struct {
    volatile cc_state_t cc_State;
    volatile cc_state_t cc_LastState;

    Source_PDO_Storage_t source_caps;

    volatile uint8_t cc1_ConnectTimes;
    volatile uint8_t cc2_ConnectTimes;
    volatile uint8_t cc_NoneTimes;
    volatile uint8_t cc_PD_Version;
    volatile uint8_t cc_PDO_Pos;  // TODO:待删除
    volatile uint8_t cc_Last_PDO_Pos;
    volatile uint8_t cc_USBPD_READY;
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

    uint32_t cc_EPRSourceCap[11];                                    // EPR Source Capabilities, 最多 11 个 PDO
    volatile uint8_t cc_CurrentChunkNumber;                          // 跟踪当前接收的块号
    volatile uint8_t cc_ChunkRequestSent;                            // 标记是否已发送分块请求
    __attribute__((aligned(4))) uint8_t cc_EPRSourceCapBuffer[128];  // 用于存储分块数据的缓冲区
    uint16_t cc_EPRSourceCapBufferSize;                              // 当前缓冲区中的数据大小

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

#ifdef __cplusplus
}
#endif
