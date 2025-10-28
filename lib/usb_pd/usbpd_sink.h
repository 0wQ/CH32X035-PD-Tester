#pragma once

#include <stdbool.h>
#include <string.h>
#include "usbpd_def.h"

// 调试打印
#define CONFIG_PD_DEBUG_ENABLE

// 虚拟 E-Marker
// #define CONFIG_EMARKER_ENABLE

// EPR Mode
#define CONFIG_EPR_MODE_ENABLE

// 硬件最高电压限制 (mV)
#define CONFIG_HW_MAX_VOLTAGE (36000)  // TODO: 发送请求时 FPDO 已限制，其他类型 APDO 尚未实现限制，应该还需在更新 pd_available_pdos_t 时做限制

// 不应小于 5V
#if CONFIG_HW_MAX_VOLTAGE < 5000
#undef CONFIG_HW_MAX_VOLTAGE
#define CONFIG_HW_MAX_VOLTAGE 5000
#endif

#ifdef CONFIG_PD_DEBUG_ENABLE
#define pd_printf(format, ...) LOG(format, ##__VA_ARGS__)
#else
#define pd_printf(x...)
#endif

// USB_PD_R3_2 V1.1 2024-10.pdf (Page 260, 6.6.21.2 SinkEPRKeepAlive Timer)
// tSinkEPRKeepAlive (min:0.250s nom:0.375s max:0.500s)
#define tSinkEPRKeepAlive (500 - 50)

// USB_PD_R3_2 V1.1 2024-10.pdf (Page 259, 6.6.19.1 SinkPPSPeriodic Timer)
// tPPSRequest (max:10s)
#define tPPSRequest (10000 - 50)

// 用于存储可用的 PDO 以供后续发送请求使用
typedef struct {
    uint32_t raw;          // PDO 原始数据
    uint8_t position;      // PDO Position
    uint8_t pdo_type;      // PDO 类型
    uint8_t apdo_subtype;  // APDO 子类型

    union {
        // Fixed Supply PDO
        struct {
            uint16_t voltage;      // 电压 (mV)
            uint16_t current;      // 电流 (mA)
            uint16_t epr_capable;  // 是否支持 EPR
        } fixed;

        // SPR PPS APDO
        struct {
            uint16_t min_voltage;  // 最小电压 (mV)
            uint16_t max_voltage;  // 最大电压 (mV)
            uint16_t current;      // 电流 (mA)
        } pps;

        // SPR AVS APDO
        struct {
            uint16_t min_voltage;          // 最小电压 (mV)
            uint16_t max_voltage;          // 最大电压 (mV)
            uint16_t max_current_9v_15v;   // 9-15V 最大电流 (mA)
            uint16_t max_current_15v_20v;  // 15-20V 最大电流 (mA)
        } spr_avs;

        // EPR AVS APDO
        struct {
            uint16_t min_voltage;  // 最小电压 (mV)
            uint16_t max_voltage;  // 最大电压 (mV)
            uint16_t pdp;          // 功率 (W)
        } epr_avs;
    };
} pd_pdo_t;

typedef struct {
    pd_pdo_t pdo[11];   // 最多 11 个 PDO
    uint8_t pdo_count;  // PDO 计数
} pd_available_pdos_t;

// PD 状态机
// TODO: 按照 Table 8.154 Policy Engine States 规范命名 PE_SNK_XXX
// TODO: 实现 wait 状态的超时退出
typedef enum {
    PD_STATE_DISCONNECTED = 0,
    PD_STATE_CHECK_CONNECT,
    PD_STATE_CONNECT,

    PD_STATE_RECEIVED_SPR_SOURCE_CAP,
    PD_STATE_SEND_SPR_REQUEST,
    PD_STATE_WAIT_ACCEPT,
    PD_STATE_WAIT_PS_RDY,
    PD_STATE_RECEIVED_PS_RDY,

    PD_STATE_SEND_EPR_ENTER,
    PD_STATE_WAIT_EPR_ENTER_RESPONSE,
    PD_STATE_WAIT_EPR_MODE_SOURCE_CAP,
    PD_STATE_RECEIVED_EPR_SOURCE_CAP,
    PD_STATE_SEND_EPR_SRC_CAP_REQ_CHUNK,
    PD_STATE_SEND_EPR_REQUEST,
    PD_STATE_WAIT_EPR_KEEP_ALIVE_ACK,

    PD_STATE_SEND_NOT_SUPPORTED,
    PD_STATE_SEND_REJECT,

    MIPPS_STATE_SEND_DRSWAP,
    MIPPS_STATE_WAIT_DRSWAP_RESPONSE,

    MIPPS_STATE_SEND_VDM_REQ_DISCOVER_IDENTITY,
    MIPPS_STATE_WAIT_VDM_ACK_DISCOVER_IDENTITY,  // 判断 USB Vendor ID, TODO: id 错误时，还需要再判断是否需要进入 EPR_MODE

    MIPPS_STATE_SEND_VDM_REQ_DISCOVER_SVIDS,
    MIPPS_STATE_WAIT_VDM_ACK_DISCOVER_SVIDS,  // TODO: 判断 SVID 0x2717 ?

    MIPPS_STATE_SEND_VDM_1,  // 0x27170101
    MIPPS_STATE_WAIT_VDM_1,

    MIPPS_STATE_SEND_VDM_2,  // 0x27170102
    MIPPS_STATE_WAIT_VDM_2,

    MIPPS_STATE_SEND_VDM_3,  // 0x27170103
    MIPPS_STATE_WAIT_VDM_3,

    MIPPS_STATE_SEND_VDM_4,  // 0x27170104 0xD6E97705 0x06C7F5F4 0xDF7A4000 0x8C7467B8
    MIPPS_STATE_WAIT_VDM_4,

    MIPPS_STATE_SEND_VDM_5,  // 0x27170105 0x464E9E0C 0xEE13382B 0x4124E2DA 0x43DDF86F
    MIPPS_STATE_WAIT_VDM_5,

    MIPPS_STATE_SEND_VDM_6,  // 0x27170106 0x00000001
    MIPPS_STATE_WAIT_VDM_6,

    MIPPS_STATE_SEND_VDM_7,  // 0x27170107 0x00000001
    MIPPS_STATE_WAIT_VDM_7,  // TODO: 可能需要 drswap 回来再发 get_src_cap

    MIPPS_STATE_SEND_GET_SRC_CAP,

    PD_STATE_IDLE,
} pd_state_t;

typedef struct {
    volatile bool is_ready;                               // PD 是否已就绪（已发送第一次电源请求，并且 Source 已回复 RS_RDY）
    volatile USBPD_SpecificationRevision_t pd_version;    // PD Specification Revision，需在收到 Source_Capabilities 后设置为和 Source 相同的 version
    volatile pd_state_t pd_state;                         // PD 状态机当前状态
    volatile pd_state_t pd_last_state;                    // PD 状态机之前状态
    volatile uint8_t pdo_pos;                             // 当前选择的 PDO Position
    volatile uint16_t pdo_voltage_mv;                     // 当前选择的 PDO 电压 (mV)，用于传递 APDO 电压
    pd_available_pdos_t available_pdos;                   // 用于存储可用的 PDO 以供后续发送请求使用
    uint32_t spr_source_cap_buffer[7];                    // SPR Source Capability 缓冲区（SPR 最多 7 个 PDO）
    volatile uint8_t spr_source_cap_buffer_pdo_count;     // SPR Source Capability 缓冲区中的 PDO 数量
    uint32_t epr_source_cap_buffer[11];                   // EPR Source Capability 缓冲区（EPR 最多 11 个 PDO）
    volatile uint8_t epr_source_cap_buffer_pdo_count;     // EPR Source Capability 缓冲区中的 PDO 数量
    volatile uint8_t epr_source_cap_buffer_size;          // EPR Source Capability 缓冲区大小（用于分块接收）
    volatile uint8_t epr_source_cap_buffer_chunk_number;  // EPR Source Capability 缓冲区正在接收的分块号
    volatile uint8_t cc_none_times;                       // cc 未连接计数
    volatile uint8_t cc1_connect_times;                   // cc1 检测计数
    volatile uint8_t cc2_connect_times;                   // cc2 检测计数
    volatile uint8_t sink_message_id;                     // Sink 消息 ID
    volatile uint8_t cable_message_id;                    // Cable 消息 ID，用于模拟 E-Marker
    volatile bool sink_goodcrc_over;                      // TODO: 暂未使用，待删除 // 已回复 GoodCRC（需在回复 GoodCRC 后标记 true，需在收到非 GoodCRC 消息时标记 false）在发送前等待 true，发送后设为 false
    volatile bool source_goodcrc_over;                    // TODO: 暂未使用，待删除 // 已收到 GoodCRC（需在收到 GoodCRC 消息时标记 true，需在发送非 GoodCRC 消息后标记 false）
    volatile bool is_epr_ready;                           // EPR 是否已进入成功
    volatile bool source_epr_capable;                     // Source 是否支持 EPR
    volatile bool cable_epr_capable;                      // Cable 是否支持 EPR
    volatile uint32_t epr_keepalive_timer;                // SinkEPRKeepAliveTimer 定时器，需在收到和回复 GoodCRC 后重置，超时 tSinkEPRKeepAlive 需发送 EPR Keep Alive
    volatile uint32_t pps_periodic_timer;                 // SinkPPSPeriodicTimer 定时器，需在收到和回复 GoodCRC 后重置，超时 tPPSRequest 需重新发送 SPR Request

    bool mipps_is_drswap_requested;  // MIPPS 尚未发送 drswap
} pd_control_t;

/******************************************************************************
 * Public Interface Functions
 *****************************************************************************/

/**
 * @brief 初始化 PD Sink
 */
void usbpd_sink_init(void);

/**
 * @brief 获取 PD 通信就绪状态 (已发送第一次电源请求，并且 Source 已回复 RS_RDY)
 */
bool usbpd_sink_get_ready(void);

/**
 * @brief 获取 EPR Mode 就绪状态
 */
bool usbpd_sink_get_epr_ready(void);

/**
 * @brief 调试打印 available_pdos
 */
void usbpd_sink_debug_available_pdos(void);

/**
 * @brief Get the available PDOs (Power Data Objects)
 * @return pd_available_pdos_t Structure containing the available PDOs.
 */
const pd_available_pdos_t *usbpd_sink_get_available_pdos(void);

/**
 * @brief 获取当前 Position
 * @return uint8_t Position (1-based), 如果未连接或无效则返回 0
 */
uint8_t usbpd_sink_get_position(void);

/**
 * @brief 设置 Fixed Supply PDO Position
 * @param position Position (1-based)
 * @return true 成功
 * @return false 失败
 */
bool usbpd_sink_set_fpdo_position(uint8_t position);

/**
 * @brief 设置 APDO Position 并指定电压
 * @param position Position (1-based)
 * @param voltage_mv 请求的电压 (mV)，必须在 PDO 的 min_voltage 和 max_voltage 范围内，且符合步进要求
 * @return true 成功
 * @return false 失败
 */
bool usbpd_sink_set_apdo_position_with_voltage(uint8_t position, uint16_t voltage_mv);

/**
 * @brief 发送 HARD_RESET
 */
void usbpd_sink_hard_reset(void);

/**
 * @brief 获取当前 PDO
 * @param pdo_type PDO 类型
 * @return true 成功
 * @return false 失败
 */
bool usbpd_sink_get_current_pdo_type(USBPD_PDO_Type_t *pdo_type, USBPD_APDO_Subtype_t *apdo_subtype);

/**
 * @brief 获取当前 PDO 类型和 APDO 子类型
 * @param pdo_type PDO 类型
 * @param apdo_subtype APDO 子类型
 * @return true 成功
 * @return false 失败
 */
bool usbpd_sink_get_current_pdo_type(USBPD_PDO_Type_t *pdo_type, USBPD_APDO_Subtype_t *apdo_subtype);

/**
 * @brief 查找指定电压的 pdo position
 * @param voltage_mv 电压 (mV)
 * @return uint8_t position (1-based), 如果未找到则返回 0
 */
uint8_t usbpd_sink_find_pdo_position_by_voltage(uint16_t voltage_mv);
