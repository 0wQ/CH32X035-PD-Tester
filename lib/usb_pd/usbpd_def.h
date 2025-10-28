/******************************************************************************
 * USB_PD_R3_2 V1.1 2024-10.pdf
 *****************************************************************************/

#pragma once

#include <stdint.h>

/* Maximum length of USB PD data */
#define USBPD_DATA_MAX_LEN 34

/* get values from PDO representation */
#define POWER_DECODE_50MV(value)  ((uint16_t)(((value) * 50)))   // From 50mV  multiples to mV
#define POWER_DECODE_100MV(value) ((uint16_t)(((value) * 100)))  // From 100mV multiples to mV
#define POWER_DECODE_10MA(value)  ((uint16_t)(((value) * 10)))   // From 10mA  multiples to mA
#define POWER_DECODE_50MA(value)  ((uint16_t)(((value) * 50)))   // From 50mA  multiples to mA
#define POWER_DECODE_250MW(value) ((uint16_t)(((value) * 250)))  // From 250mW  multiples to mW

// PD PHY Channel
typedef enum {
    USBPD_CCNONE,
    USBPD_CC1,
    USBPD_CC2,
} USBPD_CC_State_t;

// 6.2.1.1.5 Specification Revision
typedef enum {
    USBPD_SPECIFICATION_REV1 = 0b00,      // Revision 1.0 (Deprecated)
    USBPD_SPECIFICATION_REV2 = 0b01,      // Revision 2.0
    USBPD_SPECIFICATION_REV3 = 0b10,      // Revision 3.x
    USBPD_SPECIFICATION_RESERVED = 0b11,  // Reserved, Shall Not be used
} USBPD_SpecificationRevision_t;

// Table 6.5 Control Message Types
typedef enum {
    USBPD_CONTROL_MSG_GOODCRC = 0x01u,
    USBPD_CONTROL_MSG_GOTOMIN = 0x02u,  // Deprecated
    USBPD_CONTROL_MSG_ACCEPT = 0x03u,
    USBPD_CONTROL_MSG_REJECT = 0x04u,
    USBPD_CONTROL_MSG_PING = 0x05u,  // Deprecated
    USBPD_CONTROL_MSG_PS_RDY = 0x06u,
    USBPD_CONTROL_MSG_GET_SRC_CAP = 0x07u,
    USBPD_CONTROL_MSG_GET_SNK_CAP = 0x08u,
    USBPD_CONTROL_MSG_DR_SWAP = 0x09u,
    USBPD_CONTROL_MSG_PR_SWAP = 0x0Au,
    USBPD_CONTROL_MSG_VCONN_SWAP = 0x0Bu,
    USBPD_CONTROL_MSG_WAIT = 0x0Cu,
    USBPD_CONTROL_MSG_SOFT_RESET = 0x0Du,
    USBPD_CONTROL_MSG_DATA_RESET = 0x0Eu,
    USBPD_CONTROL_MSG_DATA_RESET_COMPLETE = 0x0Fu,
    USBPD_CONTROL_MSG_NOT_SUPPORTED = 0x10u,
    USBPD_CONTROL_MSG_GET_SRC_CAP_EXTENDED = 0x11u,
    USBPD_CONTROL_MSG_GET_STATUS = 0x12u,
    USBPD_CONTROL_MSG_FR_SWAP = 0x13u,
    USBPD_CONTROL_MSG_GET_PPS_STATUS = 0x14u,
    USBPD_CONTROL_MSG_GET_COUNTRY_CODES = 0x15u,
    USBPD_CONTROL_MSG_GET_SNK_CAP_EXTENDED = 0x16u,
    USBPD_CONTROL_MSG_GET_SRC_INFO = 0x17u,
    USBPD_CONTROL_MSG_GET_REVISION = 0x18u,
} USBPD_ControlMessageType_t;

// Table 6.6 Data Message Types
typedef enum {
    USBPD_DATA_MSG_SRC_CAP = 0x01u,
    USBPD_DATA_MSG_REQUEST = 0x02u,
    USBPD_DATA_MSG_BIST = 0x03u,
    USBPD_DATA_MSG_SNK_CAP = 0x04u,
    USBPD_DATA_MSG_BATTERY_STATUS = 0x05u,
    USBPD_DATA_MSG_ALERT = 0x06u,
    USBPD_DATA_MSG_GET_COUNTRY_INFO = 0x07u,
    USBPD_DATA_MSG_ENTER_USB = 0x08u,
    USBPD_DATA_MSG_EPR_REQUEST = 0x09u,
    USBPD_DATA_MSG_EPR_MODE = 0x0Au,
    USBPD_DATA_MSG_SRC_INFO = 0x0Bu,
    USBPD_DATA_MSG_REVISION = 0x0Cu,
    USBPD_DATA_MSG_VENDOR_DEFINED = 0x0Fu,
} USBPD_DataMessageType_t;

// 6.2.1.1 Message Header
typedef union {
    uint16_t d16;
    struct {
        uint16_t MessageType             : 5u;
        uint16_t PortDataRole            : 1u;  // 0b->UFP, 1b->DFP (SOP Only)
        uint16_t SpecificationRevision   : 2u;  // 00b->v1.0, 01b->v2.0, 10b->v3.0
        uint16_t PortPowerRole_CablePlug : 1u;  // PortPowerRole: 0b->sink, 1b->source (SOP Only)
                                                // CablePlug: 0b->DFP/UFP, 1b->CablePlug/VPD (SOP’/SOP’’)
                                                // The Cable Plug field Shall only apply to SOP’ Packet and SOP’’ Packet types
        uint16_t MessageID           : 3u;
        uint16_t NumberOfDataObjects : 3u;
        uint16_t Extended            : 1u;
    } MessageHeader;
} USBPD_MessageHeader_t;

// 6.2.1.2 Extended Message Header
typedef union {
    uint16_t d16;
    struct {
        uint16_t DataSize     : 9u;
        uint16_t Reserved     : 1u;
        uint16_t RequestChunk : 1u;
        uint16_t ChunkNumber  : 4u;
        uint16_t Chunked      : 1u;
    } ExtendedMessageHeader;
} USBPD_ExtendedMessageHeader_t;

// 6.5 Extended Message
// Table 6.53 Extended Message Types
typedef enum {
    ExtendedMessageType_SourceCapabilitiesExtended = 0b00001,
    ExtendedMessageType_Status = 0b00010,
    ExtendedMessageType_GetBatteryCap = 0b00011,
    ExtendedMessageType_GetBatteryStatus = 0b00100,
    ExtendedMessageType_BatteryCapabilities = 0b00101,
    ExtendedMessageType_GetManufacturerInfo = 0b00110,
    ExtendedMessageType_ManufacturerInfo = 0b00111,
    ExtendedMessageType_SecurityRequest = 0b01000,
    ExtendedMessageType_SecurityResponse = 0b01001,
    ExtendedMessageType_FirmwareUpdateRequest = 0b01010,
    ExtendedMessageType_FirmwareUpdateResponse = 0b01011,
    ExtendedMessageType_PPSStatus = 0b01100,
    ExtendedMessageType_CountryInfo = 0b01101,
    ExtendedMessageType_CountryCodes = 0b01110,
    ExtendedMessageType_SinkCapabilitiesExtended = 0b01111,
    ExtendedMessageType_ExtendedControl = 0b10000,
    ExtendedMessageType_EPRSourceCapabilities = 0b10001,
    ExtendedMessageType_EPRSinkCapabilities = 0b10010,
    ExtendedMessageType_VendorDefinedExtended = 0b11110,
} USBPD_ExtendedMessageType_t;

// Table 6.29 Structured VDM Header
typedef union {
    uint32_t d32;
    struct {
        uint32_t Command                   : 5u;
        uint32_t Reserved                  : 1u;
        uint32_t CommandType               : 2u;
        uint32_t ObjectPosition            : 3u;
        uint32_t StructuredVDMVersionMinor : 2u;
        uint32_t StructuredVDMVersionMajor : 2u;
        uint32_t VDMType                   : 1u;
        uint32_t SVID                      : 16u;
    } StructuredVDMHeader;
} USBPD_StructuredVDMHeader_t;

// Table 6.41 Passive Cable VDO
typedef union {
    uint32_t d32;  // 原始 32-bit VDO 数据

    struct {
        uint32_t USBHighestSpeed      : 3u;  // B2…0  USB 最高速度
        uint32_t Reserved_4_3         : 2u;  // B4…3  保留
        uint32_t VBUSCurrentHandling  : 2u;  // B6…5  VBUS 电流承载能力
        uint32_t Reserved_8_7         : 2u;  // B8…7  保留
        uint32_t MaxVBUSVoltage       : 2u;  // B10…9 最大 VBUS 电压
        uint32_t CableTerminationType : 2u;  // B12…11 电缆终端类型
        uint32_t CableLatency         : 4u;  // B16…13 电缆延迟
        uint32_t EPRCapable           : 1u;  // B17 是否支持 EPR
        uint32_t USBTypeCPlug         : 2u;  // B19…18 USB Type-C 插头类型
        uint32_t Reserved_20          : 1u;  // B20 保留
        uint32_t VDOVersion           : 3u;  // B23…21 VDO 版本
        uint32_t FirmwareVersion      : 4u;  // B27…24 固件版本
        uint32_t HWVersion            : 4u;  // B31…28 硬件版本
    } PassiveCableVDO;
} USBPD_PassiveCableVDO_t;

// Table 6.33 ID Header VDO
typedef union {
    uint32_t d32;

    struct {
        uint32_t USBVendorID    : 16u;    // B15…0  USB Vendor ID
        uint32_t Reserved_20_16 : 5u;     // B20…16 保留，Shall Not be used
        uint32_t ConnectorType  : 2u;     // B22…21 连接器类型
                                          // 00b = Reserved
                                          // 01b = Reserved, Shall Not be used
                                          // 10b = USB Type-C Receptacle
                                          // 11b = USB Type-C Plug
        uint32_t SOPProductTypeDFP : 3u;  // B25…23 DFP 产品类型
                                          // 000b = Not a DFP
                                          // 001b = PDUSB Hub
                                          // 010b = PDUSB Host
                                          // 011b = Power Brick
                                          // 100b–111b = Reserved, Shall Not be used
        uint32_t ModalOperation    : 1u;  // B26 是否支持 Modal Operation (Alternate Modes)
        uint32_t SOPProductTypeUFP : 3u;  // B29…27 UFP 产品类型
                                          // 000b = Not a UFP
                                          // 001b = PDUSB Hub
                                          // 010b = PDUSB Peripheral
                                          // 011b = PSD
                                          // 100b–111b = Reserved, Shall Not be used
        uint32_t USBDeviceCapable : 1u;   // B30 是否具备 USB Device 枚举能力
        uint32_t USBHostCapable   : 1u;   // B31 是否具备 USB Host 枚举能力
    } IDHeaderVDO;
} USBPD_IDHeaderVDO_t;

// 6.4.1.1 Power Data Objects

// Table 6.7 Power Data Object (B31-B30)
typedef enum {
    PDO_TYPE_FIXED_SUPPLY = 0b00,     // Fixed Supply (Vmin = Vmax)
    PDO_TYPE_BATTERY = 0b01,          // Battery
    PDO_TYPE_VARIABLE_SUPPLY = 0b10,  // Variable Supply (non-Battery)
    PDO_TYPE_APDO = 0b11              // Augmented Power Data Object (APDO)
} USBPD_PDO_Type_t;

// Table 6.8 Augmented Power Data Object (B29-B28, only valid when B31-B30 = 0b11)
typedef enum {
    APDO_TYPE_SPR_PPS = 0b00,  // SPR Programmable Power Supply
    APDO_TYPE_EPR_AVS = 0b01,  // EPR Adjustable Voltage Supply
    APDO_TYPE_SPR_AVS = 0b10,  // SPR Adjustable Voltage Supply
    APDO_TYPE_RESERVED = 0b11  // Reserved
} USBPD_APDO_Subtype_t;

// 6.4.1.2 Source Power Data Objects
typedef union {
    uint32_t d32;

    // General structure for parsing PDO type
    struct {
        uint32_t SpecificFields : 28u;  // Specific fields depending on the PDO type
        uint32_t APDO_SubType   : 2u;   // B29-B28: APDO Subtype(only valid when B31-B30 = 0b11)
        uint32_t PDO_Type       : 2u;   // B31-B30: PDO Type
    } General;

    // Table 6.9 Fixed Supply PDO – Source
    struct {
        uint32_t MaxCurrentIn10mAunits             : 10u;  // Maximum current in 10mA units
        uint32_t VoltageIn50mVunits                : 10u;  // Voltage in 50mV units
        uint32_t PeakCurrent                       : 2u;   // Peak Current value
        uint32_t Reserved_22                       : 1u;   // Reserved – Shall be set to zero
        uint32_t EPRCapable                        : 1u;   // Set to 1 if EPR Capable.
        uint32_t UnchunkedExtendedMessageSupported : 1u;   // Set to 1 if Unchunked Extended Messages are supported.
        uint32_t DualRoleData                      : 1u;   // Set to 1 for a Dual-Role Data device.
        uint32_t USBCommunicationsCapable          : 1u;   // Set to 1 if capable of USB Communications capable
        uint32_t UnconstrainedPower                : 1u;   // Set to 1 if unconstrained power is available.
        uint32_t USBSuspendSupported               : 1u;   // Set to 1 if USB suspend is supported.
        uint32_t DualRolePower                     : 1u;   // Set to 1 for Dual-Role Power device.
        uint32_t FixedSupply                       : 2u;   // 00b - Fixed Supply PDO
    } Fixed;
    // } FixedSupplyPDO;

    // Table 6.11 Variable Supply (non-Battery) PDO – Source
    struct {
        uint32_t MaxCurrentIn10mAunits : 10u;  // Maximum current in 10mA units
        uint32_t MinVoltageIn50mVunits : 10u;  // Minimum voltage in 50mV units
        uint32_t MaxVoltageIn50mVunits : 10u;  // Maximum voltage in 50mV units
        uint32_t VariableSupply        : 2u;   // 01b - Variable Supply (non-Battery) PDO
    } Variable;
    // } VariableSupplyPDO;

    // Table 6.12 Battery Supply PDO – Source
    struct {
        uint32_t MaxAllowablePowerIn250mWunits : 10u;  // Maximum allowable power in 250mW units
        uint32_t MinVoltageIn50mVunits         : 10u;  // Minimum voltage in 50mV units
        uint32_t MaxVoltageIn50mVunits         : 10u;  // Maximum voltage in 50mV units
        uint32_t BatterySupply                 : 2u;   // 10b - Battery Supply PDO
    } Battery;
    // } BatterySupplyPDO;

    // Table 6.13 SPR Programmable Power Supply APDO – Source
    struct {
        uint32_t MaxCurrentIn50mAunits  : 7u;  // Maximum current in 50mA units
        uint32_t Reserved_7             : 1u;  // Reserved – Shall be set to zero
        uint32_t MinVoltageIn100mVunits : 8u;  // Minimum voltage in 100mV units
        uint32_t Reserved_16            : 1u;  // Reserved – Shall be set to zero
        uint32_t MaxVoltageIn100mVunits : 8u;  // Maximum voltage in 100mV units
        uint32_t Reserved_25_26         : 2u;  // Reserved – Shall be set to zero
        uint32_t PPSpowerLimited        : 1u;  // Set to 1 when PPS Power Limited
        uint32_t SPRPPS                 : 2u;  // 00b – SPR PPS
        uint32_t APDO                   : 2u;  // 11b – Augmented Power Data Object (APDO)
    } SPR_PPS;
    // } SPRProgrammablePowerSupplyAPDO;

    // Table 6.14 SPR Adjustable Voltage Supply APDO – Source
    // 下面描述了在 SPR 模式下的 9V-15V 15V-20V SPR AVS APDO
    struct {
        uint32_t MaxCurrentFor15V20VIn10mAunits : 10u;  // For 15V – 20V range: Maximum current in 10mA units equal to the Maximum Current field of the 20V Fixed Supply PDO, set to 0 if the maximum voltage in the SPR AVS range is 15V
        uint32_t MaxCurrentFor9V15VIn10mAunits  : 10u;  // For  9V – 15V range: Maximum current in 10mA units equal to the Maximum Current field of the 15V Fixed Supply PDO
        uint32_t Reserved_20_25                 : 6u;   // Reserved – Shall be set to zero
        uint32_t PeakCurrent                    : 2u;   // Peak Current (see Table 6.10, "Fixed Power Source Peak Current Capability"))
        uint32_t SPRAVS                         : 2u;   // 10b – SPR AVS
        uint32_t APDO                           : 2u;   // 11b – Augmented Power Data Object (APDO)
    } SPR_AVS;
    // } SPRAdjustableVoltageSupplyAPDO;

    // Table 6.15 EPR Adjustable Voltage Supply APDO – Source
    // 下面描述了在 EPR 模式下的 15V-48V EPR AVS APDO
    struct {
        uint32_t PDPIn1Wunits           : 8u;  // PDP in 1W units
        uint32_t MinVoltageIn100mVunits : 8u;  // Minimum voltage in 100mV units
        uint32_t Reserved_16            : 1u;  // Reserved – Shall be set to zero
        uint32_t MaxVoltageIn100mVunits : 9u;  // Maximum voltage in 100mV units
        uint32_t PeakCurrent            : 2u;  // Peak Current (see Table 6.10, "Fixed Power Source Peak Current Capability"))
        uint32_t EPRAVS                 : 2u;  // 01b – EPR AVS
        uint32_t APDO                   : 2u;  // 11b – Augmented Power Data Object (APDO)
    } EPR_AVS;
    // } EPRAdjustableVoltageSupplyAPDO;
} USBPD_SourcePDO_t;

// 6.4.1.3 Sink Power Data Objects
typedef union {
    uint32_t d32;

    // Table 6.17 Fixed Supply PDO – Sink
    struct {
        uint32_t OperationalCurrentIn10mAunits : 10u;  // B9..0  Operational current in 10mA units
        uint32_t VoltageIn50mVunits            : 10u;  // B19..10 Voltage in 50mV units
        uint32_t Reserved_22_20                : 3u;   // B22..20 Reserved – Shall be set to zero
        uint32_t FastRoleSwapCurrent           : 2u;   // B24..23 Fast Role Swap required USB Type-C Current
        uint32_t DualRoleData                  : 1u;   // B25 Dual-Role Data
        uint32_t USBCommunicationsCapable      : 1u;   // B26 USB Communications Capable
        uint32_t UnconstrainedPower            : 1u;   // B27 Unconstrained Power
        uint32_t HigherCapability              : 1u;   // B28 Higher Capability
        uint32_t DualRolePower                 : 1u;   // B29 Dual-Role Power
        uint32_t FixedSupply                   : 2u;   // B31..30 00b - Fixed Supply PDO
    } Fixed;
    // } FixedSupplyPDO;

    // Table 6.18 Variable Supply (non-Battery) PDO – Sink
    struct {
        uint32_t OperationalCurrentIn10mAunits : 10u;  // B9..0  Operational current in 10mA units
        uint32_t MinimumVoltageIn50mVunits     : 10u;  // B19..10 Minimum voltage in 50mV units
        uint32_t MaximumVoltageIn50mVunits     : 10u;  // B29..20 Maximum voltage in 50mV units
        uint32_t VariableSupply                : 2u;   // B31..30 01b - Variable Supply (non-Battery) PDO
    } Variable;
    // } VariableSupplyPDO;

    // Table 6.19 Battery Supply PDO – Sink
    struct {
        uint32_t OperationalPowerIn250mWunits : 10u;  // B9..0  Operational Power in 250mW units
        uint32_t MinimumVoltageIn50mVunits    : 10u;  // B19..10 Minimum voltage in 50mV units
        uint32_t MaximumVoltageIn50mVunits    : 10u;  // B29..20 Maximum voltage in 50mV units
        uint32_t BatterySupply                : 2u;   // B31..30 10b - Battery Supply PDO
    } Battery;
    // } BatterySupplyPDO;

    // Table 6.20 SPR Programmable Power Supply APDO – Sink
    struct {
        uint32_t MaximumCurrentIn50mAunits  : 7u;  // B6..0   Maximum current in 50mA increments
        uint32_t Reserved_7                 : 1u;  // B7      Reserved – Shall be set to zero
        uint32_t MinimumVoltageIn100mVunits : 8u;  // B15..8  Minimum voltage in 100mV increments
        uint32_t Reserved_16                : 1u;  // B16     Reserved – Shall be set to zero
        uint32_t MaximumVoltageIn100mVunits : 8u;  // B24..17 Maximum voltage in 100mV increments
        uint32_t Reserved_27_25             : 3u;  // B27..25 Reserved – Shall be set to zero
        uint32_t SPRPPS                     : 2u;  // B29..28 00b – SPR PPS
        uint32_t APDO                       : 2u;  // B31..30 11b – Augmented Power Data Object (APDO)
    } SPR_PPS;
    // } SPRProgrammablePowerSupplyAPDO;

    // Table 6.21 SPR Adjustable Voltage Supply APDO – Sink
    struct {
        uint32_t MaxCurrentFor20VIn10mAunits : 10u;  // B9..0   Maximum current (15V–20V range)
        uint32_t MaxCurrentFor15VIn10mAunits : 10u;  // B19..10 Maximum current (9V–15V range)
        uint32_t Reserved_27_20              : 8u;   // B27..20 Reserved – Shall be set to zero
        uint32_t SPRAVS                      : 2u;   // B29..28 10b – SPR AVS
        uint32_t APDO                        : 2u;   // B31..30 11b – Augmented Power Data Object (APDO)
    } SPR_AVS;
    // } SPRAdjustableVoltageSupplyAPDO;

    // Table 6.22 EPR Adjustable Voltage Supply APDO – Sink
    struct {
        uint32_t PDPIn1Wunits               : 8u;  // B7..0   PDP in 1W increments
        uint32_t MinimumVoltageIn100mVunits : 8u;  // B15..8  Minimum voltage in 100mV increments
        uint32_t Reserved_16                : 1u;  // B16     Reserved – Shall be set to zero
        uint32_t MaximumVoltageIn100mVunits : 9u;  // B25..17 Maximum voltage in 100mV increments
        uint32_t Reserved_27_26             : 2u;  // B27..26 Reserved – Shall be set to zero
        uint32_t EPRAVS                     : 2u;  // B29..28 01b – EPR AVS
        uint32_t APDO                       : 2u;  // B31..30 11b – Augmented Power Data Object (APDO)
    } EPR_AVS;
    // } EPRAdjustableVoltageSupplyAPDO;

} USBPD_SinkPDO_t;

// 6.4.2 Request Message
typedef union {
    uint32_t d32;

    // Table 6.23 Fixed and Variable Request Data Object
    struct {
        uint32_t MaxOperatingCurrent10mAunits       : 10u;  // B9..B0   Maximum Operating Current in 10mA units
        uint32_t OperatingCurrentIn10mAunits        : 10u;  // B19..B10 Operating Current in 10mA units
        uint32_t Reserved_20_21                     : 2u;   // B21..B20 Reserved – Shall be set to zero
        uint32_t EPRCapable                         : 1u;   // B22      Set to ‘1’ if EPR Capable
        uint32_t UnchunkedExtendedMessagesSupported : 1u;   // B23      Set to ‘1’ if Unchunked Extended Messages are supported
        uint32_t NoUSBSuspend                       : 1u;   // B24      Set to ‘1’ if requesting No USB Suspend
        uint32_t USBCommunicationsCapable           : 1u;   // B25      Set to ‘1’ if USB Communications Capable
        uint32_t CapabilityMismatch                 : 1u;   // B26      Set to ‘1’ for a Capabilities Mismatch
        uint32_t GiveBackFlag                       : 1u;   // B27      Deprecated – Shall be set to zero
        uint32_t ObjectPosition                     : 4u;   // B31..B28 Object Position (0000b and 1110b–1111b are Reserved)
    } FixedAndVariable;

    // Table 6.24 Battery Request Data Object
    struct {
        uint32_t MaxOperatingPower                  : 10u;  // B9..B0   Maximum Operating Power in 250mW units
        uint32_t OperatingPower                     : 10u;  // B19..B10 Operating Power in 250mW units
        uint32_t Reserved_20_21                     : 2u;   // B21..B20 Reserved – Shall be set to zero
        uint32_t EPRCapable                         : 1u;   // B22      Set to ‘1’ if EPR Capable
        uint32_t UnchunkedExtendedMessagesSupported : 1u;   // B23      Set to ‘1’ if Unchunked Extended Messages are supported
        uint32_t NoUSBSuspend                       : 1u;   // B24      Set to ‘1’ if requesting No USB Suspend
        uint32_t USBCommunicationsCapable           : 1u;   // B25      Set to ‘1’ if USB Communications Capable
        uint32_t CapabilityMismatch                 : 1u;   // B26      Set to ‘1’ for a Capabilities Mismatch
        uint32_t GiveBackFlag                       : 1u;   // B27      Deprecated – Shall be set to zero
        uint32_t ObjectPosition                     : 4u;   // B31..B28 Object Position (0000b and 1110b–1111b are Reserved)
    } Battery;

    // Table 6.25 PPS Request Data Object
    struct {
        uint32_t OperatingCurrentIn50mAunits        : 7u;   // B6..B0   Operating current in 50mA units
        uint32_t Reserved_7_8                       : 2u;   // B8..B7   Reserved – Shall be set to zero
        uint32_t OutputVoltageIn20mVunits           : 12u;  // B20..B9  Output voltage in 20mV units
        uint32_t Reserved_21                        : 1u;   // B21      Reserved – Shall be set to zero
        uint32_t EPRCapable                         : 1u;   // B22      Set to ‘1’ if EPR Capable
        uint32_t UnchunkedExtendedMessagesSupported : 1u;   // B23      Set to ‘1’ if Unchunked Extended Messages Supported
        uint32_t NoUSBSuspend                       : 1u;   // B24      Set to ‘1’ if requesting No USB Suspend
        uint32_t USBCommunicationsCapable           : 1u;   // B25      Set to ‘1’ if USB Communications Capable
        uint32_t CapabilityMismatch                 : 1u;   // B26      Set to ‘1’ for a Capabilities Mismatch
        uint32_t Reserved_27                        : 1u;   // B27      Reserved – Shall be set to zero
        uint32_t ObjectPosition                     : 4u;   // B31..B28 Object position (0000b and 1110b–1111b are Reserved)
    } PPS;
    // } ProgrammablePowerSupply;

    // Table 6.26 AVS Request Data Object
    struct {
        uint32_t OperatingCurrentIn50mAunits        : 7u;   // B6..B0   Operating current in 50mA units
        uint32_t Reserved_7_8                       : 2u;   // B8..B7   Reserved – Shall be set to zero
        uint32_t OutputVoltageIn25mVunits           : 12u;  // B20..B9  Output voltage in 25mV units, the least two significant bits Shall be set to zero making the effective voltage step size 100mV
        uint32_t Reserved_21                        : 1u;   // B21      Reserved – Shall be set to zero
        uint32_t EPRCapable                         : 1u;   // B22      Set to ‘1’ if EPR Capable
        uint32_t UnchunkedExtendedMessagesSupported : 1u;   // B23      Set to ‘1’ if Unchunked Extended Messages Supported
        uint32_t NoUSBSuspend                       : 1u;   // B24      Set to ‘1’ if requesting No USB Suspend
        uint32_t USBCommunicationsCapable           : 1u;   // B25      Set to ‘1’ if USB Communications Capable
        uint32_t CapabilityMismatch                 : 1u;   // B26      Set to ‘1’ for a Capabilities Mismatch
        uint32_t Reserved_27                        : 1u;   // B27      Reserved – Shall be set to zero
        uint32_t ObjectPosition                     : 4u;   // B31..B28 Object position (0000b and 1110b–1111b are Reserved)
    } AVS;
    // } AdjustableVoltageSupply;
} USBPD_RDO_t;

typedef enum {
    USBPD_SPR_PPS_VOLTAGE_UNIT = 20,
    USBPD_SPR_AVS_VOLTAGE_UNIT = 25,
    USBPD_EPR_AVS_VOLTAGE_UNIT = 100,
} USBPD_APDO_VOLTAGE_UNIT_t;