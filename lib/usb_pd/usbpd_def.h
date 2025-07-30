/******************************************************************************
 * Copyright (c) 2024
 *
 * USB_PD_R3_2 V1.1 2024-10.pdf
 *
 *****************************************************************************/

#pragma once

#include <stdint.h>

/* get values from PDO representation */
#define POWER_DECODE_50MV(value)  ((uint16_t)(((value) * 50)))  /* From 50mV  multiples to mV */
#define POWER_DECODE_100MV(value) ((uint16_t)(((value) * 100))) /* From 100mV multiples to mV */
#define POWER_DECODE_10MA(value)  ((uint16_t)(((value) * 10)))  /* From 10mA  multiples to mA */
#define POWER_DECODE_50MA(value)  ((uint16_t)(((value) * 50)))  /* From 50mA  multiples to mA */
#define POWER_DECODE_250MW(value) ((uint16_t)(((value) * 250))) /* From 250mW  multiples to mW */

/* PD PHY Channel */
typedef enum {
    USBPD_CCNONE,
    USBPD_CC1,
    USBPD_CC2,
} USBPD_CC_State_t;

// 6.2.1.1.5 Specification Revision
typedef enum {
    USBPD_SPECIFICATION_REV1,      // Revision 1.0 (Deprecated)
    USBPD_SPECIFICATION_REV2,      // Revision 2.0
    USBPD_SPECIFICATION_REV3,      // Revision 3.x
    USBPD_SPECIFICATION_RESERVED,  // Reserved, Shall Not be used
} USBPD_Specification_Revision_t;

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
} USBPD_ControlMessage;

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
} USBPD_DataMessage;

typedef union {
    uint16_t d16;
    USBPD_ControlMessage ControlMessage;
    USBPD_DataMessage DataMessage;
} USBPD_MessageType_t;

// 6.2.1.1 Message Header
typedef union {
    uint16_t d16;
    struct {
        uint16_t MessageType : 5u;
        uint16_t PortDataRole : 1u;             // 0b->UFP, 1b->DFP (SOP Only)
        uint16_t SpecificationRevision : 2u;    // 00b->v1.0, 01b->v2.0, 10b->v3.0
        uint16_t PortPowerRole_CablePlug : 1u;  // PortPowerRole: 0b->sink, 1b->source (SOP Only)
                                                // CablePlug: 0b->DFP/UFP, 1b->CablePlug/VPD (SOP’/SOP’’)
                                                // The Cable Plug field Shall only apply to SOP’ Packet and SOP’’ Packet types
        uint16_t MessageID : 3u;
        uint16_t NumberOfDataObjects : 3u;
        uint16_t Extended : 1u;
    } MessageHeader;
} USBPD_MessageHeader_t;

// 6.2.1.2 Extended Message Header
typedef union {
    uint16_t d16;
    struct {
        uint16_t DataSize : 9;
        uint16_t Reserved : 1;
        uint16_t RequestChunk : 1;
        uint16_t ChunkNumber : 4;
        uint16_t Chunked : 1;
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

// 6.4.1.1 Power Data Objects

// Table 6.7 Power Data Object (B31-B30)
typedef enum {
    PDO_TYPE_FIXED_SUPPLY = 0b00,     // Fixed Supply (Vmin = Vmax)
    PDO_TYPE_BATTERY = 0b01,          // Battery
    PDO_TYPE_VARIABLE_SUPPLY = 0b10,  // Variable Supply (non-Battery)
    PDO_TYPE_APDO = 0b11              // Augmented Power Data Object (APDO)
} PDO_Type_t;

// Table 6.8 Augmented Power Data Object (B29-B28, only valid when B31-B30 = 0b11)
typedef enum {
    APDO_TYPE_SPR_PPS = 0b00,  // SPR Programmable Power Supply
    APDO_TYPE_EPR_AVS = 0b01,  // EPR Adjustable Voltage Supply
    APDO_TYPE_SPR_AVS = 0b10,  // SPR Adjustable Voltage Supply
    APDO_TYPE_RESERVED = 0b11  // Reserved
} APDO_Subtype_t;

// 6.4.1.2 Source Power Data Objects
typedef union {
    uint32_t d32;

    // General structure for parsing PDO type
    struct {
        uint32_t SpecificFields : 28;            // Specific fields depending on the PDO type
        APDO_Subtype_t SubTypeOrOtherUsage : 2;  // B29-B28: APDO Subtype or other usage
        PDO_Type_t PDO_Type : 2;                 // B31-B30: PDO Type
    } General;

    // Table 6.9 Fixed Supply PDO – Source
    struct {
        uint32_t MaxCurrentIn10mAunits : 10u;             // Maximum current in 10mA units
        uint32_t VoltageIn50mVunits : 10u;                // Voltage in 50mV units
        uint32_t PeakCurrent : 2u;                        // Peak Current value
        uint32_t Reserved_22 : 1u;                        // Reserved – Shall be set to zero
        uint32_t EPRCapable : 1u;                         // Set to 1 if EPR Capable.
        uint32_t UnchunkedExtendedMessageSupported : 1u;  // Set to 1 if Unchunked Extended Messages are supported.
        uint32_t DualRoleData : 1u;                       // Set to 1 for a Dual-Role Data device.
        uint32_t USBCommunicationsCapable : 1u;           // Set to 1 if capable of USB Communications capable
        uint32_t UnconstrainedPower : 1u;                 // Set to 1 if unconstrained power is available.
        uint32_t USBSuspendSupported : 1u;                // Set to 1 if USB suspend is supported.
        uint32_t DualRolePower : 1u;                      // Set to 1 for Dual-Role Power device.
        uint32_t FixedSupply : 2u;                        // 00b - Fixed Supply PDO
    } SourceFixedSupplyPDO;

    // Table 6.11 Variable Supply (non-Battery) PDO – Source
    struct {
        uint32_t MaxCurrentIn10mAunits : 10u;  // Maximum current in 10mA units
        uint32_t MinVoltageIn50mVunits : 10u;  // Minimum voltage in 50mV units
        uint32_t MaxVoltageIn50mVunits : 10u;  // Maximum voltage in 50mV units
        uint32_t VariableSupply : 2u;          // 01b - Variable Supply (non-Battery) PDO
    } SourceVariableSupplyPDO;

    // Table 6.12 Battery Supply PDO – Source
    struct {
        uint32_t MaxAllowablePowerIn250mWunits : 10u;  // Maximum allowable power in 250mW units
        uint32_t MinVoltageIn50mVunits : 10u;          // Minimum voltage in 50mV units
        uint32_t MaxVoltageIn50mVunits : 10u;          // Maximum voltage in 50mV units
        uint32_t BatterySupply : 2u;                   // 10b - Battery Supply PDO
    } SourceBatterySupplyPDO;

    // Table 6.13 SPR Programmable Power Supply APDO – Source
    struct {
        uint32_t MaxCurrentIn50mAunits : 7u;   // Maximum current in 50mA units
        uint32_t Reserved_7 : 1u;              // Reserved – Shall be set to zero
        uint32_t MinVoltageIn100mVunits : 8u;  // Minimum voltage in 100mV units
        uint32_t Reserved_16 : 1u;             // Reserved – Shall be set to zero
        uint32_t MaxVoltageIn100mVunits : 8u;  // Maximum voltage in 100mV units
        uint32_t Reserved_25_26 : 2u;          // Reserved – Shall be set to zero
        uint32_t PPSpowerLimited : 1u;         // Set to 1 when PPS Power Limited
        uint32_t SPRPPS : 2u;                  // 00b – SPR PPS
        uint32_t APDO : 2u;                    // 11b – Augmented Power Data Object (APDO)
    } SourceSPRProgrammablePowerAPDO;

    // Table 6.14 SPR Adjustable Voltage Supply APDO – Source
    // 下面描述了在 SPR 模式下运行并提供 9V 至 20V 电​​压的源的 SPR AVS APDO
    struct {
        uint32_t MaxCurrentFor15V20VIn10mAunits : 10u;  // For 15V – 20V range: Maximum current in 10mA units equal to the Maximum Current field of the 20V Fixed Supply PDO, set to 0 if the maximum voltage in the SPR AVS range is 15V
        uint32_t MaxCurrentFor9V15VIn10mAunits : 10u;   // For  9V – 15V range: Maximum current in 10mA units equal to the Maximum Current field of the 15V Fixed Supply PDO
        uint32_t Reserved_20_25 : 6u;                   // Reserved – Shall be set to zero
        uint32_t PeakCurrent : 2u;                      // Peak Current (see Table 6.10, "Fixed Power Source Peak Current Capability"))
        uint32_t SPRAVS : 2u;                           // 10b – SPR AVS
        uint32_t APDO : 2u;                             // 11b – Augmented Power Data Object (APDO)
    } SourceSPRAdjustableVoltageSupplyAPDO;

    // Table 6.15 EPR Adjustable Voltage Supply APDO – Source
    struct {
        uint32_t PDPIn1Wunits : 8u;            // PDP in 1W units
        uint32_t MinVoltageIn100mVunits : 8u;  // Minimum voltage in 100mV units
        uint32_t Reserved_16 : 1u;             // Reserved – Shall be set to zero
        uint32_t MaxVoltageIn100mVunits : 9u;  // Maximum voltage in 100mV units
        uint32_t PeakCurrent : 2u;             // Peak Current (see Table 6.10, "Fixed Power Source Peak Current Capability"))
        uint32_t EPRAVS : 2u;                  // 01b – EPR AVS
        uint32_t APDO : 2u;                    // 11b – Augmented Power Data Object (APDO)
    } SourceEPRAdjustableVoltageSupplyAPDO;
} USBPD_SourcePDO_t;

// 6.4.2 Request Message
typedef union {
    uint32_t d32;
    // Table 6.23 Fixed and Variable Request Data Object
    struct {
        uint32_t MaxOperatingCurrent10mAunits : 10u;
        uint32_t OperatingCurrentIn10mAunits : 10u;
        uint32_t Reserved_20_21 : 2u;
        uint32_t EPRCapable : 1u;
        uint32_t UnchunkedExtendedMessage : 1u;
        uint32_t NoUSBSuspend : 1u;
        uint32_t USBCommunicationsCapable : 1u;
        uint32_t CapabilityMismatch : 1u;
        uint32_t GiveBackFlag : 1u;
        uint32_t ObjectPosition : 4u;
    } FixedAndVariableRequestDataObject;
    // Table 6.24 Battery Request Data Object
    // TODO...
    // Table 6.25 PPS Request Data Object
    struct {
        uint32_t OperatingCurrentIn50mAunits : 7u;
        uint32_t Reserved_7_8 : 2u;
        uint32_t OutputVoltageIn20mVunits : 12u;
        uint32_t Reserved_21 : 1u;
        uint32_t EPRCapable : 1u;
        uint32_t UnchunkedExtendedMessagesSupported : 1u;
        uint32_t NoUSBSuspend : 1u;
        uint32_t USBCommunicationsCapable : 1u;
        uint32_t CapabilityMismatch : 1u;
        uint32_t Reserved_27 : 1u;
        uint32_t ObjectPosition : 4u;
    } PPSRequestDataObject;
    // Table 6.26 AVS Request Data Object
    struct {
        uint32_t OperatingCurrentIn50mAunits : 7u;
        uint32_t Reserved_7_8 : 2u;
        uint32_t OutputVoltageIn25mVunits : 12u;  // Output voltage in 25mV units, the least two
                                                  // significant bits Shall be set to zero making the
                                                  // effective voltage step size 100mV
        uint32_t Reserved_21 : 1u;
        uint32_t EPRCapable : 1u;
        uint32_t UnchunkedExtendedMessagesSupported : 1u;
        uint32_t NoUSBSuspend : 1u;
        uint32_t USBCommunicationsCapable : 1u;
        uint32_t CapabilityMismatch : 1u;
        uint32_t Reserved_27 : 1u;
        uint32_t ObjectPosition : 4u;
    } AVSRequestDataObject;
} USBPD_RequestDataObject_t;
