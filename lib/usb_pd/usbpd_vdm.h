#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "usbpd_def.h"

// PD SID (0xFF00), the Standard ID allocated to the PD specification by USB-IF.
#define USBPD_SID_USBIF (0xFF00u)

typedef struct {
    uint16_t svid;
    uint8_t vdm_type;
    uint8_t version_major;
    uint8_t version_minor;
    uint8_t object_position;
    uint8_t command_type;
    uint8_t command;
} usbpd_vdm_header_info_t;

bool usbpd_vdm_unpack_header(uint32_t raw, usbpd_vdm_header_info_t *out);

uint32_t usbpd_vdm_pack_structured_header(uint16_t svid,
                                          uint8_t version_major,
                                          uint8_t version_minor,
                                          uint8_t object_position,
                                          uint8_t command_type,
                                          uint8_t command);

bool usbpd_vdm_discover_svids_contains(const uint8_t *rx_buffer, uint8_t num_data_objects, uint16_t target_id);
