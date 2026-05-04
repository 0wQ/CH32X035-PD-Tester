#include "usbpd_vdm.h"

#include <stddef.h>

bool usbpd_vdm_unpack_header(uint32_t raw, usbpd_vdm_header_info_t *out) {
    if (out == NULL) {
        return false;
    }

    out->svid = (uint16_t)((raw >> 16) & 0xFFFFu);
    out->vdm_type = (uint8_t)((raw >> 15) & 0x01u);
    out->version_major = (uint8_t)((raw >> 13) & 0x03u);
    out->version_minor = (uint8_t)((raw >> 11) & 0x03u);
    out->object_position = (uint8_t)((raw >> 8) & 0x07u);
    out->command_type = (uint8_t)((raw >> 6) & 0x03u);
    out->command = (uint8_t)(raw & 0x1Fu);
    return true;
}

uint32_t usbpd_vdm_pack_structured_header(uint16_t svid,
                                          uint8_t version_major,
                                          uint8_t version_minor,
                                          uint8_t object_position,
                                          uint8_t command_type,
                                          uint8_t command) {
    uint32_t raw = 0;

    raw |= (uint32_t)(command & 0x1Fu);
    raw |= (uint32_t)(command_type & 0x03u) << 6;
    raw |= (uint32_t)(object_position & 0x07u) << 8;
    raw |= (uint32_t)(version_minor & 0x03u) << 11;
    raw |= (uint32_t)(version_major & 0x03u) << 13;
    raw |= (uint32_t)VDM_TYPE_STRUCTURED << 15;
    raw |= (uint32_t)svid << 16;

    return raw;
}

bool usbpd_vdm_discover_svids_contains(const uint8_t *rx_buffer, uint8_t num_data_objects, uint16_t target_id) {
    if (rx_buffer == NULL || num_data_objects < 2u) {
        return false;
    }

    uint8_t vdo_count = (uint8_t)(num_data_objects - 1u);

    for (uint8_t i = 0; i < vdo_count; i++) {
        uint32_t vdo = *(uint32_t *)&rx_buffer[6 + (i * 4)];
        uint16_t id0 = (uint16_t)(vdo & 0xFFFFu);
        uint16_t id1 = (uint16_t)((vdo >> 16) & 0xFFFFu);

        if (id0 == target_id || id1 == target_id) {
            return true;
        }

        if (id0 == 0u && id1 == 0u) {
            break;
        }
    }

    return false;
}
