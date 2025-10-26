#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "usbpd_def.h"

/**
 * @brief 构建 Fixed Supply RDO
 * @param rdo 输出的 RDO 结构
 * @param position PDO Position
 * @param voltage_mv 电压 (mV)
 * @param current_ma 电流 (mA)
 */
void usbpd_rdo_build_fixed(USBPD_RDO_t *rdo, uint8_t position, uint16_t voltage_mv, uint16_t current_ma);

/**
 * @brief 构建 PPS RDO
 * @param rdo 输出的 RDO 结构
 * @param position PDO Position
 * @param voltage_mv 请求电压 (mV)
 * @param current_ma 最大电流 (mA)
 */
void usbpd_rdo_build_pps(USBPD_RDO_t *rdo, uint8_t position, uint16_t voltage_mv, uint16_t current_ma);

/**
 * @brief 构建 SPR/EPR AVS RDO
 * @param rdo 输出的 RDO 结构
 * @param position PDO Position
 * @param voltage_mv 请求电压 (mV)
 * @param current_ma 电流 (mA)
 */
void usbpd_rdo_build_avs(USBPD_RDO_t *rdo, uint8_t position, uint16_t voltage_mv, uint16_t current_ma);

/**
 * @brief 格式化 SPR 模式请求 (USBPD_DATA_MSG_REQUEST)
 * @param buffer 发送缓冲区（至少 6 字节）
 * @param rdo RDO 数据
 * @param message_id 消息 ID
 * @param pd_version PD 版本
 * @return 发送数据长度（字节）
 */
uint8_t usbpd_rdo_format_spr_request(uint8_t *buffer, const USBPD_RDO_t *rdo, uint8_t message_id, USBPD_SpecificationRevision_t pd_version);

/**
 * @brief 格式化 EPR 模式请求 (USBPD_DATA_MSG_EPR_REQUEST)
 * @param buffer 发送缓冲区（至少 10 字节）
 * @param rdo RDO 数据
 * @param pdo_raw PDO 原始数据（副本）
 * @param message_id 消息 ID
 * @return 发送数据长度（字节）
 */
uint8_t usbpd_rdo_format_epr_request(uint8_t *buffer, const USBPD_RDO_t *rdo, uint32_t pdo_raw, uint8_t message_id);