#pragma once

#include <stdint.h>

#define CHIP_UID_1 ((uint32_t)(*((volatile uint32_t *)(0x1FFFF7E8))))
#define CHIP_UID_2 ((uint32_t)(*((volatile uint32_t *)(0x1FFFF7EC))))
#define CHIP_UID_3 ((uint32_t)(*((volatile uint32_t *)(0x1FFFF7F0))))  // 保留位
