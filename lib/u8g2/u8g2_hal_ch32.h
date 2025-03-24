#ifndef __OLED_H
#define __OLED_H

#include "debug.h"
#include "u8g2.h"
#include "u8g2_hal_ch32_i2c.h"
#include "u8x8.h"

uint8_t u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

#endif