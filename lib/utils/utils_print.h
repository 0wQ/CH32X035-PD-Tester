#pragma once

#include <stdint.h>
#include <stdio.h>

#define PRINT_DISABLE -1
#define PRINT_SDI     0
#define PRINT_UART1   1
#define PRINT_UART2   2
#define PRINT_UART3   3

#ifndef DEBUG
#define DEBUG PRINT_DISABLE
#endif

#if (DEBUG == PRINT_DISABLE)
#define LOG(X...)
#else
#define LOG(format, ...) printf(format, ##__VA_ARGS__)
#endif

#define DEBUG_DATA0_ADDRESS ((volatile uint32_t *)0xE0000380)
#define DEBUG_DATA1_ADDRESS ((volatile uint32_t *)0xE0000384)

void print_init(uint32_t baudrate);