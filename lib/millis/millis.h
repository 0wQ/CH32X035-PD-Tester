#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// 初始化毫秒计时器
void millis_init(void);

// 获取自启动以来的毫秒数
uint32_t millis(void);

#ifdef __cplusplus
}
#endif