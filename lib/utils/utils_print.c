#include "utils_print.h"

#include <stddef.h>

#include "ch32x035.h"

typedef struct {
    USART_TypeDef *usart;
    GPIO_TypeDef *gpio;
    uint16_t pin;
    uint32_t usart_rcc;
    uint32_t gpio_rcc;
    uint8_t is_apb1;  // 1 for APB1, 0 for APB2
} uart_config_t;

#if (DEBUG >= PRINT_UART1 && DEBUG <= PRINT_UART3)
static const uart_config_t uart_configs[] = {
    {USART1, GPIOB, GPIO_Pin_10, RCC_APB2Periph_USART1, RCC_APB2Periph_GPIOB, 0},
    {USART2, GPIOA, GPIO_Pin_2, RCC_APB1Periph_USART2, RCC_APB2Periph_GPIOA, 1},
    {USART3, GPIOB, GPIO_Pin_3, RCC_APB1Periph_USART3, RCC_APB2Periph_GPIOB, 1},
};
#endif

void print_init(uint32_t baudrate) {
#if (DEBUG == PRINT_SDI)
    *(DEBUG_DATA0_ADDRESS) = 0;
#elif (DEBUG >= PRINT_UART1 && DEBUG <= PRINT_UART3)
    const uart_config_t *cfg = &uart_configs[DEBUG - 1];

    // 启用时钟
    if (cfg->is_apb1) {
        RCC_APB1PeriphClockCmd(cfg->usart_rcc, ENABLE);
    } else {
        RCC_APB2PeriphClockCmd(cfg->usart_rcc, ENABLE);
    }
    RCC_APB2PeriphClockCmd(cfg->gpio_rcc, ENABLE);

    // GPIO 配置
    GPIO_InitTypeDef gpio_init = {
        .GPIO_Pin = cfg->pin,
        .GPIO_Speed = GPIO_Speed_50MHz,
        .GPIO_Mode = GPIO_Mode_AF_PP,
    };
    GPIO_Init(cfg->gpio, &gpio_init);

    // USART 配置
    USART_InitTypeDef usart_init = {
        .USART_BaudRate = baudrate,
        .USART_WordLength = USART_WordLength_8b,
        .USART_StopBits = USART_StopBits_1,
        .USART_Parity = USART_Parity_No,
        .USART_HardwareFlowControl = USART_HardwareFlowControl_None,
        .USART_Mode = USART_Mode_Tx,
    };

    USART_Init(cfg->usart, &usart_init);
    USART_Cmd(cfg->usart, ENABLE);
#endif
}

/*********************************************************************
 * @fn      _write
 *
 * @brief   Support Printf Function
 *
 * @param   *buf - UART send Data.
 *          size - Data length
 *
 * @return  size - Data length
 */
__attribute__((used)) int _write(int fd, char *buf, int size) {
#if (DEBUG == PRINT_SDI)
    // SDI 输出
    int i = 0, writeSize = size;
    do {
        while (*(DEBUG_DATA0_ADDRESS) != 0u);

        int chunk = (writeSize > 7) ? 7 : writeSize;
        *(DEBUG_DATA1_ADDRESS) = (*(buf + i + 3)) | (*(buf + i + 4) << 8) | (*(buf + i + 5) << 16) | (*(buf + i + 6) << 24);
        *(DEBUG_DATA0_ADDRESS) = chunk | (*(buf + i) << 8) | (*(buf + i + 1) << 16) | (*(buf + i + 2) << 24);

        i += chunk;
        writeSize -= chunk;
    } while (writeSize);
#elif (DEBUG >= PRINT_UART1 && DEBUG <= PRINT_UART3)
    // UART 输出
    static USART_TypeDef *uart_map[] = {NULL, USART1, USART2, USART3};
    USART_TypeDef *uart = uart_map[DEBUG];
    for (int i = 0; i < size; i++) {
        while (USART_GetFlagStatus(uart, USART_FLAG_TC) == RESET);
        USART_SendData(uart, *buf++);
    }
#endif
    return size;
}

/*********************************************************************
 * @fn      _sbrk
 *
 * @brief   Change the spatial position of data segment.
 *
 * @return  size - Data length
 */
__attribute__((used)) void *_sbrk(ptrdiff_t incr) {
    extern char _end[];
    extern char _heap_end[];
    static char *curbrk = _end;

    if ((curbrk + incr < _end) || (curbrk + incr > _heap_end)) return NULL - 1;

    curbrk += incr;
    return curbrk - incr;
}