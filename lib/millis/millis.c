#include "millis.h"

#include "ch32x035.h"

static volatile uint32_t _millis = 0;

static void TIM_Init(uint16_t arr, uint16_t psc) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = {0};

    NVIC_InitTypeDef NVIC_InitStructure = {0};

    /* Enable timer1 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    /* Initialize timer1 */
    TIM_TimeBaseStructure.TIM_Period = arr;
    TIM_TimeBaseStructure.TIM_Prescaler = psc;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    /* Enable updating timer1 interrupt */
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

    /* Configure timer1 interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable timer1 */
    TIM_Cmd(TIM1, ENABLE);

    /* Enable timer1 interrupt */
    NVIC_EnableIRQ(TIM1_UP_IRQn);
}

void millis_init(void) {
    // 初始化定时器
    TIM_Init(1000 - 1, 48 - 1);
}

void TIM1_UP_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM1_UP_IRQHandler(void) {
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
        _millis++;
    }
}

uint32_t millis(void) {
    return _millis;
}
