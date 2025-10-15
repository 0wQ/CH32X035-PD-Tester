#include "u8g2_hal_ch32.h"

#include "ch32x035.h"

static void _i2c_init(uint32_t bound, uint16_t address) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    I2C_InitTypeDef I2C_InitTSturcture = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    GPIO_PinRemapConfig(GPIO_FullRemap_I2C1, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_18 | GPIO_Pin_19;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    I2C_InitTSturcture.I2C_ClockSpeed = bound;
    I2C_InitTSturcture.I2C_Mode = I2C_Mode_I2C;
    I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitTSturcture.I2C_OwnAddress1 = address;
    I2C_InitTSturcture.I2C_Ack = I2C_Ack_Enable;
    I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &I2C_InitTSturcture);

    I2C_Cmd(I2C1, ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
}

uint8_t u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    /* u8g2/u8x8 will never send more than 32 bytes between START_TRANSFER and END_TRANSFER */
    uint8_t *data = (uint8_t *)arg_ptr;

    switch (msg) {
        case U8X8_MSG_BYTE_INIT:
            _i2c_init(400 * 1000, 0x78);  // I2C 初始化
            break;

        case U8X8_MSG_BYTE_START_TRANSFER:
            while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) != RESET);
            I2C_GenerateSTART(I2C1, ENABLE);
            while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
            I2C_Send7bitAddress(I2C1, 0x78, I2C_Direction_Transmitter);
            while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
            break;

        case U8X8_MSG_BYTE_SEND:
            while (arg_int-- > 0) {
                I2C_SendData(I2C1, *data++);
                while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
            }
            break;

        case U8X8_MSG_BYTE_END_TRANSFER:
            I2C_GenerateSTOP(I2C1, ENABLE);
            break;

        case U8X8_MSG_BYTE_SET_DC:
            break;

        default:
            return 0;
    }

    return 1;
}

uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    return 1;
}