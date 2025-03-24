#include "u8g2_hal_ch32.h"

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

// 提供给软件模拟 I2C 的 GPIO 输出和延时，使用之前编写的配置函数
uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    switch (msg) {
        case U8X8_MSG_GPIO_AND_DELAY_INIT:  // called once during init phase of u8g2/u8x8
            // Delay_Init();
            break;  // can be used to setup pins

        case U8X8_MSG_DELAY_100NANO:  // delay arg_int * 100 nano seconds
            __NOP();
            break;

        case U8X8_MSG_DELAY_10MICRO:  // delay arg_int * 10 micro seconds
            for (uint16_t n = 0; n < 320; n++) {
                __NOP();
            }
            break;

        case U8X8_MSG_DELAY_MILLI:  // delay arg_int * 1 milli second
            Delay_Ms(1);
            break;

        case U8X8_MSG_DELAY_I2C:  // arg_int is the I2C speed in 100KHz, e.g. 4 = 400 KHz
            Delay_Us(1);
            break;  // arg_int=1: delay by 5us, arg_int = 4: delay by 1.25us

        case U8X8_MSG_GPIO_I2C_CLOCK:  // arg_int=0: Output low at I2C clock pin
                                       //            arg_int ? GPIO_SetBits(GPIOB, GPIO_Pin_6) : GPIO_ResetBits(GPIOB, GPIO_Pin_6);
            break;                     // arg_int=1: Input dir with pullup high for I2C clock pin

        case U8X8_MSG_GPIO_I2C_DATA:  // arg_int=0: Output low at I2C data pin
                                      //            arg_int ? GPIO_SetBits(GPIOB, GPIO_Pin_7) : GPIO_ResetBits(GPIOB, GPIO_Pin_7);
            break;                    // arg_int=1: Input dir with pullup high for I2C data pin

        case U8X8_MSG_GPIO_MENU_SELECT:
            u8x8_SetGPIOResult(u8x8, /* get menu select pin state */ 0);
            break;

        case U8X8_MSG_GPIO_MENU_NEXT:
            u8x8_SetGPIOResult(u8x8, /* get menu next pin state */ 0);
            break;

        case U8X8_MSG_GPIO_MENU_PREV:
            u8x8_SetGPIOResult(u8x8, /* get menu prev pin state */ 0);
            break;

        case U8X8_MSG_GPIO_MENU_HOME:
            u8x8_SetGPIOResult(u8x8, /* get menu home pin state */ 0);
            break;

        default:
            u8x8_SetGPIOResult(u8x8, 1);  // default return value
            break;
    }
    return 1;
}