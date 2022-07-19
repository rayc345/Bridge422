#ifndef __SWD_PIN_H__
#define __SWD_PIN_H__

#include <stdint.h>
#include "main.h"

__STATIC_FORCEINLINE void SWDIO_SET_OUTPUT(void)
{
    LL_GPIO_SetPinMode(SWDIO_GPIO_Port, SWDIO_Pin, LL_GPIO_MODE_OUTPUT);
}

__STATIC_FORCEINLINE void SWDIO_SET_INPUT(void)
{
    LL_GPIO_SetPinMode(SWDIO_GPIO_Port, SWDIO_Pin, LL_GPIO_MODE_INPUT);
}

__STATIC_FORCEINLINE void SWCLK_SET(void)
{
    SWCLK_GPIO_Port->BSRR = SWCLK_Pin;
}

__STATIC_FORCEINLINE void SWCLK_CLR(void)
{
    SWCLK_GPIO_Port->BSRR = (uint32_t)SWCLK_Pin << 16;
}

__STATIC_FORCEINLINE uint8_t SWDIO_IN(void)
{
    return (SWDIO_GPIO_Port->IDR & SWDIO_Pin) ? 1 : 0;
}

__STATIC_FORCEINLINE void SWDIO_SET(void)
{
    SWDIO_GPIO_Port->BSRR = SWDIO_Pin;
}

__STATIC_FORCEINLINE void SWDIO_CLR(void)
{
    SWDIO_GPIO_Port->BSRR = (uint32_t)SWDIO_Pin << 16;
}

__STATIC_FORCEINLINE void SWDIO_OUT(const uint8_t bit)
{
    if (bit & 0x01)
        SWDIO_SET();
    else
        SWDIO_CLR();
}

__STATIC_FORCEINLINE void PIN_DELAY(void)
{
    TIM6->CNT = 0;
    while (TIM6->CNT <= 5)
    {
    }
    // __ASM volatile(
    //     "MOV r0, r0\n"
    //     "MOV r0, r0\n");
}

__STATIC_FORCEINLINE uint32_t SW_READ_BIT(void)
{
    // uint32_t bit;
    // SWCLK_CLR();
    // PIN_DELAY();
    // bit = SWDIO_IN();
    // SWCLK_SET();
    // PIN_DELAY();
    // return bit;

    PIN_DELAY();
    uint32_t bit = SWDIO_IN();
    SWCLK_SET();
    PIN_DELAY();
    SWCLK_CLR();
    return bit;
}

__STATIC_FORCEINLINE void SW_CLOCK_CYCLE(void)
{
    // SWCLK_CLR();
    // PIN_DELAY();
    // SWCLK_SET();
    // PIN_DELAY();

    PIN_DELAY();
    SWCLK_SET();
    PIN_DELAY();
    SWCLK_CLR();
}

__STATIC_FORCEINLINE void SW_WRITE_BIT(uint8_t bit)
{
    SWDIO_OUT(bit);
    SW_CLOCK_CYCLE();
}

// __STATIC_FORCEINLINE void PIN_DELAY_ONE(void)
// {
//     TIM6->CNT = 0;
//     while (TIM6->CNT <= 4)
//     {
//     }
//     // __ASM volatile(
//     //     "MOV r0, r0\n"
//     //     "MOV r0, r0\n");
// }

// __STATIC_FORCEINLINE void PIN_DELAY_TWO(void)
// {
//     TIM6->CNT = 0;
//     while (TIM6->CNT <= 8)
//     {
//     }
// }

// __STATIC_FORCEINLINE void SW_CLOCK_CYCLE(void)
// {
//     PIN_DELAY_ONE();
//     SWCLK_SET();
//     PIN_DELAY_TWO();
//     SWCLK_CLR();
//     PIN_DELAY_ONE();
// }

// __STATIC_FORCEINLINE uint8_t SW_READ_BIT(void)
// {
//     uint8_t bit = SWDIO_IN();
//     SW_CLOCK_CYCLE();
//     return bit;
// }

// __STATIC_FORCEINLINE void SW_WRITE_BIT(const uint8_t bit)
// {
//     SWDIO_OUT(bit);
//     SW_CLOCK_CYCLE();
// }

#endif
