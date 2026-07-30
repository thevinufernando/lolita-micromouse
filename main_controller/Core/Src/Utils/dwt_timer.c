#include "dwt_timer.h"

//Cycles per microsecond, derived from HCLK at init time.
static uint32_t cycles_per_us = 1U;

void DWT_Timer_Init(void)
{
    //Enable the trace subsystem that clocks the DWT block
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    //Reset and start the cycle counter
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    //SystemCoreClock tracks HCLK, which is what the DWT counts
    cycles_per_us = SystemCoreClock / 1000000U;

    if (cycles_per_us == 0U) {
        cycles_per_us = 1U;
    }
}

uint32_t DWT_GetCycles(void)
{
    return DWT->CYCCNT;
}

uint32_t DWT_ElapsedUs(uint32_t start_cycles)
{
    //Unsigned wraparound makes this correct across a single counter wrap
    uint32_t elapsed_cycles = DWT->CYCCNT - start_cycles;

    return elapsed_cycles / cycles_per_us;
}

uint32_t DWT_Micros(void)
{
    return DWT->CYCCNT / cycles_per_us;
}

void DWT_DelayUs(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t target = us * cycles_per_us;

    while ((DWT->CYCCNT - start) < target) {
        //busy wait
    }
}
