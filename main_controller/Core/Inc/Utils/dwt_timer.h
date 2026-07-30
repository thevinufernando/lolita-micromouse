#ifndef DWT_TIMER_H
#define DWT_TIMER_H

#include "main.h"
#include <stdint.h>

/*
 * Microsecond timebase built on the Cortex-M4 DWT cycle counter.
 *
 * HAL_GetTick() only resolves to 1 ms, which is too coarse for integrating
 * gyro rates: at 1 kHz the whole sample interval is one tick. The DWT
 * counter runs at the CPU clock (HCLK, 48 MHz here) and gives ~21 ns
 * resolution.
 *
 * The 32-bit cycle counter wraps every 2^32 / 48e6 = ~89.5 s. Unsigned
 * subtraction of two cycle stamps stays correct across a single wrap, so
 * always measure elapsed time with DWT_ElapsedUs() rather than comparing
 * absolute values from DWT_Micros().
 */

//Enable the cycle counter. Call once after SystemClock_Config().
void DWT_Timer_Init(void);

//Raw free-running CPU cycle count.
uint32_t DWT_GetCycles(void);

//Microseconds elapsed since the cycle stamp `start_cycles`.
//Wrap-safe for intervals shorter than ~89 s.
uint32_t DWT_ElapsedUs(uint32_t start_cycles);

//Free-running microsecond counter. Wraps every ~89.5 s.
uint32_t DWT_Micros(void);

//Busy-wait delay. Intended for short waits only.
void DWT_DelayUs(uint32_t us);

#endif /* DWT_TIMER_H */
