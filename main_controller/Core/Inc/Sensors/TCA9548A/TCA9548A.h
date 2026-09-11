#ifndef TCA9548A_H
#define TCA9548A_H

#include "main.h"
#include <stdint.h>

/*
 * TCA9548A 8-channel I2C multiplexer.
 *
 * All five VL53L0X footprints on the Main PCB share the same factory I2C
 * address (0x29) -- three are currently populated and used. Rather than reassigning addresses at boot — which needs one
 * XSHUT GPIO per sensor and has to be redone after every power cycle — each
 * sensor hangs off its own mux channel and the MCU opens exactly one channel
 * at a time.
 *
 * The device has a single 8-bit control register with no register index: you
 * write one byte, and each bit enables the matching channel. Writing 0x00
 * disconnects everything. Multiple bits can legally be set at once, but doing
 * so would gang several identically-addressed sensors onto the bus, so
 * TCA9548A_SelectChannel() enforces one-hot selection.
 */

#define TCA_OK     0
#define TCA_ERROR -1

/* 7-bit address 0x70 with A0..A2 tied low, shifted left for the HAL's 8-bit
 * addressing. If the address pins are strapped differently on a respin, this
 * is the only line that changes. */
#define TCA9548A_I2C_ADDR_7BIT 0x70
#define TCA9548A_I2C_ADDR      (TCA9548A_I2C_ADDR_7BIT << 1)

#define TCA9548A_CHANNEL_COUNT 8U

/* Bring up the mux: verify it acknowledges, then disable all channels so the
 * bus starts from a known state. Returns TCA_OK / TCA_ERROR. */
int TCA9548A_Init(void);

/* Open exactly one channel (0..7), closing all others.
 * Returns TCA_ERROR on an out-of-range channel or a bus failure. */
int TCA9548A_SelectChannel(uint8_t channel);

/* Disconnect every channel. Useful to park the bus before touching another
 * device, and to make a mis-sequenced access fail loudly instead of silently
 * hitting whichever sensor happened to still be connected. */
int TCA9548A_DisableAll(void);

/* Read back the control register. Mainly a bring-up aid: confirms the mux is
 * alive and reports which channel is actually open. */
int TCA9548A_ReadChannelMask(uint8_t *mask);

#endif /* TCA9548A_H */
