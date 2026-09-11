#include "TCA9548A.h"

/* CubeMX owns this handle; declared in main.c. */
extern I2C_HandleTypeDef hi2c1;

#define TCA_I2C_HANDLE   (&hi2c1)
#define TCA_I2C_TIMEOUT  10U

/* Cache of the last mask written. Selecting a channel is on the hot path —
 * every single sensor read is wrapped in one — and re-writing a mask that is
 * already active costs a full I2C transaction for nothing. At 100 kHz that is
 * ~200 us per redundant select, which matters when three sensors are polled
 * in a control loop.
 *
 * Initialised to an impossible value so the first select always writes,
 * and reset by Init/DisableAll so it can never disagree with the hardware. */
static uint8_t s_current_mask = 0xFFU;

int TCA9548A_Init(void)
{
    /* The mux is the gateway to every ToF sensor, so a missing mux and a
     * missing sensor look identical from further up. Probe it explicitly. */
    if (HAL_I2C_IsDeviceReady(TCA_I2C_HANDLE, TCA9548A_I2C_ADDR, 3,
                              TCA_I2C_TIMEOUT) != HAL_OK) {
        return TCA_ERROR;
    }

    return TCA9548A_DisableAll();
}

int TCA9548A_SelectChannel(uint8_t channel)
{
    uint8_t mask;

    if (channel >= TCA9548A_CHANNEL_COUNT) {
        return TCA_ERROR;
    }

    mask = (uint8_t)(1U << channel);

    if (mask == s_current_mask) {
        return TCA_OK;
    }

    /* No register index: the control byte is the entire payload. */
    if (HAL_I2C_Master_Transmit(TCA_I2C_HANDLE, TCA9548A_I2C_ADDR, &mask, 1,
                                TCA_I2C_TIMEOUT) != HAL_OK) {
        /* Leave the cache invalid rather than optimistically recording a mask
         * that may not have landed. */
        s_current_mask = 0xFFU;
        return TCA_ERROR;
    }

    s_current_mask = mask;
    return TCA_OK;
}

int TCA9548A_DisableAll(void)
{
    uint8_t mask = 0x00U;

    if (HAL_I2C_Master_Transmit(TCA_I2C_HANDLE, TCA9548A_I2C_ADDR, &mask, 1,
                                TCA_I2C_TIMEOUT) != HAL_OK) {
        s_current_mask = 0xFFU;
        return TCA_ERROR;
    }

    s_current_mask = mask;
    return TCA_OK;
}

int TCA9548A_ReadChannelMask(uint8_t *mask)
{
    if (mask == NULL) {
        return TCA_ERROR;
    }

    /* A plain 1-byte read returns the control register — again, no index. */
    if (HAL_I2C_Master_Receive(TCA_I2C_HANDLE, TCA9548A_I2C_ADDR, mask, 1,
                               TCA_I2C_TIMEOUT) != HAL_OK) {
        return TCA_ERROR;
    }

    return TCA_OK;
}
