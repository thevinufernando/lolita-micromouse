/*
 * vl53l0x_platform.c — STM32 HAL port of the ST VL53L0X PAL platform layer.
 *
 * ST ships this file as a Win32/serial reference implementation that talks to
 * a Nucleo over a COM port via ranging_sensor_comms.dll. That cannot build for
 * this target, so the original was replaced wholesale with a direct
 * STM32 HAL I2C implementation. Everything else under Sensors/VL53L0X/ is
 * stock ST API (VL53L0X_1.0.4) and should stay untouched, so that a future API
 * update is a drop-in replacement of Core/ and a re-port of only this file.
 *
 * The companion Win32-only sources from the same package
 * (vl53l0x_i2c_platform.c, vl53l0x_i2c_win_serial_comms.c) are deliberately
 * NOT in CMakeLists.txt — the functions they declare are bypassed here.
 *
 * ---------------------------------------------------------------------------
 * MUX NOTE
 * ---------------------------------------------------------------------------
 * All three ToF sensors sit behind a TCA9548A and therefore share the same
 * I2C address. This layer does NOT touch the mux — it just transacts on
 * whichever channel is currently open. Selecting the channel is the caller's
 * job (see tof_sensors.c, which wraps every API call in a channel select).
 *
 * ---------------------------------------------------------------------------
 * ADDRESSING
 * ---------------------------------------------------------------------------
 * Dev->I2cDevAddr holds the ST convention 8-bit address (0x52). The HAL also
 * wants an 8-bit left-aligned address, so it is passed through unshifted.
 * Do not "fix" this to 0x29 << 1 — that is the same number, and the API's own
 * VL53L0X_SetDeviceAddress() writes Dev->I2cDevAddr in 8-bit form.
 */

#include "vl53l0x_platform.h"
#include "vl53l0x_api.h"
#include "main.h"

/* CubeMX owns this handle; declared in main.c. */
extern I2C_HandleTypeDef hi2c1;

#define VL53L0X_I2C_HANDLE      (&hi2c1)

/* Per-transfer timeout. The API's longest single transaction is a 12-byte
 * burst; at 100 kHz that is well under a millisecond, so this is purely a
 * bus-lockup escape hatch, not a functional limit. */
#define VL53L0X_I2C_TIMEOUT_MS  10U

/* Largest burst the ST API issues in one call is 12 bytes (SPAD map). The
 * +1 is the register index that gets prepended for the write path. */
#define VL53L0X_MAX_I2C_XFER_SIZE 64

VL53L0X_Error VL53L0X_LockSequenceAccess(VL53L0X_DEV Dev)
{
    (void)Dev;
    /* Single-threaded bare metal: nothing to serialise against. */
    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_UnlockSequenceAccess(VL53L0X_DEV Dev)
{
    (void)Dev;
    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata,
                                 uint32_t count)
{
    /* The register index and payload must go out as ONE transaction with no
     * repeated start, so they are staged into a single buffer rather than
     * issued as two HAL calls. */
    uint8_t buffer[VL53L0X_MAX_I2C_XFER_SIZE + 1];

    if (count > VL53L0X_MAX_I2C_XFER_SIZE) {
        return VL53L0X_ERROR_INVALID_PARAMS;
    }

    buffer[0] = index;
    for (uint32_t i = 0; i < count; i++) {
        buffer[i + 1] = pdata[i];
    }

    if (HAL_I2C_Master_Transmit(VL53L0X_I2C_HANDLE, (uint16_t)Dev->I2cDevAddr,
                                buffer, (uint16_t)(count + 1),
                                VL53L0X_I2C_TIMEOUT_MS) != HAL_OK) {
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata,
                                uint32_t count)
{
    if (count > VL53L0X_MAX_I2C_XFER_SIZE) {
        return VL53L0X_ERROR_INVALID_PARAMS;
    }

    /* Write the index, then read back. HAL_I2C_Mem_Read does exactly this
     * (write index, repeated start, read) which is what the device expects. */
    if (HAL_I2C_Mem_Read(VL53L0X_I2C_HANDLE, (uint16_t)Dev->I2cDevAddr,
                         (uint16_t)index, I2C_MEMADD_SIZE_8BIT,
                         pdata, (uint16_t)count,
                         VL53L0X_I2C_TIMEOUT_MS) != HAL_OK) {
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data)
{
    return VL53L0X_WriteMulti(Dev, index, &data, 1);
}

VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data)
{
    /* The VL53L0X is big-endian on the wire; the MCU is little-endian, so
     * every multi-byte value is packed MSB-first by hand. */
    uint8_t buffer[2];

    buffer[0] = (uint8_t)(data >> 8);
    buffer[1] = (uint8_t)(data & 0x00FF);

    return VL53L0X_WriteMulti(Dev, index, buffer, 2);
}

VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data)
{
    uint8_t buffer[4];

    buffer[0] = (uint8_t)(data >> 24);
    buffer[1] = (uint8_t)((data & 0x00FF0000) >> 16);
    buffer[2] = (uint8_t)((data & 0x0000FF00) >> 8);
    buffer[3] = (uint8_t)(data & 0x000000FF);

    return VL53L0X_WriteMulti(Dev, index, buffer, 4);
}

VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data)
{
    return VL53L0X_ReadMulti(Dev, index, data, 1);
}

VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data)
{
    uint8_t buffer[2];
    VL53L0X_Error status = VL53L0X_ReadMulti(Dev, index, buffer, 2);

    if (status == VL53L0X_ERROR_NONE) {
        *data = (uint16_t)(((uint16_t)buffer[0] << 8) + (uint16_t)buffer[1]);
    }

    return status;
}

VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data)
{
    uint8_t buffer[4];
    VL53L0X_Error status = VL53L0X_ReadMulti(Dev, index, buffer, 4);

    if (status == VL53L0X_ERROR_NONE) {
        *data = ((uint32_t)buffer[0] << 24) + ((uint32_t)buffer[1] << 16) +
                ((uint32_t)buffer[2] << 8) + (uint32_t)buffer[3];
    }

    return status;
}

VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData,
                                 uint8_t OrData)
{
    uint8_t data;
    VL53L0X_Error status = VL53L0X_RdByte(Dev, index, &data);

    if (status == VL53L0X_ERROR_NONE) {
        data = (uint8_t)((data & AndData) | OrData);
        status = VL53L0X_WrByte(Dev, index, data);
    }

    return status;
}

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev)
{
    (void)Dev;
    /* Called from the API's internal poll loops (measurement-ready, stop
     * complete). 1 ms matches ST's reference cadence — the sensor's fastest
     * timing budget is ~20 ms, so polling faster only burns bus bandwidth. */
    HAL_Delay(1);
    return VL53L0X_ERROR_NONE;
}
