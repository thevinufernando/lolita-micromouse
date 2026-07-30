#include "ICM42688.h"

extern SPI_HandleTypeDef IMU_SPI;

// ================= LOW LEVEL ===================

static void IMU_CS_Low(void)
{
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
}

static void IMU_CS_High(void)
{
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}

static uint8_t ICM42688_ReadReg(uint8_t reg)
{
    uint8_t tx = reg | 0x80;
    uint8_t rx;

    IMU_CS_Low();
    HAL_SPI_Transmit(&IMU_SPI, &tx, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&IMU_SPI, &rx, 1, HAL_MAX_DELAY);
    IMU_CS_High();

    return rx;
}

static void ICM42688_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2];
    tx[0] = reg & 0x7F;
    tx[1] = value;

    IMU_CS_Low();
    HAL_SPI_Transmit(&IMU_SPI, tx, 2, HAL_MAX_DELAY);
    IMU_CS_High();
}

// Burst read of `len` consecutive registers starting at `reg`.
// One CS-framed transaction: send the address byte, then clock out the data.
static void ICM42688_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint8_t tx = reg | 0x80;

    IMU_CS_Low();
    HAL_SPI_Transmit(&IMU_SPI, &tx, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&IMU_SPI, buf, len, HAL_MAX_DELAY);
    IMU_CS_High();
}

// =============== HIGH LEVEL ====================

int ICM42688_CheckID(void)
{
    return (ICM42688_ReadReg(ICM42688_WHO_AM_I) == ICM42688_ID) ? IMU_OK : IMU_ERROR;
}

int ICM42688_Init(void)
{
    // Soft reset FIRST, so the device is in a known state regardless of
    // whether this is a cold boot or a warm restart from the debugger.
    ICM42688_WriteReg(ICM42688_DEVICE_CONFIG, 0x01);
    HAL_Delay(100);

    // The very first SPI transaction after power-up can return garbage while
    // the device finishes its internal start-up, so allow a few attempts
    // before declaring the IMU absent.
    int found = IMU_ERROR;

    for (int attempt = 0; attempt < 5; attempt++)
    {
        if (ICM42688_ReadReg(ICM42688_WHO_AM_I) == ICM42688_ID)
        {
            found = IMU_OK;
            break;
        }

        HAL_Delay(10);
    }

    if (found != IMU_OK)
        return IMU_ERROR;

    // Enable gyro + accel in LOW NOISE mode, temp on
    // 0b00001111 = 0x0F
    ICM42688_WriteReg(ICM42688_PWR_MGMT0, 0x0F);
    HAL_Delay(50);

    // Gyro: ±2000 dps, ODR = 1 kHz
    //0b00000110 = 0x06
    ICM42688_WriteReg(ICM42688_GYRO_CONFIG0, 0x06);

    // Accel: ±4g, ODR = 1 kHz
    //0b01000110 = 0x46
    ICM42688_WriteReg(ICM42688_ACCEL_CONFIG0, 0x46);

    HAL_Delay(20);

    return IMU_OK;
}

static int16_t make_int16(uint8_t high, uint8_t low)
{
    return (int16_t)((high << 8) | low);
}

int ICM42688_ReadData(ICM42688_t *data)
{
    if (data == 0)
        return IMU_ERROR;

    // The data registers are contiguous in this order:
    //   0x1D TEMP_DATA1  0x1E TEMP_DATA0
    //   0x1F ACCEL_X1 .. 0x24 ACCEL_Z0
    //   0x25 GYRO_X1  .. 0x2A GYRO_Z0
    // Starting the burst at TEMP_DATA1 captures temperature, accel and gyro
    // in one 14-byte transaction. Starting at ACCEL_DATA_X1 instead would run
    // two bytes past the gyro into the timestamp registers.
    uint8_t rx[14];

    ICM42688_ReadRegs(ICM42688_TEMP_DATA1, rx, 14);

    int16_t temp_raw = make_int16(rx[0],  rx[1]);
    int16_t ax_raw   = make_int16(rx[2],  rx[3]);
    int16_t ay_raw   = make_int16(rx[4],  rx[5]);
    int16_t az_raw   = make_int16(rx[6],  rx[7]);
    int16_t gx_raw   = make_int16(rx[8],  rx[9]);
    int16_t gy_raw   = make_int16(rx[10], rx[11]);
    int16_t gz_raw   = make_int16(rx[12], rx[13]);

    // ===== SCALE FACTORS =====
    // Gyro ±2000 dps => 16.384 LSB/(°/s)
    data->gx = gx_raw / ICM42688_GYRO_SENS_LSB_PER_DPS;
    data->gy = gy_raw / ICM42688_GYRO_SENS_LSB_PER_DPS;
    data->gz = gz_raw / ICM42688_GYRO_SENS_LSB_PER_DPS;

    // Accel ±4g => 8192 LSB/g
    data->ax = ax_raw / ICM42688_ACCEL_SENS_LSB_PER_G;
    data->ay = ay_raw / ICM42688_ACCEL_SENS_LSB_PER_G;
    data->az = az_raw / ICM42688_ACCEL_SENS_LSB_PER_G;

    // Temperature (rough formula)
    data->temperature = (float)temp_raw / 132.48f + 25.0f;

    return IMU_OK;
}

int ICM42688_ReadGyroZ(float *gz_dps)
{
    if (gz_dps == 0)
        return IMU_ERROR;

    uint8_t rx[2];

    ICM42688_ReadRegs(ICM42688_GYRO_DATA_Z1, rx, 2);

    *gz_dps = make_int16(rx[0], rx[1]) / ICM42688_GYRO_SENS_LSB_PER_DPS;

    return IMU_OK;
}
