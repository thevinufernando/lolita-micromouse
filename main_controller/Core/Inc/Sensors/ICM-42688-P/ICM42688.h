#ifndef ICM42688_H
#define ICM42688_H

#include "main.h"
#include <stdint.h>

// ================= USER CONFIG =================
#define IMU_SPI              hspi1
#define IMU_CS_GPIO_Port     GPIOA
#define IMU_CS_Pin           GPIO_PIN_4
// ===============================================

#define IMU_OK               0
#define IMU_ERROR           -1

// Register Map
#define ICM42688_DEVICE_CONFIG   0x11
#define ICM42688_WHO_AM_I        0x75
#define ICM42688_PWR_MGMT0       0x4E
#define ICM42688_GYRO_CONFIG0    0x4F
#define ICM42688_ACCEL_CONFIG0   0x50

#define ICM42688_TEMP_DATA1      0x1D
#define ICM42688_GYRO_DATA1      0x25
#define ICM42688_ACCEL_DATA_X1   0x1F
#define ICM42688_GYRO_DATA_Z1    0x29
#define ICM42688_INT_STATUS      0x2D

// WHO_AM_I expected value
#define ICM42688_ID              0x47

// Scale factors for the configured full-scale ranges.
// Gyro  +/-2000 dps => 16.384 LSB/(deg/s)
// Accel +/-4g       => 8192 LSB/g
#define ICM42688_GYRO_SENS_LSB_PER_DPS   16.384f
#define ICM42688_ACCEL_SENS_LSB_PER_G    8192.0f

typedef struct
{
    float ax;   // g
    float ay;
    float az;

    float gx;   // deg/s
    float gy;
    float gz;

    float temperature;
} ICM42688_t;

// Public API
int ICM42688_Init(void);
int ICM42688_ReadData(ICM42688_t *data);

// Verify the WHO_AM_I response. Returns IMU_OK when the device answers.
int ICM42688_CheckID(void);

// Read only the Z-axis gyro, in deg/s. Two data bytes instead of fourteen,
// which matters when this is polled at 1 kHz to drive the EKF prediction.
int ICM42688_ReadGyroZ(float *gz_dps);

#endif
