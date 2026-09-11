#include "tof_sensors.h"
#include "tof_filter.h"
#include "TCA9548A.h"
#include "control_config.h"
#include "vl53l0x_api.h"

/* One PAL device context per sensor. The ST API keeps a substantial amount of
 * per-device calibration state in here (SPAD map, offsets, sequence config),
 * which is exactly why each sensor needs its own: they are physically
 * different parts and their reference calibrations are not interchangeable. */
static VL53L0X_Dev_t s_dev[TOF_SENSOR_COUNT];

/* Mux channel per sensor. Index order must match ToF_Sensor_t. */
static const uint8_t s_channel[TOF_SENSOR_COUNT] = {
    TOF_CHANNEL_FRONT,
    TOF_CHANNEL_LEFT,
    TOF_CHANNEL_RIGHT,
};

/* Init succeeded for this sensor. A failed sensor is skipped by every
 * subsequent call rather than being retried, so one dead sensor cannot stall
 * a control loop with repeated I2C timeouts. */
static uint8_t s_ready[TOF_SENSOR_COUNT];

/* Sensor is currently free-running. Determines whether ToF_ReadAll() polls or
 * triggers, and guards against starting continuous mode twice. */
static uint8_t s_continuous[TOF_SENSOR_COUNT];

/* Noise filter state, one per sensor. Kept here rather than inside the filter
 * module so the filter stays a pure, host-testable transform with no global
 * state of its own. */
static ToF_Filter_t s_filter[TOF_SENSOR_COUNT];

/* Per-sensor bias correction, mm, added to the raw reading. Index order must
 * match ToF_Sensor_t. Signed: a sensor that reads long needs a negative
 * offset. */
static const int16_t s_offset_mm[TOF_SENSOR_COUNT] = {
    TOF_OFFSET_FRONT_MM,
    TOF_OFFSET_LEFT_MM,
    TOF_OFFSET_RIGHT_MM,
};

/* Apply a sensor's offset, clamped to a sane range.
 *
 * Clamping at 0 matters: a negative offset larger than a short reading would
 * underflow uint16_t into a huge positive distance -- the robot would believe
 * a wall pressed against its bumper was metres away. Clamping below
 * TOF_DISTANCE_INVALID keeps a corrected value from colliding with the
 * sentinel. */
static uint16_t ToF_ApplyOffset(ToF_Sensor_t sensor, uint16_t raw_mm)
{
    int32_t corrected = (int32_t)raw_mm + (int32_t)s_offset_mm[sensor];

    if (corrected < 0) {
        corrected = 0;
    }

    if (corrected >= (int32_t)TOF_DISTANCE_INVALID) {
        corrected = (int32_t)TOF_DISTANCE_INVALID - 1;
    }

    return (uint16_t)corrected;
}

/* Convert a float in MCPS/mm to the API's FixPoint1616 format. */
#define TOF_FP1616(x) ((FixPoint1616_t)((x) * 65536.0f))

/* Open the sensor's mux channel. Every entry point goes through this -- an
 * API call made on the wrong channel would silently talk to a different
 * sensor at the same address. */
static int ToF_SelectSensor(ToF_Sensor_t sensor)
{
    if (sensor >= TOF_SENSOR_COUNT) {
        return TOF_ERROR;
    }

    if (TCA9548A_SelectChannel(s_channel[sensor]) != TCA_OK) {
        return TOF_ERROR;
    }

    return TOF_OK;
}

/* Mark a measurement invalid. Centralised so no path can forget to clear a
 * stale distance on failure. */
static void ToF_InvalidateMeasurement(ToF_Measurement_t *out)
{
    if (out != NULL) {
        out->distance_mm = TOF_DISTANCE_INVALID;
        out->raw_mm = TOF_DISTANCE_INVALID;
        out->range_status = 255U;
        out->valid = 0U;
    }
}

/* Decode an ST measurement struct into ours.
 *
 * RangeStatus 0 is the only status ST considers a good measurement; anything
 * else (sigma too high, signal too weak, phase fail, out of bounds) means the
 * transaction worked but the number is not usable. That distinction is worth
 * preserving: a bus fault needs investigating, an out-of-range reading just
 * means there is no wall there. */
static int ToF_DecodeMeasurement(ToF_Sensor_t sensor,
                                 const VL53L0X_RangingMeasurementData_t *data,
                                 ToF_Measurement_t *out)
{
    uint16_t corrected;
    uint16_t filtered;

    if (data->RangeStatus != 0U) {
        /* Break filter continuity rather than letting the next good sample
         * blend with one from before an unknown-length gap. */
        ToF_Filter_Invalidate(&s_filter[sensor]);

        if (out != NULL) {
            out->distance_mm = TOF_DISTANCE_INVALID;
            out->raw_mm = TOF_DISTANCE_INVALID;
            out->range_status = data->RangeStatus;
            out->valid = 0U;
        }
        return TOF_ERROR_RANGE;
    }

    /* Bias first, then noise: the filter should smooth an already-centred
     * signal. Filtering first and offsetting after would give the same mean
     * here, but it would mean the jump detector compares uncorrected values
     * against a threshold chosen for corrected ones. */
    corrected = ToF_ApplyOffset(sensor, data->RangeMilliMeter);
    filtered = ToF_Filter_Update(&s_filter[sensor], corrected);

    if (out != NULL) {
        out->distance_mm = filtered;
        out->raw_mm = corrected;
        out->range_status = data->RangeStatus;
        out->valid = 1U;
    }

    return TOF_OK;
}

/* Full ST bring-up sequence for one sensor, on an already-selected channel.
 *
 * The order below is mandated by the API and is not rearrangeable:
 * DataInit must precede StaticInit, and both must precede the reference
 * calibrations. See UM2039 and ST's own SingleRanging example. */
static int ToF_InitSensor(ToF_Sensor_t sensor)
{
    VL53L0X_Dev_t *dev = &s_dev[sensor];
    VL53L0X_Error status;
    uint32_t ref_spad_count;
    uint8_t is_aperture_spads;
    uint8_t vhv_settings;
    uint8_t phase_cal;

    dev->I2cDevAddr = TOF_I2C_ADDR_DEFAULT;
    dev->comms_type = 1; /* I2C */
    dev->comms_speed_khz = 100;

    /* Device data init -- resets the API's view of the part and reads factory
     * NVM settings out of the sensor. */
    status = VL53L0X_DataInit(dev);

    /* Loads the tuning settings and initialises the ranging sequence. */
    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_StaticInit(dev);
    }

    /* Reference SPAD management: works out which SPADs to use for this
     * particular part and cover glass. Must run before ref calibration. */
    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_PerformRefSpadManagement(dev, &ref_spad_count,
                                                  &is_aperture_spads);
    }

    /* VHV + phase reference calibration. */
    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_PerformRefCalibration(dev, &vhv_settings, &phase_cal);
    }

    /* Default to single ranging; ToF_StartContinuous() switches modes later. */
    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_SetDeviceMode(dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    }

    /* ---- Range profile ----
     * Enable the sigma and signal-rate limit checks so the sensor rejects
     * imprecise and weak readings itself, then set the thresholds. Without
     * these enabled the limit VALUES are stored but never applied. */
    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_SetLimitCheckEnable(
            dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    }

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_SetLimitCheckEnable(
            dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    }

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_SetLimitCheckValue(
            dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
            TOF_FP1616(TOF_SIGMA_LIMIT_MM));
    }

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_SetLimitCheckValue(
            dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
            TOF_FP1616(TOF_SIGNAL_RATE_LIMIT_MCPS));
    }

    /* VCSEL periods must be set before the timing budget: changing them
     * alters how long a measurement takes, and the API recomputes the budget
     * against the current periods. Setting the budget first would have it
     * silently readjusted. */
    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_SetVcselPulsePeriod(
            dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, TOF_VCSEL_PERIOD_PRE_RANGE);
    }

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_SetVcselPulsePeriod(
            dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE,
            TOF_VCSEL_PERIOD_FINAL_RANGE);
    }

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(
            dev, TOF_TIMING_BUDGET_US);
    }

    return (status == VL53L0X_ERROR_NONE) ? TOF_OK : TOF_ERROR;
}

int ToF_Init(void)
{
    int result = TOF_OK;

    for (uint8_t i = 0; i < TOF_SENSOR_COUNT; i++) {
        s_ready[i] = 0U;
        s_continuous[i] = 0U;
        ToF_Filter_Reset(&s_filter[i]);
    }

    /* No mux, no sensors -- fail early rather than emitting three identical
     * per-sensor timeouts. */
    if (TCA9548A_Init() != TCA_OK) {
        return TOF_ERROR;
    }

    for (uint8_t i = 0; i < TOF_SENSOR_COUNT; i++) {
        ToF_Sensor_t sensor = (ToF_Sensor_t)i;

        if (ToF_SelectSensor(sensor) != TOF_OK) {
            result = TOF_ERROR;
            continue;
        }

        /* The datasheet allows up to 1.2 ms from power-up to first valid I2C
         * transaction. Boot is long past by now, but a channel switch is the
         * sensor's first activity on the bus, so give it a moment. */
        HAL_Delay(2);

        if (ToF_InitSensor(sensor) != TOF_OK) {
            /* Carry on with the rest: a partial sensor set is still useful,
             * and the caller can find out which via ToF_IsSensorReady(). */
            result = TOF_ERROR;
            continue;
        }

        s_ready[i] = 1U;
    }

    /* Park the bus so a stray transaction cannot land on a sensor. */
    (void)TCA9548A_DisableAll();

    return result;
}

uint8_t ToF_IsSensorReady(ToF_Sensor_t sensor)
{
    if (sensor >= TOF_SENSOR_COUNT) {
        return 0U;
    }

    return s_ready[sensor];
}

int ToF_ReadSingle(ToF_Sensor_t sensor, ToF_Measurement_t *out)
{
    VL53L0X_RangingMeasurementData_t data;
    VL53L0X_Error status;

    ToF_InvalidateMeasurement(out);

    if (sensor >= TOF_SENSOR_COUNT || !s_ready[sensor]) {
        return TOF_ERROR;
    }

    if (ToF_SelectSensor(sensor) != TOF_OK) {
        return TOF_ERROR;
    }

    /* A sensor left free-running would ignore a single-shot trigger, so fall
     * back to reading its continuous stream instead of returning nothing. */
    if (s_continuous[sensor]) {
        return ToF_ReadContinuous(sensor, out, 1U);
    }

    /* Triggers the measurement, polls until complete, reads it back and
     * clears the interrupt -- the whole single-shot cycle in one call. */
    status = VL53L0X_PerformSingleRangingMeasurement(&s_dev[sensor], &data);

    if (status != VL53L0X_ERROR_NONE) {
        ToF_Filter_Invalidate(&s_filter[sensor]);
        return TOF_ERROR;
    }

    return ToF_DecodeMeasurement(sensor, &data, out);
}

uint16_t ToF_GetDistanceSingle(ToF_Sensor_t sensor)
{
    ToF_Measurement_t measurement;

    if (ToF_ReadSingle(sensor, &measurement) != TOF_OK) {
        return TOF_DISTANCE_INVALID;
    }

    return measurement.distance_mm;
}

int ToF_StartContinuous(ToF_Sensor_t sensor)
{
    VL53L0X_Error status;

    if (sensor >= TOF_SENSOR_COUNT || !s_ready[sensor]) {
        return TOF_ERROR;
    }

    if (s_continuous[sensor]) {
        return TOF_OK;
    }

    if (ToF_SelectSensor(sensor) != TOF_OK) {
        return TOF_ERROR;
    }

    /* TIMED rather than BACK_TO_BACK: back-to-back re-triggers the instant a
     * measurement finishes, which pins the sensor at maximum duty and floods
     * the bus for no benefit when the control loop reads far slower than the
     * sensor produces. TIMED paces it at a period we choose. */
    status = VL53L0X_SetDeviceMode(&s_dev[sensor],
                                   VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING);

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_SetInterMeasurementPeriodMilliSeconds(
            &s_dev[sensor], TOF_INTER_MEASUREMENT_MS);
    }

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_StartMeasurement(&s_dev[sensor]);
    }

    if (status != VL53L0X_ERROR_NONE) {
        return TOF_ERROR;
    }

    s_continuous[sensor] = 1U;
    return TOF_OK;
}

int ToF_StartContinuousAll(void)
{
    int result = TOF_OK;

    for (uint8_t i = 0; i < TOF_SENSOR_COUNT; i++) {
        if (!s_ready[i]) {
            continue;
        }

        if (ToF_StartContinuous((ToF_Sensor_t)i) != TOF_OK) {
            result = TOF_ERROR;
        }
    }

    return result;
}

int ToF_ReadContinuous(ToF_Sensor_t sensor, ToF_Measurement_t *out,
                       uint8_t wait_for_new)
{
    VL53L0X_RangingMeasurementData_t data;
    VL53L0X_Error status;
    uint8_t data_ready = 0U;
    uint32_t start_ms;

    ToF_InvalidateMeasurement(out);

    if (sensor >= TOF_SENSOR_COUNT || !s_ready[sensor]) {
        return TOF_ERROR;
    }

    if (!s_continuous[sensor]) {
        return TOF_ERROR;
    }

    if (ToF_SelectSensor(sensor) != TOF_OK) {
        return TOF_ERROR;
    }

    status = VL53L0X_GetMeasurementDataReady(&s_dev[sensor], &data_ready);

    if (status != VL53L0X_ERROR_NONE) {
        ToF_Filter_Invalidate(&s_filter[sensor]);
        return TOF_ERROR;
    }

    if (!data_ready) {
        if (!wait_for_new) {
            /* Nothing new yet. Expected in a loop faster than the sensor --
             * the caller keeps its previous reading.
             *
             * Deliberately does NOT invalidate the filter: this is the normal
             * outcome of polling faster than TOF_INTER_MEASUREMENT_MS, and
             * resetting here would clear the history on most calls and
             * destroy the smoothing entirely. */
            return TOF_ERROR_TIMEOUT;
        }

        start_ms = HAL_GetTick();

        while (!data_ready) {
            if ((HAL_GetTick() - start_ms) > TOF_DATA_READY_TIMEOUT_MS) {
                /* Waited longer than a measurement can legitimately take, so
                 * the stream really is broken -- unlike the non-blocking case
                 * above. */
                ToF_Filter_Invalidate(&s_filter[sensor]);
                return TOF_ERROR_TIMEOUT;
            }

            HAL_Delay(1);

            status = VL53L0X_GetMeasurementDataReady(&s_dev[sensor],
                                                     &data_ready);

            if (status != VL53L0X_ERROR_NONE) {
                ToF_Filter_Invalidate(&s_filter[sensor]);
                return TOF_ERROR;
            }
        }
    }

    status = VL53L0X_GetRangingMeasurementData(&s_dev[sensor], &data);

    if (status != VL53L0X_ERROR_NONE) {
        ToF_Filter_Invalidate(&s_filter[sensor]);
        return TOF_ERROR;
    }

    /* The interrupt latch must be cleared or the sensor never flags the next
     * measurement as ready and this function returns TIMEOUT forever after. */
    (void)VL53L0X_ClearInterruptMask(
        &s_dev[sensor], VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);

    return ToF_DecodeMeasurement(sensor, &data, out);
}

int ToF_StopContinuous(ToF_Sensor_t sensor)
{
    VL53L0X_Error status;

    if (sensor >= TOF_SENSOR_COUNT || !s_ready[sensor]) {
        return TOF_ERROR;
    }

    if (!s_continuous[sensor]) {
        return TOF_OK;
    }

    if (ToF_SelectSensor(sensor) != TOF_OK) {
        return TOF_ERROR;
    }

    status = VL53L0X_StopMeasurement(&s_dev[sensor]);

    /* Clear the flag regardless of the outcome. If the stop failed the sensor
     * is in an unknown state, and continuing to treat it as a healthy
     * free-running source would be worse than forcing a single-shot path. */
    s_continuous[sensor] = 0U;

    if (status != VL53L0X_ERROR_NONE) {
        return TOF_ERROR;
    }

    /* ---------------- wait for the stop to actually complete ----------------
     * VL53L0X_StopMeasurement() only requests the stop; the sensor finishes a
     * measurement already in flight before it takes effect. Reconfiguring the
     * device mode during that window leaves it in an undefined state and later
     * single-shot reads fail.
     *
     * Polarity is the opposite of what the name suggests: the status reads
     * NON-ZERO while still stopping and ZERO once done, so this polls until it
     * reaches zero. That zero-read is not just an observation -- look at
     * VL53L0X_GetStopCompletedStatus() in the ST API and you will see it also
     * rewrites StopVariable to re-arm the device. Skipping the poll therefore
     * skips a required device write, not merely a wait. */
    uint32_t stop_status = 1U;
    uint32_t start_ms = HAL_GetTick();

    while (stop_status != 0U) {

        if (VL53L0X_GetStopCompletedStatus(&s_dev[sensor], &stop_status)
                != VL53L0X_ERROR_NONE) {
            return TOF_ERROR;
        }

        if (stop_status == 0U) {
            break;
        }

        if ((HAL_GetTick() - start_ms) > TOF_STOP_TIMEOUT_MS) {
            /* Never seen in practice -- the stop takes a millisecond or two.
             * Reaching here means the re-arm write above never happened, so
             * the sensor is left in single-shot but unverified. */
            return TOF_ERROR_TIMEOUT;
        }

        HAL_Delay(1);
    }

    /* Return to single-shot so a later ToF_ReadSingle() behaves. */
    (void)VL53L0X_SetDeviceMode(&s_dev[sensor],
                                VL53L0X_DEVICEMODE_SINGLE_RANGING);
    (void)VL53L0X_ClearInterruptMask(
        &s_dev[sensor], VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);

    return TOF_OK;
}

int ToF_StopContinuousAll(void)
{
    int result = TOF_OK;

    for (uint8_t i = 0; i < TOF_SENSOR_COUNT; i++) {
        if (!s_ready[i] || !s_continuous[i]) {
            continue;
        }

        if (ToF_StopContinuous((ToF_Sensor_t)i) != TOF_OK) {
            result = TOF_ERROR;
        }
    }

    return result;
}

void ToF_ResetFilter(ToF_Sensor_t sensor)
{
    if (sensor >= TOF_SENSOR_COUNT) {
        return;
    }

    ToF_Filter_Reset(&s_filter[sensor]);
}

void ToF_ResetFilterAll(void)
{
    for (uint8_t i = 0; i < TOF_SENSOR_COUNT; i++) {
        ToF_Filter_Reset(&s_filter[i]);
    }
}

uint32_t ToF_GetFilterJumpCount(ToF_Sensor_t sensor)
{
    if (sensor >= TOF_SENSOR_COUNT) {
        return 0U;
    }

    return s_filter[sensor].jump_count;
}

int ToF_ReadAll(ToF_Measurement_t out[TOF_SENSOR_COUNT])
{
    int result = TOF_OK;

    if (out == NULL) {
        return TOF_ERROR;
    }

    for (uint8_t i = 0; i < TOF_SENSOR_COUNT; i++) {
        ToF_Sensor_t sensor = (ToF_Sensor_t)i;
        int status;

        if (!s_ready[i]) {
            ToF_InvalidateMeasurement(&out[i]);
            result = TOF_ERROR;
            continue;
        }

        /* Non-blocking in continuous mode so one slow sensor cannot hold up
         * the sweep; blocking single-shot otherwise. */
        if (s_continuous[i]) {
            status = ToF_ReadContinuous(sensor, &out[i], 0U);
        } else {
            status = ToF_ReadSingle(sensor, &out[i]);
        }

        if (status != TOF_OK) {
            result = TOF_ERROR;
        }
    }

    return result;
}
