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

/* MOST RECENT GOOD MEASUREMENT, and when it arrived.
 *
 * Continuous ranging and a 10 ms control loop do not tick at the same rate:
 * the sensor produces a result about every TOF_INTER_MEASUREMENT_MS, so three
 * polls in four legitimately find nothing new. A non-blocking read reports
 * that as TOF_ERROR_TIMEOUT with the measurement invalidated, which is honest
 * but useless to a control loop -- the wall follower reads "invalid" as "no
 * wall", so it would drop its reference, slew its correction to zero, pick the
 * wall back up on the next fresh sample, and flicker at the poll rate.
 *
 * So the newest good reading is kept here and served, with an age, by
 * ToF_ReadAllLatest(). Holding it in the DRIVER rather than in each controller
 * keeps `valid` meaning the one thing every consumer already assumes it means:
 * this number can be trusted. */
static ToF_Measurement_t s_last[TOF_SENSOR_COUNT];
static uint32_t          s_last_ms[TOF_SENSOR_COUNT];

/* Throw away the next sample this sensor produces.
 *
 * Set by ToF_ResetFilterAll(), which the navigator calls after every pivot
 * because the stored samples describe a heading the robot no longer holds.
 * Single-shot made that reset a guarantee: the next measurement was triggered
 * after it. Free-running does not -- the sample already in flight may have
 * been captured halfway through the turn, and it would refill the history with
 * exactly the data the reset existed to discard.
 *
 * Dropping one sample makes the first reading the filter sees provably later
 * than the reset, with no assumption about how long anything takes. */
static uint8_t s_discard_next[TOF_SENSOR_COUNT];

/* Which sensor ToF_PollOneLatest() talks to next. */
static uint8_t s_poll_next;

/* How the reads are going. tof_stale_drops is the one that matters: it counts
 * readings that aged out entirely, which means a sensor stopped producing
 * rather than merely not being ready yet. It should be zero. */
volatile uint32_t tof_fresh_count;
volatile uint32_t tof_cached_count;
volatile uint32_t tof_stale_drops;

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

    /* The sample that may straddle a pivot. Read and cleared above -- which is
     * the whole point, the latch had to be cleared for the sensor to produce
     * another -- but never decoded, so the filter does not see it. Reported as
     * "nothing new yet", which is exactly what it is from the caller's side. */
    if (s_discard_next[sensor]) {
        s_discard_next[sensor] = 0U;
        return TOF_ERROR_TIMEOUT;
    }

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

        /* The cache is history too, and it is history about a heading the
         * robot no longer holds. Clearing the filter but serving the cached
         * reading afterwards would defeat the reset entirely. */
        ToF_InvalidateMeasurement(&s_last[i]);
        s_last_ms[i]      = 0U;
        s_discard_next[i] = s_continuous[i];   /* only free-running needs it */
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


/* Poll one sensor and fold the result into the cache. */
static int ToF_PollInto(uint8_t i, ToF_Measurement_t *out, uint32_t now)
{
    ToF_Sensor_t sensor = (ToF_Sensor_t)i;
    int status;

    if (s_continuous[i]) {
        status = ToF_ReadContinuous(sensor, out, 0U);
    } else {
        status = ToF_ReadSingle(sensor, out);
    }

    if (status == TOF_OK) {
        s_last[i]    = *out;
        s_last_ms[i] = now;
        tof_fresh_count++;
    }

    return status;
}


/* Fill out[i] from the cache, deciding whether it is still worth having.
 *
 * The age limit is what keeps this honest. Without it a sensor that has died
 * goes unnoticed: its last reading would be repeated forever and the wall
 * follower would steer to a wall that is no longer there. Past the limit the
 * measurement goes invalid, which every consumer already knows how to handle.
 *
 * Shared by both entry points below on purpose -- two copies of this decision
 * would be two chances for them to disagree about what a stale reading is. */
static int ToF_ServeCached(uint8_t i, ToF_Measurement_t *out,
                           uint32_t now, uint32_t max_age_ms)
{
    if (s_last[i].valid && (now - s_last_ms[i]) <= max_age_ms) {
        *out = s_last[i];
        tof_cached_count++;
        return TOF_OK;
    }

    ToF_InvalidateMeasurement(out);

    if (s_last[i].valid) {
        /* Had a reading, and it aged out. That is a sensor that stopped
         * producing, not a poll that was merely early. */
        ToF_InvalidateMeasurement(&s_last[i]);
        tof_stale_drops++;
    }

    return TOF_ERROR;
}


int ToF_ReadAllLatest(ToF_Measurement_t out[TOF_SENSOR_COUNT],
                      uint32_t max_age_ms)
{
    int result = TOF_OK;

    if (out == NULL) {
        return TOF_ERROR;
    }

    const uint32_t now = HAL_GetTick();

    for (uint8_t i = 0; i < TOF_SENSOR_COUNT; i++) {
        if (!s_ready[i]) {
            ToF_InvalidateMeasurement(&out[i]);
            result = TOF_ERROR;
            continue;
        }

        if (ToF_PollInto(i, &out[i], now) == TOF_OK) {
            continue;
        }

        if (ToF_ServeCached(i, &out[i], now, max_age_ms) != TOF_OK) {
            result = TOF_ERROR;
        }
    }

    return result;
}


int ToF_PollOneLatest(ToF_Measurement_t out[TOF_SENSOR_COUNT],
                      uint32_t max_age_ms)
{
    int result = TOF_OK;

    if (out == NULL) {
        return TOF_ERROR;
    }

    const uint32_t now = HAL_GetTick();

    /* ONE SENSOR PER CALL, in rotation. The other two come from the cache.
     *
     * Talking to all three at once costs about 35 ms of I2C -- mux select,
     * data-ready poll, the ranging block, and clearing the interrupt latch,
     * three times over -- and the control loop is stopped for every
     * millisecond of it. Spread across three calls no single cycle blocks for
     * more than about twelve, and each sensor is still refreshed every three
     * cycles, which at CONTROL_SAMPLE_TIME_S is comfortably inside
     * TOF_INTER_MEASUREMENT_MS. Nothing is sampled less often; it is the same
     * work, not bunched.
     *
     * The cost is that the side pair is no longer simultaneous. Two cycles
     * apart at cruise is about 2 mm of travel along the corridor and a small
     * fraction of a millimetre across it, which is well inside the noise the
     * span check already tolerates. */
    const uint8_t polled = s_poll_next;
    uint8_t       got    = 0U;

    s_poll_next = (uint8_t)((s_poll_next + 1U) % TOF_SENSOR_COUNT);

    if (s_ready[polled]) {
        got = (ToF_PollInto(polled, &out[polled], now) == TOF_OK) ? 1U : 0U;
    }

    for (uint8_t i = 0; i < TOF_SENSOR_COUNT; i++) {
        if (!s_ready[i]) {
            ToF_InvalidateMeasurement(&out[i]);
            result = TOF_ERROR;
            continue;
        }

        /* The one just polled successfully is already in `out`, and counting
         * it as a cache hit as well would make the fresh-to-held ratio -- the
         * number that says whether the loop is outrunning the sensors -- read
         * one in four when it is really one in one. */
        if (i == polled && got) {
            continue;
        }

        if (ToF_ServeCached(i, &out[i], now, max_age_ms) != TOF_OK) {
            result = TOF_ERROR;
        }
    }

    return result;
}


int ToF_ReadAllFresh(ToF_Measurement_t out[TOF_SENSOR_COUNT])
{
    int result = TOF_OK;

    if (out == NULL) {
        return TOF_ERROR;
    }

    const uint32_t now = HAL_GetTick();

    for (uint8_t i = 0; i < TOF_SENSOR_COUNT; i++) {
        ToF_Sensor_t sensor = (ToF_Sensor_t)i;
        int status;

        if (!s_ready[i]) {
            ToF_InvalidateMeasurement(&out[i]);
            result = TOF_ERROR;
            continue;
        }

        /* WAITS for a genuinely new measurement in continuous mode, which is
         * the opposite of what a moving robot wants and exactly right for a
         * stationary one. WallSense votes several times at a cell centre and
         * the votes are only worth counting if they are independent samples;
         * back-to-back non-blocking reads would return one sample several
         * times over and turn a 5-sample vote into a 1-sample vote wearing a
         * disguise.
         *
         * Single-shot already blocks for a fresh measurement, so there it is
         * the existing behaviour unchanged. */
        if (s_continuous[i]) {
            status = ToF_ReadContinuous(sensor, &out[i], 1U);

            /* One retry, and only for a timeout. The sample armed for discard
             * by ToF_ResetFilterAll() reports itself that way, and consuming
             * it is the point -- but a caller that asked to WAIT for a new
             * measurement should get one, not the news that the previous one
             * was thrown away. A genuinely broken stream simply times out
             * twice, which is bounded and still reports failure. */
            if (status == TOF_ERROR_TIMEOUT) {
                status = ToF_ReadContinuous(sensor, &out[i], 1U);
            }
        } else {
            status = ToF_ReadSingle(sensor, &out[i]);
        }

        if (status == TOF_OK) {
            s_last[i]    = out[i];
            s_last_ms[i] = now;
            tof_fresh_count++;
        } else {
            result = TOF_ERROR;
        }
    }

    return result;
}
