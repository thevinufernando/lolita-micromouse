#ifndef TOF_SENSORS_H
#define TOF_SENSORS_H

#include "main.h"
#include <stdint.h>

/*
 * ============================================================================
 *                    VL53L0X TIME-OF-FLIGHT RANGING DRIVER
 * ============================================================================
 *
 * Thin application layer over ST's official VL53L0X API (STSW-IMG005,
 * v1.0.4), covering the three sensors used for wall detection: front, left
 * and right. Everything under Sensors/VL53L0X/ is stock ST code and is not
 * modified here; the one exception is the platform layer
 * (Sensors/VL53L0X/Platform/vl53l0x_platform.c), which ST ships as a Win32
 * reference port and which has been rewritten against the STM32 HAL.
 *
 * SCOPE: this driver reports DISTANCES ONLY. Turning distances into
 * "is there a wall there" decisions is deliberately left out — that belongs
 * with the maze logic, which does not exist yet.
 *
 * ---------------------------------------------------------------------------
 * MUX
 * ---------------------------------------------------------------------------
 * All sensors share address 0x29 and are separated by a TCA9548A (see
 * TCA9548A.h). Every entry point here opens the right channel before touching
 * a sensor, so callers never deal with the mux directly. The channel numbers
 * are in control_config.h.
 *
 * ---------------------------------------------------------------------------
 * MODES
 * ---------------------------------------------------------------------------
 * Both ranging modes the API offers are exposed, because they suit different
 * phases of the project:
 *
 *   SINGLE     — ToF_ReadSingle(). The MCU asks for one measurement and
 *                blocks until it lands (~30 ms with the default budget).
 *                Simple and always in step with the caller. Good for
 *                bring-up and for a stationary "look around" at a cell
 *                centre. Costs a full measurement's latency per read.
 *
 *   CONTINUOUS — ToF_StartContinuous() once, then ToF_ReadContinuous()
 *                whenever a fresh sample is wanted. The sensor free-runs and
 *                the MCU picks up the newest result without waiting for a
 *                conversion. This is what a moving robot wants: reads become
 *                cheap and bounded, at the cost of the data being up to one
 *                measurement period stale.
 *
 * A sensor is in one mode at a time. ToF_StopContinuous() returns it to
 * single-shot.
 *
 * ---------------------------------------------------------------------------
 * UNITS AND VALIDITY
 * ---------------------------------------------------------------------------
 * Distances are UNSIGNED MILLIMETRES, matching the ST API's native
 * RangeMilliMeter. Note this differs from the cm used by the motion
 * controllers — the conversion belongs to whoever consumes both, not here.
 *
 * A reading is only meaningful when the call returns TOF_OK. On anything else
 * the distance is set to TOF_DISTANCE_INVALID rather than left stale, so a
 * caller that ignores the return code gets an obviously-wrong number instead
 * of a plausible old one. Out-of-range (nothing in front of the sensor) is
 * reported as TOF_ERROR_RANGE, which is a normal condition, not a fault.
 * ============================================================================
 */

#define TOF_OK             0
#define TOF_ERROR         -1  /* I2C / API failure                       */
#define TOF_ERROR_RANGE   -2  /* Sensor answered, measurement not valid  */
#define TOF_ERROR_TIMEOUT -3  /* No data-ready within the deadline       */

/* Returned in place of a distance whenever a read does not return TOF_OK. */
#define TOF_DISTANCE_INVALID 0xFFFFU

/* Sensor identifiers. These index the internal device table, so the order
 * must match the s_channel[] mapping in tof_sensors.c. */
typedef enum {
  TOF_FRONT = 0,
  TOF_LEFT = 1,
  TOF_RIGHT = 2,
  TOF_SENSOR_COUNT
} ToF_Sensor_t;

/* One decoded measurement. */
typedef struct {
  uint16_t distance_mm; /* TOF_DISTANCE_INVALID when not valid          */
  uint8_t range_status; /* Raw ST status; 0 = valid. Kept for diagnosis */
  uint8_t valid;        /* 1 = distance_mm is trustworthy               */
} ToF_Measurement_t;

/*
 * Bring up the mux and all three sensors.
 *
 * Per sensor this runs the full ST init sequence — DataInit, StaticInit,
 * reference SPAD management and reference calibration — then applies the
 * timing budget and range profile from control_config.h. Reference
 * calibration is done at runtime rather than loading stored values because
 * the sensors have never been characterised on this chassis; once they have,
 * the results can be cached to save ~40 ms per sensor at boot.
 *
 * Returns TOF_OK only when EVERY sensor came up. If some subset works the
 * function still returns TOF_ERROR, but the healthy sensors remain usable —
 * check ToF_IsSensorReady() to find out which. This mirrors how the turn
 * controller degrades when the IMU is absent: report the fault, keep going.
 */
int ToF_Init(void);

/* 1 if that sensor initialised and is usable, 0 otherwise. */
uint8_t ToF_IsSensorReady(ToF_Sensor_t sensor);

/* ---- Single-shot ---- */

/* Trigger one measurement and block until it completes.
 * Returns TOF_OK, TOF_ERROR_RANGE (valid transaction, unusable reading), or
 * TOF_ERROR. `out` may be NULL if only the return code is wanted. */
int ToF_ReadSingle(ToF_Sensor_t sensor, ToF_Measurement_t *out);

/* Convenience form: distance only, TOF_DISTANCE_INVALID on any failure. */
uint16_t ToF_GetDistanceSingle(ToF_Sensor_t sensor);

/* ---- Continuous ---- */

/* Put one sensor into free-running continuous ranging. */
int ToF_StartContinuous(ToF_Sensor_t sensor);

/* Put every successfully-initialised sensor into continuous ranging.
 * Returns TOF_OK only if all ready sensors started. */
int ToF_StartContinuousAll(void);

/* Fetch the most recent continuous result.
 *
 * If `wait_for_new` is 0 this returns immediately: TOF_OK with a fresh sample
 * if one is pending, TOF_ERROR_TIMEOUT if the sensor has not finished a new
 * measurement yet. That non-blocking form is the one a control loop wants.
 * If `wait_for_new` is 1 it blocks up to TOF_DATA_READY_TIMEOUT_MS. */
int ToF_ReadContinuous(ToF_Sensor_t sensor, ToF_Measurement_t *out,
                       uint8_t wait_for_new);

/* Stop continuous ranging and return the sensor to single-shot mode. */
int ToF_StopContinuous(ToF_Sensor_t sensor);

/* Stop continuous ranging on every sensor. */
int ToF_StopContinuousAll(void);

/* ---- Bulk helper ---- */

/* Read all three sensors into `out` (indexed by ToF_Sensor_t).
 * Works in either mode: continuous sensors are polled non-blocking, others
 * are read single-shot. Sensors that failed init are filled with an invalid
 * measurement. Returns TOF_OK only when all three produced a valid reading. */
int ToF_ReadAll(ToF_Measurement_t out[TOF_SENSOR_COUNT]);

#endif /* TOF_SENSORS_H */
