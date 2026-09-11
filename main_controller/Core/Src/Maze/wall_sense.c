#include "wall_sense.h"
#include "control_config.h"

volatile uint16_t wall_front_mm = TOF_DISTANCE_INVALID;
volatile uint16_t wall_left_mm  = TOF_DISTANCE_INVALID;
volatile uint16_t wall_right_mm = TOF_DISTANCE_INVALID;

volatile uint8_t wall_front_votes;
volatile uint8_t wall_left_votes;
volatile uint8_t wall_right_votes;

/* Latched state for the hysteretic path. */
static uint8_t latched[TOF_SENSOR_COUNT];
static uint8_t run_len[TOF_SENSOR_COUNT];


void WallSense_Reset(void)
{
    for (uint8_t i = 0; i < TOF_SENSOR_COUNT; i++) {
        latched[i] = 0U;
        run_len[i] = 0U;
    }
}


/* A reading only counts as a wall if it is both VALID and close. An invalid
 * reading is not evidence of open space -- the sensor simply did not answer --
 * so it is voted as "no wall seen" rather than being allowed to assert one. */
static uint8_t isWall(const ToF_Measurement_t *m, uint16_t threshold_mm)
{
    if (!m->valid || m->distance_mm == TOF_DISTANCE_INVALID) {
        return 0U;
    }

    return (m->distance_mm <= threshold_mm) ? 1U : 0U;
}


uint8_t WallSense_ReadCell(WallReading_t *out)
{
    uint8_t votes[TOF_SENSOR_COUNT] = {0U, 0U, 0U};
    uint8_t valid[TOF_SENSOR_COUNT] = {0U, 0U, 0U};
    uint32_t sum_mm[TOF_SENSOR_COUNT] = {0U, 0U, 0U};

    const uint16_t threshold[TOF_SENSOR_COUNT] = {
        WALL_FRONT_THRESHOLD_MM,
        WALL_SIDE_THRESHOLD_MM,
        WALL_SIDE_THRESHOLD_MM,
    };

    for (uint8_t s = 0; s < WALL_SENSE_SAMPLES; s++) {

        ToF_Measurement_t m[TOF_SENSOR_COUNT];
        (void)ToF_ReadAll(m);   /* a partial sweep is still worth voting on */

        for (uint8_t i = 0; i < TOF_SENSOR_COUNT; i++) {
            if (m[i].valid && m[i].distance_mm != TOF_DISTANCE_INVALID) {
                valid[i]++;
                sum_mm[i] += m[i].distance_mm;
            }
            votes[i] += isWall(&m[i], threshold[i]);
        }
    }

    wall_front_votes = votes[TOF_FRONT];
    wall_left_votes  = votes[TOF_LEFT];
    wall_right_votes = votes[TOF_RIGHT];

    /* Mean of the VALID samples only, so one dropped reading does not drag the
     * reported distance toward zero. */
    wall_front_mm = valid[TOF_FRONT] ? (uint16_t)(sum_mm[TOF_FRONT] / valid[TOF_FRONT])
                                     : TOF_DISTANCE_INVALID;
    wall_left_mm  = valid[TOF_LEFT]  ? (uint16_t)(sum_mm[TOF_LEFT]  / valid[TOF_LEFT])
                                     : TOF_DISTANCE_INVALID;
    wall_right_mm = valid[TOF_RIGHT] ? (uint16_t)(sum_mm[TOF_RIGHT] / valid[TOF_RIGHT])
                                     : TOF_DISTANCE_INVALID;

    /* Strict majority. On an even split the answer is "no wall", which is the
     * safe direction to be wrong in: a missed wall gets seen again from the
     * next cell, whereas a phantom wall is never cleared. */
    if (out != NULL) {
        out->front = (votes[TOF_FRONT] * 2U > WALL_SENSE_SAMPLES) ? 1U : 0U;
        out->left  = (votes[TOF_LEFT]  * 2U > WALL_SENSE_SAMPLES) ? 1U : 0U;
        out->right = (votes[TOF_RIGHT] * 2U > WALL_SENSE_SAMPLES) ? 1U : 0U;
    }

    return (valid[TOF_FRONT] && valid[TOF_LEFT] && valid[TOF_RIGHT]) ? 1U : 0U;
}


void WallSense_Update(const ToF_Measurement_t m[TOF_SENSOR_COUNT])
{
    const uint16_t enter[TOF_SENSOR_COUNT] = {
        WALL_FRONT_THRESHOLD_MM,
        WALL_SIDE_THRESHOLD_MM,
        WALL_SIDE_THRESHOLD_MM,
    };

    for (uint8_t i = 0; i < TOF_SENSOR_COUNT; i++) {

        /* Separate enter and exit thresholds. A single level would flip back
         * and forth on noise exactly at a cell boundary, which is the one
         * place the answer actually changes. */
        uint16_t exit_mm = (uint16_t)(enter[i] + WALL_SENSE_HYSTERESIS_MM);

        uint8_t want = latched[i];

        if (m[i].valid && m[i].distance_mm != TOF_DISTANCE_INVALID) {
            if (!latched[i] && m[i].distance_mm <= enter[i])   want = 1U;
            if ( latched[i] && m[i].distance_mm >  exit_mm)    want = 0U;
        }

        if (want != latched[i]) {
            run_len[i]++;

            /* Require the new answer to persist. One sample is noise; several
             * in a row is the world changing. */
            if (run_len[i] >= WALL_SENSE_CONFIRM) {
                latched[i] = want;
                run_len[i] = 0U;
            }
        }
        else {
            run_len[i] = 0U;
        }
    }

    wall_front_mm = m[TOF_FRONT].distance_mm;
    wall_left_mm  = m[TOF_LEFT].distance_mm;
    wall_right_mm = m[TOF_RIGHT].distance_mm;
}


uint8_t WallSense_Front(void) { return latched[TOF_FRONT]; }
uint8_t WallSense_Left(void)  { return latched[TOF_LEFT];  }
uint8_t WallSense_Right(void) { return latched[TOF_RIGHT]; }
