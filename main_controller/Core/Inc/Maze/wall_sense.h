#ifndef WALL_SENSE_H
#define WALL_SENSE_H

#include "tof_sensors.h"
#include <stdint.h>

/*
 * ============================================================================
 *                  WALL DETECTION - DISTANCES TO BOOLEANS
 * ============================================================================
 *
 * Turns three ToF distances into three "is there a wall" answers. This is the
 * layer the scope notes deliberately left out until now: tof_sensors.c reports
 * millimetres and refuses to interpret them, and maze_map.c stores booleans
 * and refuses to measure them. This is the join.
 *
 * ---------------------------------------------------------------------------
 * WHY A THRESHOLD IS ENOUGH HERE
 * ---------------------------------------------------------------------------
 * The two cases are nowhere near each other, so this is not a marginal call:
 *
 *   side wall present   ~62 mm      side open      300 mm and up
 *   front wall present  ~77 mm      front open     250 mm and up
 *
 * That is why the sensors' known +27 mm over-read at close range does not
 * matter for wall DETECTION and does not need the offset constants. It matters
 * for wall FOLLOWING, where the number itself is the feedback.
 *
 * ---------------------------------------------------------------------------
 * TWO DIFFERENT JOBS, TWO DIFFERENT FUNCTIONS
 * ---------------------------------------------------------------------------
 * WallSense_ReadCell() is for standing still at a cell centre and deciding
 * what to write into the map. It votes over several samples, because a single
 * bad reflection writing a phantom wall into the map is unrecoverable: the
 * algorithm never clears walls, so one bad sample closes a corridor forever.
 *
 * WallSense_Update() is for use while moving, gating the wall-following loop.
 * It is hysteretic rather than voting, because the question there is not "is
 * this reading good" but "has the wall ended", and a single threshold chatters
 * at every cell boundary as the sensor cone catches the wall edge.
 * ============================================================================
 */

typedef struct {
  uint8_t front;
  uint8_t left;
  uint8_t right;
} WallReading_t;

/* Raw distances behind the most recent decision, for telemetry and for
 * telling a marginal call apart from a confident one. */
extern volatile uint16_t wall_front_mm;
extern volatile uint16_t wall_left_mm;
extern volatile uint16_t wall_right_mm;

/* Per-sensor vote tallies from the last WallSense_ReadCell(), out of
 * WALL_SENSE_SAMPLES. A unanimous result is trustworthy; a 3-2 split means
 * the robot is not where it thinks it is, or a sensor is marginal. */
extern volatile uint8_t wall_front_votes;
extern volatile uint8_t wall_left_votes;
extern volatile uint8_t wall_right_votes;

/* Blocking. Takes WALL_SENSE_SAMPLES sweeps and majority-votes each sensor.
 * The robot must be STATIONARY and at a cell centre.
 * Returns 1 if every sensor produced at least one valid reading, 0 otherwise
 * (in which case `out` still holds the best answer available). */
uint8_t WallSense_ReadCell(WallReading_t *out);

/* Hysteretic form for use while moving. Feed it one ToF sweep per control
 * cycle; read the latched answers with the accessors below. */
void WallSense_Update(const ToF_Measurement_t m[TOF_SENSOR_COUNT]);
void WallSense_Reset(void);

uint8_t WallSense_Front(void);
uint8_t WallSense_Left(void);
uint8_t WallSense_Right(void);

#endif /* WALL_SENSE_H */
