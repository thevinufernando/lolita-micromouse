#ifndef WALL_FOLLOW_H
#define WALL_FOLLOW_H

#include "tof_sensors.h"
#include <stdint.h>

/*
 * ============================================================================
 *            WALL FOLLOWING - LATERAL POSITION FROM ONE SIDE WALL
 * ============================================================================
 *
 * Converts a side-wall distance into a heading offset that the straight-line
 * controller adds to its heading target. It does NOT command motors.
 *
 * ---------------------------------------------------------------------------
 * WHY THIS IS A CASCADE AND NOT A SECOND STEERING TERM
 * ---------------------------------------------------------------------------
 * Lateral position is two integrations away from the steering command:
 * steering changes heading, heading changes lateral rate. Adding a lateral
 * correction alongside a heading correction gives an undamped double
 * integrator, which is a textbook route to weaving down a corridor.
 *
 * Feeding lateral error into the heading SETPOINT instead means the heading
 * loop supplies the derivative term, and the system is damped for free. It
 * also bounds how hard the robot ever tilts, which summing would not.
 *
 * ---------------------------------------------------------------------------
 * ONE WALL, NOT TWO
 * ---------------------------------------------------------------------------
 * A maze corridor often has a wall on only one side, so requiring both would
 * abandon correction exactly where it is most needed. The cost is that the
 * sensors' common-mode bias no longer cancels in a difference -- which is
 * handled by making the setpoint a MEASURED READING rather than a true
 * distance. A constant bias then cancels exactly, because it is present in
 * both the setpoint and the measurement.
 *
 * ---------------------------------------------------------------------------
 * THE DRIFT CORRECTOR
 * ---------------------------------------------------------------------------
 * With no magnetometer the heading estimate drifts unbounded, and the maze
 * walls are the only absolute reference available.
 *
 * The correction falls straight out of the cascade. If the gyro has drifted,
 * then holding the wall at its setpoint requires a PERSISTENT non-zero tilt --
 * so the standing output of this loop is a direct measurement of the drift.
 * Bleeding it slowly into the heading target bounds the error without ever
 * differentiating wall distance or needing both walls.
 *
 * "Slowly" is load-bearing: the bleed must be far slower than the lateral
 * loop, or the two fight each other and the robot weaves.
 * ============================================================================
 */

typedef enum {
  WALL_FOLLOW_NONE = 0,  /* no usable wall; hold heading open-loop      */
  WALL_FOLLOW_LEFT = 1,  /* one wall, measured against its setpoint     */
  WALL_FOLLOW_RIGHT = 2,
  WALL_FOLLOW_BOTH = 3   /* centred on (L - R)/2 -- the reference to want */
} WallFollowSide_t;

/* Which wall is being used right now, and by how much the loop is tilting. */
extern volatile uint8_t wf_side;
extern volatile float wf_error_mm;   /* measured - setpoint, signed */
extern volatile float wf_tilt_deg;   /* heading offset being asked for */
extern volatile float wf_drift_deg;  /* total bled into the heading target */
extern volatile uint32_t wf_switches; /* times the active side changed */

/* Forget the active side and the accumulated drift. Call at the start of a
 * move, and any time continuity is broken. */
void WallFollow_Reset(void);

/* Feed one ToF sweep. Returns the heading offset in degrees to ADD to the
 * heading target: positive tilts the robot anticlockwise.
 *
 * FORWARD TRAVEL ONLY. The side sensors are at the very front of the chassis,
 * so in reverse they trail the body on a long arm and report the opposite of
 * what a correction is doing until long after the fact. There is no reverse
 * form of this and there should not be one; see control_config.h.
 *
 * Returns 0 when no wall is usable, which is the correct behaviour rather
 * than a failure -- the robot then holds its heading target open-loop until a
 * wall comes back. */
float WallFollow_Update(const ToF_Measurement_t m[TOF_SENSOR_COUNT]);

/* Drift accumulated so far, in degrees, to be applied to the heading target.
 * See the drift corrector note above. */
float WallFollow_GetDriftDeg(void);

#endif /* WALL_FOLLOW_H */
