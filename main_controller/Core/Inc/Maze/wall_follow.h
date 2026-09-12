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
extern volatile float wf_drift_deg;  /* integral, added to the heading target */
extern volatile uint32_t wf_switches; /* times the active side changed */

/* Mirror of wf_drift_deg under the name the telemetry reader prints: the
 * standing asymmetry the loop has learned, in degrees of heading.
 *
 * It should settle to a small non-zero number and stay there -- that value IS
 * the robot's bias. Still climbing at the end of a run means it has not
 * converged. Pinned at WALL_FOLLOW_KI_LIMIT_DEG means the asymmetry is
 * mechanical and wants fixing rather than trimming out. */
extern volatile float wf_integral;

/* WHAT THE MAP KNOWS ABOUT THE TWO CELLS THIS MOVE TOUCHES.
 *
 * The side sensors sit at the very FRONT of the chassis, about TOF_SIDE_AHEAD_CM
 * ahead of the axle, so they cross the cell boundary well before the robot
 * does -- at cross_cm into the move, which on the current geometry is under a
 * third of it. For the rest of every move they are looking at the cell being
 * ENTERED, not the one being left.
 *
 * Nothing in this module knew that, and it cost a run. The loop tracked the
 * right wall of the cell it was leaving for 3.4 cm past the boundary, read that
 * wall's recession as the robot drifting, and leaned into the opposite wall to
 * correct something that was not happening.
 *
 * THE MAP MAY ONLY WITHHOLD TRUST, NEVER ADD IT. A reference is dropped when
 * the applicable cell is known and records no wall on that side. It is never
 * invented: a wall the map believes in but the sensor cannot see is still not
 * followed. That direction is the whole safety argument -- a wrong pose then
 * makes this loop more cautious rather than more confident, and this robot has
 * driven off the edge of its own map before.
 *
 * An unknown cell contributes no opinion, so a cleared context vetoes nothing
 * and behaves exactly as this module did before it existed. */
typedef struct {
  uint8_t left_this,  right_this;  /* walls of the cell being left    */
  uint8_t left_next,  right_next;  /* walls of the cell being entered */
  uint8_t this_known, next_known;  /* 0 = that cell has no opinion    */
  float   cross_cm;                /* travel at which the sensors cross */
} WallFollowCells_t;

/* Set the context for the move about to start, or clear it with NULL.
 *
 * The NAVIGATOR calls this, because the navigator is what owns the map. The
 * straight-line controller stays ignorant of the maze, which matters because
 * the test harness drives it directly. */
void WallFollow_SetCells(const WallFollowCells_t *cells);

/* Per-move reset: forgets the active side and the current tilt, and KEEPS what
 * the loop has learned about the robot. Call at the start of a move and any
 * time continuity is broken. Also clears the cell context, so a move that
 * never sets one cannot inherit the previous move's. */
void WallFollow_Reset(void);

/* Per-SEGMENT reset, for chained cell motion. Tells the movement detector that
 * travelled distance has restarted at zero, and keeps everything else: the
 * robot has not stopped or turned, so the followed side and the current lean
 * are still valid. See the note in wall_follow.c. */
void WallFollow_NewSegment(void);

/* Full reset, including the learned lateral bias and drift. Call ONCE at the
 * start of a run. Per move it would mean re-learning the robot's asymmetry
 * every cell and never converging on it. */
void WallFollow_ResetBias(void);

/* Feed one ToF sweep, with the MEASURED seconds since the previous one.
 *
 * The interval is a parameter rather than a constant because the true gap is
 * not the nominal WALL_FOLLOW_UPDATE_S. A ToF read blocks the control loop for
 * about 138 ms, so the real cadence is 168 ms, and the slew limit and the
 * integral -- both rates -- were running at a quarter of the speed their
 * constants claim. The constant survives as the nominal value the host test
 * and the notes reason about, and as the floor this is clamped to.
 *
 * `travelled_cm` is how far into the move the robot is, and decides which of
 * the two cells in the context the side sensors are currently looking at.
 *
 * Returns the heading offset in degrees to ADD to the heading target:
 * positive tilts the robot anticlockwise.
 *
 * FORWARD TRAVEL ONLY. The side sensors are at the very front of the chassis,
 * so in reverse they trail the body on a long arm and report the opposite of
 * what a correction is doing until long after the fact. There is no reverse
 * form of this and there should not be one; see control_config.h.
 *
 * Returns 0 when no wall is usable, which is the correct behaviour rather
 * than a failure -- the robot then holds its heading target open-loop until a
 * wall comes back. */
float WallFollow_Update(const ToF_Measurement_t m[TOF_SENSOR_COUNT],
                        float dt_s, float travelled_cm);

/* The integral, in degrees, to be ADDED TO THE HEADING TARGET by the caller.
 *
 * This is the loop's only integrating term and it lives here, outside the tilt
 * clamp, on purpose: what it learns is a heading reference error, so a ceiling
 * set by how far the robot may lean would make it unable to correct a yaw
 * estimate that has drifted further than that. See WALL_FOLLOW_KI_LIMIT_DEG. */
float WallFollow_GetDriftDeg(void);

#endif /* WALL_FOLLOW_H */
