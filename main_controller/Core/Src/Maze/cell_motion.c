#include "cell_motion.h"

#include "maze_map.h"
#include "navigator.h"
#include "wall_follow.h"
#include "tof_sensors.h"
#include "straightline_controller.h"
#include "turn_controller.h"
#include "encoders.h"
#include "DRV8833.h"
#include "control_config.h"
#include "main.h"

volatile float   tm_maze_residual_cm;
volatile uint32_t tm_chain_gap_ms_max;
volatile uint32_t tm_chain_segments;
volatile uint32_t tm_chain_flight_reads;
volatile uint32_t tm_chain_stop_reads;

/* ---- WHERE THE ROBOT IS BETWEEN CELLS ----
 *
 * s_rolling says the robot is sitting at a DECISION POINT --
 * CELL_DECISION_OFFSET_CM short of a cell centre, still travelling at cruise,
 * with the motors holding their last command while the solver thinks.
 *
 * s_chain_ref_cm is the reading the odometer WOULD have at the centre of the
 * cell the pose names. The robot has usually not reached it (that is the whole
 * point) and may never reach it exactly, which is why every segment's length
 * is worked out from this against the live odometer rather than accumulated.
 * Lag, open-loop travel during the gap and a front-wall correction all land in
 * the same place and are all absorbed the same way.
 *
 * The old per-move residual is still published, because it is the clearest
 * single number for whether the robot is keeping its place along a corridor --
 * it is now derived from this rather than carried separately. */
static uint8_t s_rolling;
static float   s_chain_ref_cm;
static uint32_t s_gap_mark_ms;

/* Walls read on the way into the cell the robot is now in, and whether enough
 * of them arrived to be worth believing.
 *
 * COPIED, NOT REFERENCED. The straight controller's sl_flight_* globals belong
 * to the LAST SEGMENT IT RAN, and the segment that stops the robot at a cell
 * centre is a later one -- it clears them on the way in. In the speed-run
 * phase, where the solver never asks for walls until after it has turned, that
 * is exactly the order things happen in, and reading them late would report
 * every distance as absent. */
static struct {
    WallReading_t w;
    uint16_t      front_mm, left_mm, right_mm;
    uint8_t       front_votes, left_votes, right_votes;
    uint8_t       samples;
} s_flight;

static uint8_t s_flight_ok;

/* WHAT THE FRONT-WALL ALIGNMENT DID ON THE MOVE THAT ARRIVED HERE.
 *
 * Latched for the same reason the walls are: the straight controller's
 * sl_align_* globals describe the LAST SEGMENT IT RAN, and a cell that ends in
 * a turn runs a second, shorter segment to come to rest before the record is
 * written. That segment deliberately does no alignment, so it reset the reason
 * to "fired" -- and a 19-cell run reported fourteen alignments while exactly
 * one correction had been applied. A log that confidently reports the opposite
 * of what happened is worse than one that reports nothing. */
/* Latched with the alignment outcome and for the same reason: the stop
 * segment that precedes a pivot would otherwise overwrite it. */
static float   s_exit_err_mm;
static uint8_t s_exit_valid;

/* The front reading taken at rest at the cell centre -- the alignment's
 * outcome, as opposed to its intent. */
static uint16_t s_stop_front_mm = TOF_DISTANCE_INVALID;

static uint8_t s_align_reason;
static uint8_t s_align_applied;
static float   s_align_delta_cm;
volatile uint8_t tm_maze_tof_start_fail;
volatile uint8_t tm_maze_tof_stop_fail;

/* THE TRACE LIVES HERE, not with either driver, because it is a record of what
 * the ROBOT did rather than of what any particular rule decided. Every
 * diagnosis this firmware has survived came out of it, and having two drivers
 * each keep their own would mean the flood fill starting blind. Declared in
 * navigator.h, which is also where MazeTrace_t and the NAV_ACT_* vocabulary
 * live. */
volatile MazeTrace_t tm_maze_trace[MAZE_TRACE_CAPACITY];
volatile uint32_t    tm_maze_trace_count;
volatile uint8_t     tm_maze_complete;
volatile uint32_t    tm_maze_moves;
volatile uint8_t     tm_maze_abort_reason;


void CellMotion_Record(float move_error_cm, uint8_t move_ok,
                       const WallReading_t *w, uint8_t action)
{
    if (tm_maze_trace_count >= MAZE_TRACE_CAPACITY) return;

    volatile MazeTrace_t *r = &tm_maze_trace[tm_maze_trace_count];

    r->timestamp_ms   = HAL_GetTick();
    r->x              = mouse_x;
    r->y              = mouse_y;
    r->dir            = (uint8_t)mouse_dir;
    r->front          = w->front;
    r->left           = w->left;
    r->right          = w->right;
    r->front_mm       = wall_front_mm;
    r->left_mm        = wall_left_mm;
    r->right_mm       = wall_right_mm;
    r->yaw_deg        = TurnController_GetYawDeg();
    r->heading_target = TurnController_GetHeadingTargetDeg();
    r->move_error_cm  = move_error_cm;
    r->move_ok        = move_ok;
    r->wall_side      = wf_side;
    r->action         = action;
    r->drift_deg      = WallFollow_GetDriftDeg();
    r->entry_err_mm   = sl_entry_err_mm;
    r->entry_valid    = sl_entry_valid;
    r->align_delta_cm = s_align_applied ? s_align_delta_cm : 0.0f;
    r->exit_err_mm    = s_exit_valid ? s_exit_err_mm : 0.0f;
    r->stop_front_mm  = s_stop_front_mm;

    r->votes = (uint16_t)((wall_front_votes  & 7U)
                        | ((wall_left_votes   & 7U) << 3)
                        | ((wall_right_votes  & 7U) << 6)
                        | ((s_align_reason    & 7U) << 9));

    tm_maze_trace_count++;
}


/* Hand the wall follower what the map knows about the two cells this move
 * touches, so it can tell a reference that has ended from a robot that has
 * drifted.
 *
 * Called immediately before each forward move, from the pose the move starts
 * at -- which is why it reads mouse_x/y/dir directly rather than taking them:
 * any disagreement between the pose used here and the pose the move actually
 * begins from would silently veto the wrong cell's walls.
 *
 * An unknown next cell leaves next_known at 0, which contributes no opinion
 * and leaves the follower exactly as it behaves without any of this. That is
 * the common case on a first pass and it is meant to be. */
/* Defined below, beside the chained forward it is the other half of. Declared
 * here because everything that needs the robot standing still calls it, and
 * those come first in the file. */
static uint8_t stopAtCell(void);


/* Build what the map knows about the two cells a segment touches.
 *
 * RETURNED RATHER THAN INSTALLED. It used to call WallFollow_SetCells() here,
 * before starting the move -- and the move's own WallFollow_Reset() then wiped
 * it. It now travels with the move and is applied after that reset. */
static WallFollowCells_t s_cells;

static const WallFollowCells_t *cellContext(float start_offset_cm)
{
    WallFollowCells_t c;
    uint8_t f;

    c.this_known = MazeMap_IsKnown(mouse_x, mouse_y);
    MazeMap_CellWalls(mouse_x, mouse_y, mouse_dir, &f, &c.left_this, &c.right_this);

    int16_t nx = 0, ny = 0;

    c.left_next  = 0U;
    c.right_next = 0U;
    c.next_known = 0U;

    if (MazeMap_NextCell(mouse_x, mouse_y, mouse_dir, &nx, &ny)
        && MazeMap_IsKnown(nx, ny)) {

        /* Same heading: the robot does not turn during a forward move, so the
         * next cell's left and right are the same sides its own are. */
        MazeMap_CellWalls(nx, ny, mouse_dir, &f, &c.left_next, &c.right_next);
        c.next_known = 1U;
    }

    /* The sensors lead the axle, so they cross into the next cell well before
     * the robot does. Derived rather than configured: it is a consequence of
     * the cell pitch and where the sensors are bolted, and two constants that
     * can disagree about the same fact is one too many. */
    /* `start_offset_cm` is how far BEFORE this cell's centre the segment
     * begins, which is CELL_DECISION_OFFSET_CM on a chained move and zero from
     * a standstill. It has to be added, because every distance the follower is
     * given is measured from the start of the segment and the boundary is not
     * where it would be if the robot had started at the centre. */
    c.cross_cm = start_offset_cm + NAV_CELL_CM * 0.5f - TOF_SIDE_AHEAD_CM;

    s_cells = c;

    return &s_cells;
}


/* Come to rest, read the walls, and write them into the map.
 *
 * Deliberately does NOT log. The record has to carry the action chosen from
 * this reading, and that is not known until after the decision -- so observing
 * and recording are two steps with the decision in between, and the log reads
 * as a decision trace rather than a list of places the robot happened to be. */
void CellMotion_Observe(WallReading_t *w, uint8_t write_map)
{
    /* The vote only means anything standing still: five sweeps taken while
     * moving are five different places, not five looks at one. A driver that
     * calls this is asking for the careful answer and is going to pay for it.
     *
     * This is also what keeps the reactive navigator on the old behaviour
     * without a line of change -- it observes at every cell, so it never
     * chains, which is exactly what it did before. */
    (void)stopAtCell();

    tm_chain_stop_reads++;

    Motor_Brake();
    HAL_Delay(NAV_SETTLE_MS);

    (void)WallSense_ReadCell(w);

    /* After a FAILED move the robot is somewhere between two cells and the
     * pose was deliberately not advanced, so these readings belong to no cell
     * the map can name. Log them, because they are the evidence of what went
     * wrong, but do not write them: a wall recorded into the wrong cell is
     * exactly the error the map can never undo. */
    if (write_map) MazeMap_UpdateWalls(w->front, w->left, w->right);
}


/* A pivot invalidates every ToF history: the sensor that was looking at a wall
 * 60 mm away is now looking somewhere else entirely, and the median and EMA
 * stages carry several samples of the old value across. The jump detector only
 * rescues LARGE steps, so a turn from one wall to another wall at a similar
 * distance would slip through as a slow ramp.
 *
 * The wall follower is reset for the same reason: its chosen side and its
 * accumulated drift both refer to a heading the robot no longer holds. */
static void afterTurn(void)
{
    ToF_ResetFilterAll();
    WallFollow_Reset();

    /* turnLeftAngle()/turnRightAngle() reset the encoders, so the distance
     * axis the chain measures against has just been rebuilt underneath it.
     * Re-anchor on the new one: the robot is standing at a cell centre, which
     * is the one place this can be stated rather than estimated.
     *
     * The in-flight reading goes with it. It is in robot-relative terms --
     * front, left, right -- and the robot no longer faces the way it did when
     * they were taken. */
    s_rolling      = 0U;
    s_flight_ok    = 0U;
    s_chain_ref_cm = Encoder_getAverageDistance();

    /* The carried residual is an error ALONG the direction of travel. A pivot
     * makes that axis the new lateral axis, where this number means nothing --
     * carrying it would aim the next move at a target derived from a distance
     * measured sideways. Zero is the honest value: after a turn the robot does
     * not know its longitudinal offset, and the wall follower and (later) the
     * front-wall anchor are what recover it. */
    tm_maze_residual_cm = 0.0f;
}


void CellMotion_BeginRun(void)
{
    WallFollow_ResetBias();   /* start of a RUN: forget the learned bias too */

    /* FREE-RUN THE SENSORS FOR THE WHOLE RUN. See MAZE_TOF_CONTINUOUS.
     *
     * Started before the filter reset, not after, so the reset sees the
     * sensors already in continuous mode and arms its discard accordingly.
     * The two are describing the same state and should not disagree. */
#if MAZE_TOF_CONTINUOUS
    tm_maze_tof_start_fail = (ToF_StartContinuousAll() != TOF_OK) ? 1U : 0U;
#endif

    ToF_ResetFilterAll();
    TurnController_ResetYaw();   /* start of run: heading origin is here */

    tm_maze_residual_cm = 0.0f;

    /* TurnController_ResetYaw() resets the encoders, so the run's distance
     * axis starts here and the robot is standing at the origin cell's centre
     * by definition. */
    s_rolling             = 0U;
    s_flight_ok           = 0U;
    s_align_applied       = 0U;
    s_align_reason        = SL_ALIGN_NO_WALL;
    s_align_delta_cm      = 0.0f;
    s_exit_err_mm         = 0.0f;
    s_exit_valid          = 0U;
    s_stop_front_mm       = TOF_DISTANCE_INVALID;
    s_chain_ref_cm        = Encoder_getAverageDistance();
    s_gap_mark_ms         = HAL_GetTick();
    tm_chain_gap_ms_max   = 0U;
    tm_chain_segments     = 0U;
    tm_chain_flight_reads = 0U;
    tm_chain_stop_reads   = 0U;
}


void CellMotion_EndRun(void)
{
    /* However the run ended, it must not end with the robot still rolling. */
    (void)stopAtCell();

    Motor_Brake();

#if MAZE_TOF_CONTINUOUS
    tm_maze_tof_stop_fail = (ToF_StopContinuousAll() != TOF_OK) ? 1U : 0U;
#endif
}


uint8_t CellMotion_TurnLeft(void)
{
    /* A PIVOT FROM A ROLLING START IS AN ARC, and an arc through a maze cell
     * ends in a wall. Chained motion leaves the robot travelling at every cell
     * it does not need to stop at, so this is where "it turns out we do need
     * to stop" gets acted on. Costs nothing when the robot is already still. */
    if (!stopAtCell()) return 0U;

    if (!turnLeftAngle(90.0f)) return 0U;

    Motor_Brake();
    HAL_Delay(NAV_PIVOT_SETTLE_MS);
    afterTurn();

    return 1U;
}


uint8_t CellMotion_TurnRight(void)
{
    if (!stopAtCell()) return 0U;

    if (!turnRightAngle(90.0f)) return 0U;

    Motor_Brake();
    HAL_Delay(NAV_PIVOT_SETTLE_MS);
    afterTurn();

    return 1U;
}


/* ==================== COMING TO REST AT A CELL CENTRE ====================
 *
 * The other half of chained motion. A chained forward deliberately stops
 * driving CELL_DECISION_OFFSET_CM short of the centre and hands the robot over
 * still moving; this drives that last stretch and stops.
 *
 * IT IS CALLED FROM EVERYTHING THAT NEEDS THE ROBOT STILL -- both pivots,
 * CellMotion_Observe(), and the end of a run -- rather than from the drivers,
 * so no caller can forget. A pivot from a rolling start would carve an arc
 * through a wall, and there is no sensible way to recover from that.
 *
 * THE DISTANCE IS MEASURED, NOT ASSUMED. Whatever the robot did during the
 * solver's think-time has already happened and is already in the odometer, so
 * asking where it is now is both simpler and more honest than budgeting for
 * it. If it has somehow gone past the centre there is nothing useful to do --
 * this chassis has no reverse -- so it stops where it is and lets the residual
 * carry say so. */
static uint8_t stopAtCell(void)
{
    if (!s_rolling) return 1U;

    s_rolling = 0U;

    /* Count the open-loop stretch the solver cost, as the number that says
     * whether CELL_DECISION_MARGIN_CM is still buying enough. */
    const uint32_t gap = HAL_GetTick() - s_gap_mark_ms;

    if (gap > tm_chain_gap_ms_max) tm_chain_gap_ms_max = gap;

    const float remaining = s_chain_ref_cm - Encoder_getAverageDistance();
    uint8_t     ok        = 1U;

    if (remaining > 0.1f) {

        StraightMove_t mv = {
            .distance_cm      = remaining,
            /* ALIGN AGAIN HERE, AGAINST THE REAL TARGET.
             *
             * This used to be 0, on the reasoning that the long segment had
             * already applied the front-wall correction and a second one would
             * correct the same error twice. That reasoning was wrong in a way
             * the numbers eventually showed: the long segment's alignment
             * places the DECISION POINT, and the five centimetres that follow
             * are pure odometry. Every bit of wheel slip, carried residual and
             * arrival slop in those five centimetres lands straight in the
             * front-wall gap, which measured 16 to 83 mm against a 75 mm
             * target across nine walled stops.
             *
             * It is not the same correction twice, it is a second and much
             * better look: the wall is at 125 mm when this segment starts and
             * 75 when it ends, which is the closest and most accurate reading
             * the front sensor ever gets. The refusal tests are unchanged, so
             * it declines when there is no wall or no room. */
            .front_target_mm  = WALL_FRONT_ALIGN_MM,
            .entry_speed_cms  = CELL_CHAIN_SPEED_CMS,
            .exit_speed_cms   = 0.0f,
            .keep_wall_follow = 1U,
            .keep_odometry    = 1U,
            .wall_window_cm   = -1.0f,
            .wall_centre_cm   = 0.0f,
        };

        /* The robot is standing one braking offset short of THIS cell's
         * centre, which is what the follower needs in order to work out when
         * its side sensors cross into the next one. They will not, over a
         * stretch this short -- and saying so correctly is cheaper than
         * relying on it. */
        mv.cells = cellContext(CELL_DECISION_OFFSET_CM);

        ok = runForwardMove(&mv);
    }

    Motor_Brake();

    /* A stop that aligned is placed by the WALL, so the gap the encoders
     * report is the correction rather than an error left over -- the same rule
     * the forward move follows, applied at the other end of the cell. */
    const uint8_t stop_aligned = sl_align_applied;

    /* The stop segment ends on a settle check -- five cycles under
     * STRAIGHT_SETTLE_SPEED_CMS -- so the robot is stopped, but a light
     * chassis is still swinging on its wheels for a moment after that, and a
     * pivot is usually the next thing to happen. */
    HAL_Delay(NAV_PIVOT_SETTLE_MS);

    /* WHERE IT ACTUALLY STOPPED. Taken here and nowhere else: this is the only
     * moment the robot is at rest at a cell centre with the front sensor
     * looking down the corridor it is about to leave or turn out of.
     *
     * The held reading costs nothing -- the sensors free-run and the robot has
     * been still for the settle above, so the newest sample is already from
     * after it stopped. */
    {
        ToF_Measurement_t m[TOF_SENSOR_COUNT];

        s_stop_front_mm = (ToF_ReadAllLatest(m, TOF_MAX_SAMPLE_AGE_MS) == TOF_OK
                           && m[TOF_FRONT].valid)
                        ? m[TOF_FRONT].distance_mm
                        : TOF_DISTANCE_INVALID;
    }

    /* Standing at the centre by definition now: whatever gap is left is error
     * the next move inherits, and it is reported the same way it always was. */
    tm_maze_residual_cm = stop_aligned
                        ? 0.0f
                        : (s_chain_ref_cm - Encoder_getAverageDistance());

    if (tm_maze_residual_cm >  NAV_RESIDUAL_LIMIT_CM)
        tm_maze_residual_cm =  NAV_RESIDUAL_LIMIT_CM;
    if (tm_maze_residual_cm < -NAV_RESIDUAL_LIMIT_CM)
        tm_maze_residual_cm = -NAV_RESIDUAL_LIMIT_CM;

    s_chain_ref_cm = Encoder_getAverageDistance() + tm_maze_residual_cm;

    return ok;
}


uint8_t CellMotion_StopAtCell(void)
{
    return stopAtCell();
}


uint8_t CellMotion_FlightWalls(WallReading_t *w)
{
    if (!s_flight_ok || w == 0) return 0U;

    *w = s_flight.w;

    /* PUBLISH THEM WHERE THE STATIONARY READ PUBLISHES ITS OWN, so the per-cell
     * trace records the same three columns whichever path produced them and a
     * log can be read without knowing which. The vote counts are rescaled to
     * the stationary read's five, because that is what the trace's three-bit
     * fields hold and what every existing log means by them. */
    wall_front_mm = s_flight.front_mm;
    wall_left_mm  = s_flight.left_mm;
    wall_right_mm = s_flight.right_mm;

    const uint8_t n = s_flight.samples ? s_flight.samples : 1U;

    wall_front_votes = (uint8_t)((s_flight.front_votes * WALL_SENSE_SAMPLES) / n);
    wall_left_votes  = (uint8_t)((s_flight.left_votes  * WALL_SENSE_SAMPLES) / n);
    wall_right_votes = (uint8_t)((s_flight.right_votes * WALL_SENSE_SAMPLES) / n);

    tm_chain_flight_reads++;

    return 1U;
}


uint8_t CellMotion_Forward(void)
{
#if MAZE_CONTINUOUS_CELLS

    /* WHERE THIS SEGMENT MUST END, on the run's own odometer: the decision
     * point of the cell being entered, which is one cell pitch past this
     * cell's centre less the braking offset.
     *
     * Written this way -- one expression, valid whether the robot is standing
     * at a centre or already rolling -- because the two cases differ only in
     * where the robot happens to be now, and that is already in the odometer.
     * Spelling them out separately is how the two get to disagree. */
    if (s_rolling) {
        /* Open-loop for as long as the solver took. Harmless between two
         * forwards -- the segment length below is worked out from where the
         * robot actually is, so the travel is absorbed rather than lost -- but
         * it is the same clock that decides whether there is room to stop, so
         * it is worth seeing either way. */
        const uint32_t gap = HAL_GetTick() - s_gap_mark_ms;

        if (gap > tm_chain_gap_ms_max) tm_chain_gap_ms_max = gap;
    }

    const float end_cm   = s_chain_ref_cm + NAV_CELL_CM - CELL_DECISION_OFFSET_CM;
    const float start_cm = Encoder_getAverageDistance();

    float distance = end_cm - start_cm;

    /* A move that has been asked for something absurd -- a residual that ran
     * away, an odometer that jumped -- drives a sane cell instead of whatever
     * the arithmetic said. The clamp is the same one the residual carry has
     * always had, applied to the same quantity. */
    const float nominal = NAV_CELL_CM - (s_rolling ? 0.0f : CELL_DECISION_OFFSET_CM);

    if (distance > nominal + NAV_RESIDUAL_LIMIT_CM)
        distance = nominal + NAV_RESIDUAL_LIMIT_CM;
    if (distance < nominal - NAV_RESIDUAL_LIMIT_CM)
        distance = nominal - NAV_RESIDUAL_LIMIT_CM;

    const float start_offset = s_rolling ? CELL_DECISION_OFFSET_CM : 0.0f;

    StraightMove_t mv = {
        .distance_cm      = distance,
        /* The segment ends short of the centre, so the wall it is aiming to
         * finish in front of is that much further away than the figure the
         * stationary alignment uses. */
        .front_target_mm  = WALL_FRONT_ALIGN_MM + CELL_DECISION_OFFSET_CM * 10.0f,
        .entry_speed_cms  = s_rolling ? CELL_CHAIN_SPEED_CMS : 0.0f,
        .exit_speed_cms   = CELL_CHAIN_SPEED_CMS,
        .keep_wall_follow = s_rolling,
        /* ALWAYS, including the segment that starts from rest: the chain's
         * distance axis spans a whole corridor and is only re-anchored by a
         * pivot, which resets the encoders itself. */
        .keep_odometry    = 1U,
        /* Sampling starts where the side sensors cross into the cell being
         * entered -- the same boundary the follower's cell veto uses, and half
         * a centimetre past it so no sample straddles the gap in the wall. */
        .wall_window_cm   = start_offset + NAV_CELL_CM * 0.5f
                            - TOF_SIDE_AHEAD_CM + 0.5f,
        .wall_centre_cm   = distance + CELL_DECISION_OFFSET_CM,
        .cells            = cellContext(start_offset),
    };

    const uint8_t ok = runForwardMove(&mv);

    /* Captured now, before anything else can run a segment. */
    s_align_reason   = sl_align_reason;
    s_align_applied  = sl_align_applied;
    s_align_delta_cm = sl_align_delta_cm;
    s_exit_err_mm    = sl_exit_err_mm;
    s_exit_valid     = sl_exit_valid;

    tm_chain_segments++;

    if (!ok) {
        Motor_Brake();
        s_rolling      = 0U;
        s_flight_ok    = 0U;
        s_chain_ref_cm = Encoder_getAverageDistance();
        tm_maze_residual_cm = 0.0f;
        return 0U;
    }

    s_rolling       = 1U;
    s_gap_mark_ms   = HAL_GetTick();
    /* Chained straight through, so this cell has no stop of its own. Carrying
     * the previous one forward put the same number on two rows of the trace
     * and made a cell with no wall ahead appear to have stopped 60 mm from
     * one. */
    s_stop_front_mm = TOF_DISTANCE_INVALID;

    /* The cell the pose is about to name has moved on by one pitch. */
    s_chain_ref_cm += NAV_CELL_CM;

    /* !! THE TWO CORRECTIONS MUST NOT BOTH FIRE !!
     *
     * The carry exists because the encoders quietly lose ground every move.
     * The front-wall alignment exists because the encoders cannot know where
     * the cell boundaries are. When the alignment has fired it has ALREADY put
     * the robot where it belongs, measured against a wall -- so the gap the
     * encoders report is not an error left over, it IS the correction, and
     * carrying it into the next move applies the same correction twice.
     *
     * Observed exactly that: two cells finished 88 and 89 mm from a wall
     * against an 87 mm target, so within a millimetre of perfect, while
     * reporting 2.18 and 3.00 cm of shortfall. Both were carried forward, the
     * next move overran by that much, and after the following turn it came
     * back as lateral error.
     *
     * Chained, "believe the wall" means re-anchoring the whole distance axis
     * on where the robot actually is, which is what this does. */
    const float here = Encoder_getAverageDistance() + CELL_DECISION_OFFSET_CM;

    if (sl_align_applied) {
        s_chain_ref_cm      = here;
        tm_maze_residual_cm = 0.0f;
    }
    else {
        float err = s_chain_ref_cm - here;

        if (err >  NAV_RESIDUAL_LIMIT_CM) err =  NAV_RESIDUAL_LIMIT_CM;
        if (err < -NAV_RESIDUAL_LIMIT_CM) err = -NAV_RESIDUAL_LIMIT_CM;

        s_chain_ref_cm      = here + err;
        tm_maze_residual_cm = err;
    }

    /* The walls of the cell just entered, if enough rotations landed inside
     * the window. Too few is not a reason to guess -- the caller falls back to
     * stopping and voting, which is slow and right. */
    s_flight_ok = (sl_flight_samples >= WALL_FLIGHT_MIN_SAMPLES) ? 1U : 0U;

    if (s_flight_ok) {
        s_flight.w.front    = sl_flight_front;
        s_flight.w.left     = sl_flight_left;
        s_flight.w.right    = sl_flight_right;
        s_flight.front_mm   = sl_flight_front_mm;
        s_flight.left_mm    = sl_flight_left_mm;
        s_flight.right_mm   = sl_flight_right_mm;
        s_flight.front_votes = sl_flight_front_votes;
        s_flight.left_votes  = sl_flight_left_votes;
        s_flight.right_votes = sl_flight_right_votes;
        s_flight.samples     = sl_flight_samples;
    }

    return 1U;

#else

    /* Aim at one cell pitch PLUS whatever the last move left short, so the
     * shortfall is corrected instead of accumulating. */
    const float target_cm = NAV_CELL_CM + tm_maze_residual_cm;

    (void)cellContext(0.0f);
    WallFollow_SetCells(&s_cells);

    const uint8_t ok = runForwardFused(target_cm);

    s_align_reason   = sl_align_reason;
    s_align_applied  = sl_align_applied;
    s_align_delta_cm = sl_align_delta_cm;
    s_exit_err_mm    = sl_exit_err_mm;
    s_exit_valid     = sl_exit_valid;

    /* !! THE TWO CORRECTIONS MUST NOT BOTH FIRE !! -- see the note above. */
    float left_over = sl_align_applied
                    ? 0.0f
                    : (target_cm - Encoder_getAverageDistance());

    if (left_over >  NAV_RESIDUAL_LIMIT_CM) left_over =  NAV_RESIDUAL_LIMIT_CM;
    if (left_over < -NAV_RESIDUAL_LIMIT_CM) left_over = -NAV_RESIDUAL_LIMIT_CM;

    tm_maze_residual_cm = left_over;

    return ok;

#endif /* MAZE_CONTINUOUS_CELLS */
}
